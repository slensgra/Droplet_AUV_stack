#!/usr/bin/python3

import collections

import numpy as np
import cv_bridge
import rospy
import sensor_msgs.msg 
import geometry_msgs.msg
import cv2
from cv2 import aruco

import localization_informed_planning_sim.srv
import localization_informed_planning_sim.msg

class BreadcrumbLocalizer():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.camera_matrix = None
        self.distortion = np.zeros(4)
        self.global_positions_by_id = {
            0: np.array([0.0, 0.0, -99.8]),
            1: np.array([0.0, 0.4, -99.8]),
        }

    def set_marker_position(self, message):
        self.global_positions_by_id[message.marker_id] = np.array([
            message.x,
            message.y,
            message.z
        ])

        return localization_informed_planning_sim.srv.SetGlobalPositionResponse(
            success=True
        )

    def initialize_ros_node(self):
        rospy.init_node('breadcrumb_localizer')

        self.image_subscriber = rospy.Subscriber(
            '/rexrov/rexrov/camera/camera_image',
            sensor_msgs.msg.Image,
            self.image_callback,
            queue_size=1
        )

        self.camera_info_subscriber = rospy.Subscriber(
            '/rexrov/rexrov/camera/camera_info',
            sensor_msgs.msg.CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

        self.fused_position_publisher = rospy.Publisher(
            'fused_position',
            localization_informed_planning_sim.msg.BreadcrumbLocalizationResult
        )

        self.set_marker_position_service = rospy.Service(
            'set_marker_position',
            localization_informed_planning_sim.srv.SetGlobalPosition,
            self.set_marker_position
        )

    def camera_info_callback(self, camera_msg):
        self.camera_matrix = np.array(camera_msg.K).reshape((3,3))
        self.distortion = np.array(camera_msg.D)

    def get_relative_position_by_id(self, corners_by_id):
        side_length = 0.1778
        hl = side_length / 2.0

        local_corners = np.array([
            [-hl, hl, 0.0],
            [hl, hl, 0.0],
            [hl, -hl, 0.0],
            [-hl, -hl, 0.0]
        ])

        relative_positions_by_id = collections.defaultdict(lambda: [])

        for target_marker in corners_by_id.keys():
            for corners_reading in corners_by_id[target_marker]:
                corners_undist = cv2.undistortPoints(
                    corners_reading,
                    self.camera_matrix,
                    self.distortion,
                    R=np.eye(3),
                    P=self.camera_matrix
                )

                _, rvec, tvec = cv2.solvePnP(
                    local_corners,
                    corners_undist,
                    self.camera_matrix,
                    np.zeros((1,5)),
                    cv2.SOLVEPNP_IPPE,
                )

                relative_positions_by_id[target_marker] = relative_positions_by_id[target_marker] + [tvec]

        return relative_positions_by_id

    def image_callback(self, image_msg):
        #print("image!")
        dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters =  aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, parameters)

        corners_by_id = collections.defaultdict(lambda: [])

        img = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray_img)

        for marker_id, marker_corners in zip(np.ravel(ids), corners):
            winSize = (3, 3)
            zeroZone = (-1, -1)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 100, 0.0001)
            mcorners = cv2.cornerSubPix(gray_img, marker_corners, winSize, zeroZone, criteria)

            corners_by_id[marker_id] = corners_by_id[marker_id] + [mcorners]

        drawn_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)

        relative_position_by_id = self.get_relative_position_by_id(corners_by_id)

        #print(relative_position_by_id)
        cv2.imshow('frame_markers', drawn_markers)
        cv2.waitKey(10)

        if len(relative_position_by_id.keys()) == 0:
            return

        fused_position = self.get_fused_position(relative_position_by_id)

        fused_position_msg = localization_informed_planning_sim.msg.BreadcrumbLocalizationResult()
        fused_position_msg.num_visible_markers = len(relative_position_by_id.keys())

        fused_position_msg.position = geometry_msgs.msg.Point(
            x=fused_position[0],
            y=fused_position[1],
            z=fused_position[2]
        )

        self.fused_position_publisher.publish(fused_position_msg)

    def get_orthogonal_vector(self, vector):
        x_axis = np.array([1, 0, 0])
        y_axis = np.array([0, 1, 0])
        z_axis = np.array([0, 0, 1])

        candidate = np.cross(vector, x_axis)
        if np.linalg.norm(candidate) > 1e-3:
            return candidate / np.linalg.norm(candidate)

        candidate = np.cross(vector, y_axis)
        if np.linalg.norm(candidate) > 1e-3:
            return candidate / np.linalg.norm(candidate)

        candidate = np.cross(vector, z_axis)
        if np.linalg.norm(candidate) > 1e-3:
            return candidate / np.linalg.norm(candidate)

        raise Exception("Vector is colinear with all axes")

    def get_predicted_covariance_for_marker_relative_position(self, relative_position):
        small_eigval = 1e-4
        large_eigval = 1e-2

        largest_eigvec = relative_position / np.linalg.norm(relative_position)
        small_eigvec_1 = self.get_orthogonal_vector(largest_eigvec)
        small_eigvec_2 = np.cross(largest_eigvec, small_eigvec_1)
        small_eigvec_2 = small_eigvec_2 / np.linalg.norm(small_eigvec_2)

        S = np.hstack([
            largest_eigvec.reshape((3,1)),
            small_eigvec_1.reshape((3, 1)),
            small_eigvec_2.reshape((3,1))
        ])

        return np.matmul(
            S.transpose(), 
            np.matmul(
                np.diag([large_eigval, small_eigval, small_eigval]),
                S
            )
        )
    
    def fuse_positions(self, measured_positions, covariances):
        # assuming positions is of length 2

        if len(measured_positions) == 1:
            return measured_positions[0], covariances[0]

        #if len(measured_positions) != 2:
        #    raise Exception("Can only fuse two positions")
        
        #if len(covariances) != 2:
        #    raise Exception("Can only fuse two covariances")
        
        elif len(measured_positions) <= 4:
            K = np.matmul(covariances[0], np.linalg.inv(covariances[0] + covariances[1]))
            fused_covariance = covariances[1] - np.matmul(K, covariances[1])
            fused_position = measured_positions[0] + np.matmul(K, measured_positions[1] - measured_positions[0])

            return fused_position, fused_covariance

        else:
            rospy.logwarn(measured_positions)
            rospy.logwarn(covariances)
            raise Exception("Can only fuse two positions")

    def get_fused_position(self, relative_positions_by_id):
        measured_positions = []
        covariances = []

        for marker_id, position in relative_positions_by_id.items():
            position_formatted = position[0].flatten()
            marker_global_position = self.global_positions_by_id[marker_id]
            measured_position = marker_global_position + position_formatted
            covariance = self.get_predicted_covariance_for_marker_relative_position(position_formatted)
            measured_positions.append(measured_position)
            covariances.append(covariance)

        fused_position, fused_covariance = self.fuse_positions(measured_positions, covariances)
        return fused_position

if __name__ == '__main__':
    localizer = BreadcrumbLocalizer()
    localizer.initialize_ros_node()
    rospy.spin()
