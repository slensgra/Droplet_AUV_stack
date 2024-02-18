#!/usr/bin/python
import math
import collections

import numpy as np
import cv_bridge
import rospy
import sensor_msgs.msg 
import geometry_msgs.msg
import cv2
from cv2 import aruco
from tf import transformations
import std_msgs.msg
#import splinter
#splinter.load(splinter.load("/usr/local/lib/libsplinter-3-0.so"))

import localization_informed_planning_sim.srv
import localization_informed_planning_sim.msg

class BreadcrumbLocalizer():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.camera_matrix = None
        self.distortion = np.zeros(4)
        self.large_eigval_scaling_factor = 1.0
        self.global_positions_by_id = {
            #2: np.array([0.0, 0.0, 0.0, 0.0]),
            3: np.array([0.0, 0.00, 0.0, 0.0]),
        }
        self.structure_yaw_offset = 0.0
        self.global_yaw_imu = 0.0
        self.masked_markers = set()
        self.visualization_mode = False
        output_file = "/home/sam/two_hop_from_robot_view_with_stats.avi"

        frame_width = 1440
        frame_height = 1080
        fps = 14

        fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        self.video_writer = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))

    def set_marker_position(self, message):
        self.global_positions_by_id[message.marker_id] = np.array([
            message.x,
            message.y,
            message.z,
            message.yaw
        ])

        if message.masked:
            self.masked_markers.add(message.marker_id)

        if not message.masked:
            if message.marker_id in self.masked_markers:
                self.masked_markers.remove(message.marker_id)

        return localization_informed_planning_sim.srv.SetGlobalPositionResponse(
            success=True
        )

    def imu_callback(self, message):
        quaternion = [
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w
        ]

        self.global_yaw_imu = transformations.euler_from_quaternion(quaternion, 'sxyz')[2]

    def initialize_ros_node(self):
        rospy.init_node('breadcrumb_localizer')

        self.visualization_mode = rospy.get_param('~visualization_mode', False)
        if self.visualization_mode:
            rospy.loginfo("Running in visualization mode!")

        self.cstar_publisher = rospy.Publisher('~cstar', std_msgs.msg.Float64, queue_size=1) 
        self.required_publisher = rospy.Publisher('~cstar_required', std_msgs.msg.Float64, queue_size=1) 

        self.spline_path = rospy.get_param('~spline_path', None)
        self.spline = None

        if self.spline_path is not None:
            self.spline = splinter.BSpline(self.spline_path)
            rospy.loginfo("Loaded spline! {}".format(self.spline_path))

        image_topic = rospy.get_param('~image_topic', '/rexrov/rexrov/camera/camera_image')
        self.image_subscriber = rospy.Subscriber(
            image_topic,
            sensor_msgs.msg.Image,
            self.image_callback,
            queue_size=1
        )

        if self.visualization_mode:
            pass

        cam_info_topic = rospy.get_param('~camera_info_topic', '/rexrov/rexrov/camera/camera_info')
        self.camera_info_subscriber = rospy.Subscriber(
            cam_info_topic,
            sensor_msgs.msg.CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

        self.fisheye_mode = rospy.get_param('~fisheye_mode', False)
        imu_topic = rospy.get_param('~imu_topic', '/mini_ahrs_ros/imu')
        self.imu_subscriber = rospy.Subscriber(
            imu_topic,
            sensor_msgs.msg.Imu,
            self.imu_callback,
            queue_size=1
        )

        rospy.loginfo("Breadcrumb localizer listening for images at {}".format(image_topic))
        rospy.loginfo("Breadcrumb localizer listening for camera info at {}".format(cam_info_topic))
        rospy.loginfo("Breadcrumb localizer fisheye mode {}".format(self.fisheye_mode))
        rospy.loginfo("Breadcrumb localizer listening for imu messages at {}".format(imu_topic))

        self.fused_position_publisher = rospy.Publisher(
            'fused_position',
            localization_informed_planning_sim.msg.BreadcrumbLocalizationResult,
            queue_size=1
        )

        self.set_marker_position_service = rospy.Service(
            'set_marker_position',
            localization_informed_planning_sim.srv.SetGlobalPosition,
            self.set_marker_position
        )

        if self.visualization_mode:
            self.camera_matrix = np.array([
                [394.21498, 0.01273, 722.66487],
                [0.0, 393.4325, 536.59619],
                [0.0, 0.0, 1.0]
            ])

            self.distortion = np.array([-0.000133, 0.007506, -0.000986, -0.000111])

    def camera_info_callback(self, camera_msg):
        self.camera_matrix = np.array(camera_msg.K).reshape((3,3))
        self.distortion = np.array(camera_msg.D)[:4]

    def get_relative_position_by_id(self, corners_by_id):
        side_length = 0.1143
        hl = side_length / 2.0

        local_corners = np.array([
            [-hl, hl, 0.0],
            [hl, hl, 0.0],
            [hl, -hl, 0.0],
            [-hl, -hl, 0.0]
        ])

        relative_positions_by_id = collections.defaultdict(lambda: [])
        tvecs_by_id = {}
        yaw_by_id = {}
        angles_by_id = {}

        for target_marker in corners_by_id.keys():
            if target_marker in self.masked_markers:
                continue

            if target_marker not in self.global_positions_by_id:
                continue

            for corners_reading in corners_by_id[target_marker]:
                corners_undist = None

                if not self.fisheye_mode:
                    corners_undist = cv2.undistortPoints(
                        corners_reading,
                        K=self.camera_matrix,
                        D=self.distortion,
                        R=np.eye(3),
                        P=self.camera_matrix
                    )
                else:
                    corners_undist = cv2.fisheye.undistortPoints(
                        corners_reading,
                        K=self.camera_matrix,
                        D=self.distortion,
                    )

                _, rvec, tvec = cv2.solvePnP(
                    objectPoints=local_corners,
                    imagePoints=corners_undist,
                    cameraMatrix=np.eye(3),
                    distCoeffs=np.zeros((1,5)),
                    #flags=cv2.SOLVEPNP_IPPE,
                )

                if tvec[2] > 0:
                    rotation_mat, _ = cv2.Rodrigues(rvec)
                    one_row = np.array([0.0, 0.0, 0.0, 1.0]).reshape((4, 1))
                    homogenous_rotation = np.vstack((rotation_mat, np.array([0.0, 0.0, 0.0])))
                    homogenous_rotation = np.hstack((homogenous_rotation, one_row))
                    translation_matrix = transformations.translation_matrix(-tvec.reshape(3))
                    #print(translation_matrix)

                    rot_inv = transformations.inverse_matrix(homogenous_rotation)
                    vehicle_in_marker_frame = transformations.concatenate_matrices(rot_inv, translation_matrix)
                    #vehicle_in_marker_frame = transformations.concatenate_matrices(homogenous_)
                    #print(transformations.decompose_matrix(vehicle_in_marker_frame)[3])
                    #_, _, euler_angles, translation, _ = transformations.decompose_matrix(vehicle_in_marker_frame)
                    world_to_marker = transformations.concatenate_matrices(
                        transformations.translation_matrix(self.global_positions_by_id[target_marker][:3]),
                        transformations.euler_matrix(0.0, 0.0, self.global_positions_by_id[target_marker][3], 'sxyz')
                    )
                    marker_to_world = transformations.inverse_matrix(world_to_marker)
                    vehicle_in_world = np.dot(world_to_marker, vehicle_in_marker_frame)
                    _, _, euler_angles, translation, _ = transformations.decompose_matrix(vehicle_in_world)
                    #print('vehicle_in_world', vehicle_in_world)
                    #print('eul', euler_angles)
                    #print('trans', translation)

                    tvecs_by_id[target_marker] = tvec.reshape(3)
                    relative_positions_by_id[target_marker] = [translation]#relative_positions_by_id[target_marker] + [-translation]
                    yaw_by_id[target_marker] = euler_angles[2]
                    angles_by_id[target_marker] = euler_angles
                else:
                    rospy.logwarn("Invalid marker reading")

        return relative_positions_by_id, yaw_by_id, tvecs_by_id, angles_by_id

    def image_callback(self, image_msg):
        dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters =  aruco.DetectorParameters_create()
        #detector = aruco.ArucoDetector(dictionary)#, parameters)

        corners_by_id = collections.defaultdict(lambda: [])

        gray_img = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        #gray_img = cv2.cvtColor(gray_img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, dictionary, parameters=parameters)

        for marker_id, marker_corners in zip(np.ravel(ids), corners):
            winSize = (3, 3)
            zeroZone = (-1, -1)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 100, 0.0001)
            mcorners = cv2.cornerSubPix(gray_img, marker_corners, winSize, zeroZone, criteria)
            corners_by_id[marker_id] = corners_by_id[marker_id] + [mcorners]

        relative_position_by_id, yaw_by_id, tvec_by_id, angles_by_id = self.get_relative_position_by_id(corners_by_id)

        if len(relative_position_by_id.keys()) == 0:
            return

        #print(relative_position_by_id)

        #print("relative positions {}".format(relative_position_by_id))
        average_yaw = sum([i for i in yaw_by_id.values()]) / float(len(yaw_by_id.keys()))
        average_roll = sum([i for i in [i[0] for i in angles_by_id.values()]]) / float(len(angles_by_id.keys()))
        average_pitch = sum([i for i in [i[1] for i in angles_by_id.values()]]) / float(len(angles_by_id.keys()))

        fused_position, fused_covariance = self.get_fused_position(relative_position_by_id, tvec_by_id)

        fused_position_msg = localization_informed_planning_sim.msg.BreadcrumbLocalizationResult()
        fused_position_msg.num_visible_markers = len(relative_position_by_id.keys())
        fused_position_msg.header.stamp = rospy.Time.now()

        fused_position_msg.position = geometry_msgs.msg.Point(
            x=fused_position[0],
            y=fused_position[1],
            z=fused_position[2]
        )
        fused_position_msg.structure_yaw = self.global_yaw_imu - average_yaw
        fused_position_msg.relative_yaw = average_yaw
        fused_position_msg.relative_roll = average_roll
        fused_position_msg.relative_pitch = average_pitch

        self.fused_position_publisher.publish(fused_position_msg)

        if self.visualization_mode:
            img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)

            max_eig = np.max(np.linalg.eigvals(fused_covariance))
            cstar = math.pow(math.erf(0.02 / (math.sqrt(max_eig) * math.sqrt(2.0))), 3)
            self.cstar_publisher.publish(cstar)
            self.required_publisher.publish(0.92)

            if 2 in tvec_by_id:
                m2_pos = tvec_by_id[2]
                max_eig = self.spline.eval(m2_pos)[0] * self.large_eigval_scaling_factor
                cstar2 = math.pow(math.erf(0.02 / (math.sqrt(max_eig) * math.sqrt(2.0))), 3)
                line2 = "    2: [{:.2f}, {:.2f}, {:.2f}] C* alone: {:.6f}".format(m2_pos[0], m2_pos[1], m2_pos[2], cstar2)
            else:
                m2_pos = [0,0,0]
                line2 = "    2:".format(m2_pos[0], m2_pos[1], m2_pos[2])
            if 3 in tvec_by_id:
                m3_pos = tvec_by_id[3]
                max_eig = self.spline.eval(m3_pos)[0] * self.large_eigval_scaling_factor
                cstar3 = math.pow(math.erf(0.02 / (math.sqrt(max_eig) * math.sqrt(2.0))), 3)
                line3 = "    3: [{:.2f}, {:.2f}, {:.2f}] C* alone: {:.6f}".format(m3_pos[0], m3_pos[1], m3_pos[2], cstar3)
            else:
                m3_pos = [0,0,0]
                line3 = "    3:".format(m3_pos[0], m3_pos[1], m3_pos[2])

            line1 = "Marker positions:"
            line4 = "Fused C* {:.6f}".format(cstar)
            font = cv2.FONT_HERSHEY_SIMPLEX
            position = (50, 50)  # Coordinates (x, y) where the text will be placed
            pos2 = (50, 100)
            pos3 = (50, 150)
            pos4 = (50, 200)
            font_scale = 1.00
            font_color = (0, 255, 0)  # BGR color tuple (green in this case)
            font_thickness = 2

            # Add the text to the image
            cv2.putText(img, line1, position, font, font_scale, font_color, font_thickness)
            cv2.putText(img, line2, pos2, font, font_scale, font_color, font_thickness)
            cv2.putText(img, line3, pos3, font, font_scale, font_color, font_thickness)
            cv2.putText(img, line4, pos4, font, font_scale, font_color, font_thickness)

            drawn_markers = aruco.drawDetectedMarkers(img, corners, ids)
            self.video_writer.write(drawn_markers)
            cv2.imshow('frame_markers', drawn_markers)
            cv2.waitKey(1)

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
        small_eigval = 1e-6
        if self.spline is None:
            large_eigval = 1e-2
        else:
            large_eigval = self.spline.eval(relative_position)[0] * self.large_eigval_scaling_factor
            #print("lev", large_eigval)
            #print("got eig from relpos", large_eigval)

        eigval_matrix = np.array([
            [large_eigval, 0.0, 0.0],
            [0.0, small_eigval, 0.0],
            [0.0, 0.0, small_eigval]
        ])

        largest_eigvec = relative_position / np.linalg.norm(relative_position)
        eigvec_1 = np.cross(largest_eigvec, np.array([largest_eigvec[0], largest_eigvec[1] + 0.5, largest_eigvec[2]]))
        eigvec_2 = np.cross(largest_eigvec, eigvec_1)

        eigvec_1 = eigvec_1 / np.linalg.norm(eigvec_1)
        eigvec_2 = eigvec_2 / np.linalg.norm(eigvec_2)

        largest_eigvec = largest_eigvec[..., None]
        eigvec_1 = eigvec_1[..., None]
        eigvec_2 = eigvec_2[..., None]

        S = np.hstack([largest_eigvec, eigvec_1, eigvec_2])
        return S.dot(eigval_matrix).dot(np.linalg.inv(S))

        #largest_eigvec = relative_position / np.linalg.norm(relative_position)
        #small_eigvec_1 = self.get_orthogonal_vector(largest_eigvec)
        #small_eigvec_2 = np.cross(largest_eigvec, small_eigvec_1)
        #small_eigvec_2 = small_eigvec_2 / np.linalg.norm(small_eigvec_2)

        #S = np.hstack([
        #    largest_eigvec.reshape((3,1)),
        #    small_eigvec_1.reshape((3,1)),
        #    small_eigvec_2.reshape((3,1))
        #])
        #print "Largest eigvec {}".format(largest_eigvec)
        #print "small eigvec1 {}".format(small_eigvec_1)
        #print "small eigvec2 {}".format(small_eigvec_2)
        #print "S is {}".format(S)
        #eigval_matrix = np.diag([large_eigval, small_eigval, small_eigval])

        #return S.dot(eigval_matrix).dot(np.linalg.inv(S))

    def fuse_positions(self, measured_positions, covariances):
        # assuming positions is of length 2
        #print("Fusing {}".format(measured_positions))

        if len(measured_positions) == 1:
            return measured_positions[0], covariances[0]

        #if len(measured_positions) != 2:
        #    raise Exception("Can only fuse two positions")
        
        #if len(covariances) != 2:
        #    raise Exception("Can only fuse two covariances")
        
        elif len(measured_positions) == 2:
            #print("Fusing here!")
            K = np.matmul(covariances[0], np.linalg.inv(covariances[0] + covariances[1]))
            fused_covariance = covariances[0] - np.matmul(K, covariances[0])
            fused_position = measured_positions[0] + np.matmul(K, measured_positions[1] - measured_positions[0])

            return fused_position, fused_covariance
        else:
            rospy.logwarn(measured_positions)
            rospy.logwarn(covariances)
            raise Exception("Can only fuse two positions")

    def get_fused_position(self, relative_positions_by_id, tvecs_by_id):
        measured_positions = []
        covariances = []

        for marker_id, measured_position in relative_positions_by_id.items():
            position_formatted = measured_position[0].flatten()
            #marker_global_position = self.global_positions_by_id[marker_id][:3]
            #measured_position = marker_global_position + position_formatted
            covariance = self.get_predicted_covariance_for_marker_relative_position(tvecs_by_id[marker_id])
            measured_positions.append(position_formatted)
            covariances.append(covariance)
        
        #print("Measured positions", measured_positions)
        fused_position, fused_covariance = self.fuse_positions(measured_positions, covariances)
        #print("Result {}".format(fused_position))
        return fused_position, fused_covariance


if __name__ == '__main__':
    localizer = BreadcrumbLocalizer()
    localizer.initialize_ros_node()
    rospy.spin()
    localizer.video_writer.release()