import rosbag
import numpy as np
import cv2
from cv2 import aruco
import collections
from tf import transformations

from matplotlib import pyplot as plt

bag = rosbag.Bag('/home/sam/initial_fisheye_positioning/day_2/test__2021-07-28-17-05-15.bag')

selected_marker = 2
y_values = []
x_values = []
z_values = []

camera_matrix = np.array([
    [394.21498, 0.01273, 722.66487], 
    [0., 393.4325, 536.59619], 
    [0., 0., 1.]
])

distortion = np.array([-0.000133, 0.007506, -0.000986, -0.000111])

m2_T_g = np.eye(4)
m2_T_g[0,3] = 0
m2_T_g[1,3] = -0.5
m2_T_g[2,3] = 0.8

def get_relative_position_by_id(corners_by_id):
    side_length = 0.1143
    hl = side_length / 2.0

    local_corners = np.array([
        [-hl, hl, 0.0],
        [hl, hl, 0.0],
        [hl, -hl, 0.0],
        [-hl, -hl, 0.0]
    ])

    relative_positions_by_id = collections.defaultdict(lambda: [])
    yaw_by_id = {}

    for target_marker in corners_by_id.keys():
        for corners_reading in corners_by_id[target_marker]:
            corners_undist = None

            #rospy.loginfo("Undistorting here....")
            #rospy.loginfo("Camera mat {}".format(self.camera_matrix))
            #rospy.loginfo("distortion {}".format(self.distortion))
            corners_undist = cv2.fisheye.undistortPoints(
                corners_reading,
                K=camera_matrix,
                D=distortion,
                #R=np.eye(3),
                #P=self.camera_matrix
            )

            _, rvec, tvec = cv2.solvePnP(
                objectPoints=local_corners,
                imagePoints=corners_undist,
                cameraMatrix=np.eye(3),#self.camera_matrix,
                distCoeffs=np.zeros((1,5)),
                #flags=cv2.SOLVEPNP_IPPE,
            )

            if tvec[2] > 0:
                #rotation_mat, _ = cv2.Rodrigues(rvec)
                #one_row = np.array([0.0, 0.0, 0.0, 1.0]).reshape((4, 1))
                #homogenous_rotation = np.vstack((rotation_mat, np.array([0.0, 0.0, 0.0])))
                #homogenous_rotation = np.hstack((homogenous_rotation, one_row))
                ##print(tvec)
                ##print(tvec.reshape(3))
                ##print('before', tvec)
                #translation_matrix = transformations.translation_matrix(tvec.reshape(3))

                #marker_in_cam_frame = transformations.concatenate_matrices(homogenous_rotation, translation_matrix)

                ##print('marker cam frame', transformations.decompose_matrix(marker_in_cam_frame)[3])
                #rot_inv = transformations.inverse_matrix(homogenous_rotation)
                #test = transformations.concatenate_matrices(rot_inv, translation_matrix)
                ##print('tvec', tvec)
                #_, _, euler_angles, translation, _ = transformations.decompose_matrix(test)
                ##print(translation)
                ##print('after', translation, euler_angles)

                ##_, _, relative_yaw = transformations.euler_from_matrix(homogenous_rotation, 'sxyz') 

                #relative_positions_by_id[target_marker] = relative_positions_by_id[target_marker] + [-translation]
                #yaw_by_id[target_marker] = euler_angles[2]
                rotation_mat, _ = cv2.Rodrigues(rvec)
                one_row = np.array([0.0, 0.0, 0.0, 1.0]).reshape((4, 1))
                homogenous_rotation = np.vstack((rotation_mat, np.array([0.0, 0.0, 0.0])))
                homogenous_rotation = np.hstack((homogenous_rotation, one_row))
                #print(tvec)
                #print(tvec.reshape(3))
                #print('before', tvec)
                translation_matrix = transformations.translation_matrix(tvec.reshape(3))

                marker_in_cam_frame = transformations.concatenate_matrices(homogenous_rotation, translation_matrix)

                #print('marker cam frame', transformations.decompose_matrix(marker_in_cam_frame)[3])
                rot_inv = transformations.inverse_matrix(homogenous_rotation)
                test = transformations.concatenate_matrices(rot_inv, translation_matrix)
                #print('tvec', tvec)
                _, _, euler_angles, translation, _ = transformations.decompose_matrix(test)
                #print(translation)
                #print('after', translation, euler_angles)

                #_, _, relative_yaw = transformations.euler_from_matrix(homogenous_rotation, 'sxyz') 

                #relative_positions_by_id[target_marker] = relative_positions_by_id[target_marker] + [-translation]
                #yaw_by_id[target_marker] = euler_angles[2]

                # Working directly with transformation matrices and the goal
                T = np.vstack((np.hstack((cv2.Rodrigues(rvec)[0], tvec)), np.array([0,0,0,1])))
                cTg = np.matmul(T, m2_T_g)

                relative_positions_by_id[target_marker] = relative_positions_by_id[target_marker] + [cTg[:3, 3]]

    return relative_positions_by_id, yaw_by_id

i = 0
for topic, compressed_image, t in bag.read_messages(topics=['/camera_array/cam1/image_raw/compressed']):
    if i > 200 and i < 3000:
        dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters =  aruco.DetectorParameters_create()
        raw_data = np.fromstring(compressed_image.data, np.uint8)
        gray_img = cv2.imdecode(raw_data, cv2.IMREAD_GRAYSCALE)

        corners_by_id = collections.defaultdict(lambda: [])

        #gray_img = cv2.cvtColor(gray_img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, dictionary, parameters=parameters)

        for marker_id, marker_corners in zip(np.ravel(ids), corners):
            winSize = (3, 3)
            zeroZone = (-1, -1)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 100, 0.0001)
            mcorners = cv2.cornerSubPix(gray_img, marker_corners, winSize, zeroZone, criteria)
            corners_by_id[marker_id] = corners_by_id[marker_id] + [mcorners]

        img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
        drawn_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)

        relative_position_by_id, yaw_by_id = get_relative_position_by_id(corners_by_id)

        if selected_marker in relative_position_by_id.keys():
            relative_positions = relative_position_by_id[selected_marker]
            x_values = x_values + [relative_positions[0][0]]
            y_values = y_values + [relative_positions[0][1]]
            z_values = z_values + [relative_positions[0][2]]

        cv2.imshow('frame', drawn_markers)
        cv2.waitKey(1)

    print(i)
    i += 1

plt.plot(y_values)
plt.show()