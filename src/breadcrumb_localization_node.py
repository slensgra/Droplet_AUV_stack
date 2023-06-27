import rospy
import sensor_msgs.msg 
import cv2
from cv2 import aruco

import cv_bridge

class BreadcrumbLocalizer():
    def __init__(self):
        pass
        self.bridge = cv_bridge.CvBridge()

    def initialize_ros_node(self):
        rospy.init_node('breadcrumb_localizer')

        self.image_subscriber = rospy.Subscriber(
            '/rexrov/rexrov/camera/camera_image',
            sensor_msgs.msg.Image,
            self.image_callback,
            queue_size=1
        )

    def image_callback(self, image_msg):
        print("image!")
        dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters =  aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, parameters)

        img = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray_img)
        frame_markers = aruco.drawDetectedMarkers(gray_img.copy(), corners, ids)
        cv2.imshow('frame_markers', frame_markers)
        cv2.waitKey(10)


if __name__ == '__main__':
    localizer = BreadcrumbLocalizer()
    localizer.initialize_ros_node()
    rospy.spin()