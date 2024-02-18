#!/usr/bin/python
import rospy

import sensor_msgs.msg
import stag_ros.msg

class ArucoDetectorNode():
    def __init__(self):
        self.image_topic = '/camera_array/cam1/image_raw'
        self.camera_info_topic = '/camera_array/cam1/camera_info'
        self.stag_message_output = '~/detected_markers'
        self.camera_matrix = None
        self.distortion = None

    def initialize_ros(self):
        self.image_subscriber = rospy.Subscriber(
            image_topic,
            sensor_msgs.msg.Image,
            self.image_callback,
            queue_size=1
        )

        self.camera_info_subscriber = rospy.Subscriber(
            self.camera_info_topic,
            sensor_msgs.msg.CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

        self.output_publisher = rospy.Publisher()

    def camera_info_callback(self, camera_msg):
        self.camera_matrix = np.array(camera_msg.K).reshape((3,3))
        self.distortion = np.array(camera_msg.D)[:4]

    def run_loop(self):
        pass 

    def image_callback(self):
        pass
