import rospy
import sensor_msgs.msg
import nav_msgs.msg
from tf import transformations

import uuv_gazebo_ros_plugins_msgs.msg

from droplet_underwater_assembly_libs import trajectory_tracker

class GazeboPositionController(object):
    def __init__(self):
        thruster_topic_name_format = '/rexrov/thrusters/{thruster_id}/input'
        global_position_topic = '/rexrov/pose_gt' # ground truth pose
        imu_topic = '/rexrov/imu'

        n_thrusters = 8

        self.latest_imu = None

        pid_gains = dict(
            x_p=1.0,
            y_p=1.0,
            z_p=1.0,
            x_i=0.0,
            y_i=0.0,
            z_i=0.0,
            x_d=0.0,
            y_d=0.0,
            z_d=0.0,
            yaw_p=1.0,
            yaw_i=0.0,
            yaw_d=0.0,
            pitch_p=0.0,
            pitch_i=0.0,
            pitch_d=0.0,
            roll_p=0.0,
            roll_i=0.0,
            roll_d=0.0
        )

        self.controller = trajectory_tracker.PIDTracker(
            **pid_gains
        )

        self.controller.x_factor = [
            0.0, 0.0, 0.0, 0.0, 1.0, 1.0, -1.0, -1.0
        ]
        self.controller.z_factor = [
            1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0
        ]
        self.controller.y_factor = [
            0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, -1.0
        ]
        self.controller.yaw_factor = [
            0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 1.0, -1.0
        ]
        self.controller.roll_factor = [
            1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0
        ]

        self.imu_subscriber = rospy.Subscriber(imu_topic, sensor_msgs.msg.Imu, self.imu_callback, queue_size=1)

        self.thruster_publishers = [
            rospy.Publisher(
                thruster_topic_name_format.format(thruster_id=i),
                uuv_gazebo_ros_plugins_msgs.msg.FloatStamped,
                queue_size=1
            ) for i in range(n_thrusters)
        ]

        self.global_position_subscriber = rospy.Subscriber(
            global_position_topic,
            nav_msgs.msg.Odometry,
            self.global_position_callback,
            queue_size=1
        )

    def imu_callback(self, msg):
        self.latest_imu = msg

    def publish_thruster_updates(self, nav_msg):
        print("updating thrusters")
        if self.latest_imu is not None:

            r, p, yaw = transformations.euler_from_quaternion([
                nav_msg.pose.pose.orientation.x,
                nav_msg.pose.pose.orientation.y,
                nav_msg.pose.pose.orientation.z,
                nav_msg.pose.pose.orientation.w,
                'sxyz'
            ])

            print("Current yaw", yaw)
            print("current roll", r)
            self.controller.set_goal_position([0.0, 0.0, -98, 0.0, 0.0, 0.0])
            self.controller.set_current_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.controller.set_current_position([
                nav_msg.pose.pose.position.x,
                nav_msg.pose.pose.position.y, 
                nav_msg.pose.pose.position.z, 
                r, 
                p, 
                yaw
            ])

            self.controller.set_latest_imu_reading(self.latest_imu)

            zrp_thrust = self.controller.get_zrp_thrust_vector()
            xyyaw_thrust = self.controller.get_xyyaw_thrust_vector()
            thrust_vector = xyyaw_thrust + zrp_thrust

            motor_intensities = self.controller.convert_thrust_vector_to_motor_intensities(thrust_vector)
            print("Next intensitites", motor_intensities)

            for i, publisher in enumerate(self.thruster_publishers):
                message = uuv_gazebo_ros_plugins_msgs.msg.FloatStamped(
                    data=motor_intensities[i] * 1000.0
                )

                publisher.publish(
                    message
                )

    def global_position_callback(self, msg):
        self.publish_thruster_updates(msg)

    def run(self):
        rospy.init_node('ground_truth_position_controller')
        rospy.spin()

if __name__ == '__main__':
    controller_node = GazeboPositionController()
    controller_node.run()
