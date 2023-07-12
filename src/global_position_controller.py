#!/usr/bin/python3
import roslib
roslib.load_manifest('localization_informed_planning_sim')

import rospy
import actionlib
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
from tf import transformations

import uuv_gazebo_ros_plugins_msgs.msg

from droplet_underwater_assembly_libs import trajectory_tracker
import localization_informed_planning_sim.msg
import localization_informed_planning_sim.srv

# this class is also an actionlib server
class GazeboPositionController(object):
    def __init__(self):
        thruster_topic_name_format = '/rexrov/thrusters/{thruster_id}/input'
        ground_truth_position_topic = '/rexrov/pose_gt' # ground truth pose
        breadcrumb_position_topic = 'fused_position'
        imu_topic = '/rexrov/imu'

        self.latest_ground_truth_position = None
        self.latest_breadcrumb_position = None
        self.latest_goal_position = None

        self.set_goal_position_server = rospy.Service(
            '~set_goal_position',
            localization_informed_planning_sim.srv.SetControllerTarget,
            self.set_controller_target_callback
        )
        
        self.action_server = actionlib.SimpleActionServer(
            'move_to_position_server',
            localization_informed_planning_sim.msg.MoveToPositionAction,
            self.start_moving_to_position,
            False
        )

        self.target_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.latest_position = None

        n_thrusters = 8

        self.latest_imu = None

        pid_gains = dict(
            x_p=1.0,
            y_p=1.0,
            z_p=1.0,
            x_i=0.0,
            y_i=0.0,
            z_i=0.0075,
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

        self.state_publisher = rospy.Publisher(
            '~state',
            localization_informed_planning_sim.msg.GlobalPositionControllerState,
            queue_size=1
        )

        self.breadcrumb_subscriber = rospy.Subscriber(
            breadcrumb_position_topic,
            geometry_msgs.msg.Point,
            self.breadcrumb_callback,
        )

        self.ground_truth_position_subscriber = rospy.Subscriber(
            ground_truth_position_topic,
            nav_msgs.msg.Odometry,
            self.global_position_callback,
            queue_size=1
        )

    def set_controller_target_callback(self, request):
        self.latest_goal_position = request.target_position

    def wait_for_sensor_information(self):
        while not rospy.is_shutdown() and self.latest_ground_truth_position is None and self.latest_imu is None:
            rospy.sleep(0.1)

    def publish_state(self):
        error = self.controller.get_error()
        position = self.controller.current_position
        goal = self.controller.goal_position

        message = localization_informed_planning_sim.msg.GlobalPositionControllerState()
        message.error = error
        message.goal = goal
        message.current_position = position

        self.state_publisher.publish(message)

    def start_moving_to_position(self, goal):
        rospy.loginfo("gazebo position controller starting move to position, {}".format(goal))
        self.latest_goal_position = goal.target_position
        self.selected_location_source = goal.position_source

        self.controller.set_goal_position(
            self.to_xyzrpy(self.latest_goal_position)
        )
        current_position = self.get_latest_position_from_selected_source()
        self.controller.set_current_position(
            current_position
        )
        self.controller.set_latest_imu_reading(self.latest_imu)

        while not rospy.is_shutdown() and not self.error_is_in_range(self.controller.get_error()):
            self.publish_thruster_updates()
            self.publish_state()

            if self.action_server.is_preempt_requested():
                self.action_server.set_preempted()
                return

        print("Reached goal position")
        self.action_server.set_succeeded()

    def error_is_in_range(self, error):
        error_range = 0.15
        if abs(error[0]) < error_range and abs(error[1]) < error_range and abs(error[2]) < error_range:
            return True
        
        return False

    def imu_callback(self, msg):
        self.latest_imu = msg

    def get_latest_position_from_selected_source(self):
        if self.selected_location_source == 'ground_truth':
            if self.latest_ground_truth_position is None:
                return None

            r, p, yaw = transformations.euler_from_quaternion([
                self.latest_ground_truth_position.pose.pose.orientation.x,
                self.latest_ground_truth_position.pose.pose.orientation.y,
                self.latest_ground_truth_position.pose.pose.orientation.z,
                self.latest_ground_truth_position.pose.pose.orientation.w,
                'sxyz'
            ])

            translation = [ 
                self.latest_ground_truth_position.pose.pose.position.x,
                self.latest_ground_truth_position.pose.pose.position.y, # just a hack to make things consistent with the markers
                self.latest_ground_truth_position.pose.pose.position.z
            ]

            return [translation[0], translation[1], translation[2], r, p, yaw]

        raise Exception(":bbbbbbbbbbbbbbbbbbbbb")

    def to_xyzrpy(self, pose):
        result = [
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ] + list(transformations.euler_from_quaternion(
            [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            ]
        ))

        return result

    def publish_thruster_updates(self):
        current_position = self.get_latest_position_from_selected_source()

        if current_position is None:
            rospy.logwarn("No current position available")
            return

        self.controller.set_current_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.controller.set_current_position(
            current_position
        )

        error = self.controller.get_error()

        self.controller.set_latest_imu_reading(self.latest_imu)

        zrp_thrust = self.controller.get_zrp_thrust_vector()
        xyyaw_thrust = self.controller.get_xyyaw_thrust_vector()
        thrust_vector = xyyaw_thrust + zrp_thrust

        motor_intensities = self.controller.convert_thrust_vector_to_motor_intensities(thrust_vector)

        for i, publisher in enumerate(self.thruster_publishers):
            message = uuv_gazebo_ros_plugins_msgs.msg.FloatStamped(
                data=motor_intensities[i] * 1000.0
            )

            publisher.publish(
                message
            )

    def breadcrumb_callback(self, msg):
        self.latest_breadcrumb_position = [msg.x, msg.y, msg.z]

    def global_position_callback(self, msg):
        self.latest_ground_truth_position = msg

    def run(self):
        self.action_server.start()

        while not rospy.is_shutdown():
            if self.latest_goal_position is not None:
                if not self.action_server.is_active():
                    self.publish_thruster_updates()
                    self.publish_state()

            if self.action_server.is_new_goal_available():
                self.action_server.accept_new_goal()

            rospy.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('ground_truth_position_controller')
    controller_node = GazeboPositionController()
    controller_node.wait_for_sensor_information()
    controller_node.run()
