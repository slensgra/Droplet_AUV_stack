#!/usr/bin/python
import copy
import numpy as np

import roslib
roslib.load_manifest('localization_informed_planning_sim')

import rospy
import actionlib
import sensor_msgs.msg
import mavros_msgs.msg
import nav_msgs.msg
from tf import transformations

from droplet_underwater_assembly_libs import trajectory_tracker, config, utils
import localization_informed_planning_sim.msg
import localization_informed_planning_sim.srv

# this class is also an actionlib server
class GlobalPositionController(object):
    def __init__(self):
        self.simulation_mode = rospy.get_param('~simulation_mode', False)
        self.dry_run_mode = rospy.get_param('~dry_run_mode', True)

        self.latest_ground_truth_position = None
        self.latest_breadcrumb_position = None
        self.latest_goal_position = None
        self.latest_position = None
        self.latest_imu = None
        self.n_thrusters = 8
        self.breadcrumb_position_topic = 'fused_position'
        self.imu_topic = config.IMU_TOPIC
        self.selected_location_source = 'breadcrumb'
        self.initial_goal_position = rospy.get_param('~initial_goal_position', None)
        rospy.loginfo("Moving to initial goal position {}".format(self.initial_goal_position))
        self.initial_goal_position = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]
        self.plunge_service_server = rospy.Service('plunge_action', localization_informed_planning_sim.srv.PlungeAction, self.do_plunge_action)

        if self.simulation_mode:
            self.imu_topic = '/rexrov/imu'
            self.selected_location_source = 'ground_truth'

        self.set_goal_position_server = rospy.Service(
            'set_goal_position',
            localization_informed_planning_sim.srv.SetControllerTarget,
            self.set_controller_target_callback
        )

        self.action_server = actionlib.SimpleActionServer(
            'move_to_position_server',
            localization_informed_planning_sim.msg.MoveToPositionAction,
            self.start_moving_to_position,
            False
        )

        self.imu_subscriber = rospy.Subscriber(
            self.imu_topic,
            sensor_msgs.msg.Imu,
            self.imu_callback,
            queue_size=1
        )

        self.state_publisher = rospy.Publisher(
            '~state',
            localization_informed_planning_sim.msg.GlobalPositionControllerState,
            queue_size=1
        )

        self.breadcrumb_subscriber = rospy.Subscriber(
            self.breadcrumb_position_topic,
            localization_informed_planning_sim.msg.BreadcrumbLocalizationResult,
            self.breadcrumb_callback,
        )

        if self.simulation_mode:
            self.initialize_simulation_mode()
        else:
            self.initialize_live_robot_mode()

        if self.initial_goal_position is not None:
            self.controller.set_goal_position(self.initial_goal_position)
            self.latest_goal_position = self.initial_goal_position

    def arm_if_necessary(self):
        if not self.simulation_mode and not self.dry_run_mode:
            rospy.logwarn("Warning! Arming motors...")
            utils.set_motor_arming(True)
            rospy.loginfo("Motors armed.")

    def initialize_live_robot_mode(self):
        self.rc_override_topic = '/mavros/rc/override'
        self.rc_override_publisher = rospy.Publisher(
            self.rc_override_topic,
            mavros_msgs.msg.OverrideRCIn,
            queue_size=1
        )

        self.pid_gains = dict(
            x_p=2.00,
            y_p=2.0,
            yaw_p=2.0, 
            x_d=-0.0, 
            y_d=-0.00,
            yaw_d=1.0,
            x_i=0.2,
            y_i=0.2,
            yaw_i=0.10,
            roll_p=1.0,
            roll_i=0.00,
            roll_d=-0.50,
            z_p=1.00,
            z_i=0.02,#config.DEFAULT_Z_I_GAIN,
            z_d=0.00,
            pitch_p=-1.0,
            pitch_i=-0.0,
            pitch_d=0.50,
        )

        self.controller = trajectory_tracker.PIDTracker(
            **self.pid_gains
        )
        temporary = list([-1.0 * i for i in self.controller.y_factor])
        self.controller.y_factor = self.controller.x_factor
        self.controller.x_factor = temporary


    def initialize_simulation_mode(self):
        import uuv_gazebo_ros_plugins_msgs.msg # import here because robot doesn't have this installed
        self.thruster_topic_name_format = '/rexrov/thrusters/{thruster_id}/input'
        self.ground_truth_position_topic = '/rexrov/pose_gt' # ground truth pose

        self.pid_gains = dict(
            x_p=1.0,
            y_p=1.0,
            z_p=1.0,
            x_i=0.0,
            y_i=0.0,
            z_i=0.0075,
            x_d=0.0,
            y_d=0.0,
            z_d=0.0,
            yaw_p=1.1,
            yaw_i=0.0001,
            yaw_d=0.0,
            pitch_p=0.0,
            pitch_i=0.0,
            pitch_d=0.0,
            roll_p=0.0,
            roll_i=0.0,
            roll_d=0.0
        )

        self.controller = trajectory_tracker.PIDTracker(
            **self.pid_gains
        )

        self.ground_truth_position_subscriber = rospy.Subscriber(
            ground_truth_position_topic,
            nav_msgs.msg.Odometry,
            self.global_position_callback,
            queue_size=1
        )

        self.thruster_publishers = [
            rospy.Publisher(
                thruster_topic_name_format.format(thruster_id=i),
                uuv_gazebo_ros_plugins_msgs.msg.FloatStamped,
                queue_size=1
            ) for i in range(n_thrusters)
        ]

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

    def set_controller_target_callback(self, request):
        rospy.loginfo("Setting controller target using using service call in GlobalPositionController")
        rospy.loginfo("Setting goal position to {}".format(request.target_position))
        self.latest_goal_position = request.target_position

        self.controller.set_goal_position(
            self.to_xyzrpy(self.latest_goal_position)
        )

        return True

    def wait_for_sensor_information(self):
        rospy.loginfo("Global position controller waiting for sensor data...")

        i = 0
        if self.simulation_mode:
            while not rospy.is_shutdown() and (self.latest_ground_truth_position is None or self.latest_imu is None):
                rospy.sleep(0.1)
                i = i + 1

                if i % 50 == 0:
                    rospy.loginfo("Global position controller waiting on simulation sensor information....")
            rospy.loginfo("Got necessary simulation position information! Starting...")
            return 
        else:
            while not rospy.is_shutdown() and (self.latest_imu is None or self.latest_breadcrumb_position is None):
               rospy.sleep(0.1)
               i = i + 1

               if i % 300 == 0:
                   rospy.loginfo("Global position controller waiting on live test sensor information....")
                   #rospy.loginfo("Latest breadcrumb: {}".format(self.latest_breadcrumb_position))
                   #rospy.loginfo("Latest imu: {}".format(self.latest_imu))
                   pass

            rospy.loginfo("Got sensor data!")
            rospy.loginfo("Latest breadcrumb: {}".format(self.latest_breadcrumb_position))
            rospy.loginfo("Latest imu: {}".format(self.latest_imu))

        rospy.sleep(5.0)

    def publish_state(self):
        error = self.controller.get_error()
        position = self.controller.current_position
        goal = self.controller.goal_position

        message = localization_informed_planning_sim.msg.GlobalPositionControllerState()
        message.error = error
        message.goal = goal
        message.current_position = position

        self.state_publisher.publish(message)

    def do_plunge_action(self, message):
        rospy.loginfo("Starting plunge for {} seconds".format(message.plunge_duration))
        self.controller.x_p = 0.0
        self.controller.x_i = 0.0
        self.controller.x_d = 0.0
        self.controller.y_p = 0.0
        self.controller.y_i = 0.0
        self.controller.y_d = 0.0
        #self.controller.z_p = 0.0
        self.controller.z_i = 0.0
        self.controller.z_d = 0.0

        previous_goal = copy.deepcopy(self.controller.goal_position)
        plunge_goal = copy.deepcopy(self.controller.goal_position)
        plunge_goal[2] = plunge_goal[2] - 0.4
        self.controller.set_goal_position(plunge_goal)
        rospy.sleep(message.plunge_duration)

        self.controller.set_goal_position(previous_goal)
        self.controller.x_p = self.pid_gains['x_p']
        self.controller.x_i = self.pid_gains['x_i']
        self.controller.x_d = self.pid_gains['x_d']
        self.controller.y_p = self.pid_gains['y_p']
        self.controller.y_i = self.pid_gains['y_i']
        self.controller.y_d = self.pid_gains['y_d']
        self.controller.z_p = self.pid_gains['z_p']
        self.controller.z_i = self.pid_gains['z_i']
        self.controller.z_d = self.pid_gains['z_d']
        rospy.loginfo("Completed plunge")

        return True

    def start_moving_to_position(self, goal):
        rospy.loginfo("gazebo position controller starting move to position, {}".format(goal))
        self.latest_goal_position = goal.target_position
        self.selected_location_source = goal.position_source
        self.controller.set_goal_position(
            self.to_xyzrpy(self.latest_goal_position)
        )

        while not rospy.is_shutdown() and not self.error_is_in_range(self.controller.get_error()):
            if self.action_server.is_preempt_requested():
                self.action_server.set_preempted()
                return

        print("Reached goal position")
        self.action_server.set_succeeded()

    def error_is_in_range(self, error):
        error_range = 0.05
        if abs(error[0]) < error_range and abs(error[1]) < error_range and abs(error[2]) < 100.0 and abs(error[5]) < 0.10:
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

        if self.selected_location_source == 'breadcrumb':
            r, p, _ = transformations.euler_from_quaternion([
                self.latest_imu.orientation.x,     
                self.latest_imu.orientation.y,     
                self.latest_imu.orientation.z,     
                self.latest_imu.orientation.w
            ])

            x = self.latest_breadcrumb_position.position.x
            y = self.latest_breadcrumb_position.position.y
            z = self.latest_breadcrumb_position.position.z
            #print 'p is {}'.format([x,y,z])
            yaw = self.latest_breadcrumb_position.relative_yaw

            return [x,y,z,r,p,yaw]

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

    def get_global_to_local_rotation(self, current_position):
        r, p, yaw = current_position[3:]
        return transformations.euler_matrix(0, 0, yaw, 'sxyz')[:3, :3]

    def publish_thruster_updates(self):
        current_position = self.get_latest_position_from_selected_source()

        if current_position is None:
            rospy.logwarn("No current position available")
            return

        self.controller.set_current_velocity([0.0, 0.0, 0.0, 0.0, 0.0, -self.latest_imu.angular_velocity.z])
        self.controller.set_current_position(
            current_position
        )

        self.controller.set_latest_imu_reading(self.latest_imu)

        if self.simulation_mode:
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
        else:
            rc_override_message = self.controller.get_next_rc_override()
            if not self.dry_run_mode:
                self.rc_override_publisher.publish(rc_override_message)
            else:
                #rospy.loginfo("Dry run. Overrides would be {}".format(rc_override_message))
                pass

    def breadcrumb_callback(self, msg):
        self.latest_breadcrumb_position = msg

    def global_position_callback(self, msg):
        self.latest_ground_truth_position = msg

    def run(self):
        self.action_server.start()
        rospy.loginfo("Starting controller run loop!")

        while not rospy.is_shutdown():
            if self.latest_goal_position is not None:
                #rospy.loginfo("Publishing to thrusters")
                self.publish_thruster_updates()
                self.publish_state()

            if not self.simulation_mode:
                max_breadcrumb_age = 5.0

                if self.latest_breadcrumb_position is not None and (rospy.Time.now() - self.latest_breadcrumb_position.header.stamp).to_sec() > max_breadcrumb_age:
                    rospy.logwarn("Terminating! No breadcrumb data for {} seconds".format(max_breadcrumb_age))
                    utils.set_motor_arming(False)
                    return

            rospy.sleep(0.03)

if __name__ == '__main__':
    rospy.init_node('ground_truth_position_controller')
    controller_node = GlobalPositionController()
    controller_node.wait_for_sensor_information()
    controller_node.arm_if_necessary()
    controller_node.run()
