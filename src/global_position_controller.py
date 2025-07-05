#!/usr/bin/python
import copy
import numpy as np
import json

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

from localization_informed_planning_sim_libs.ui_tube_pattern_publisher import UITubePatternFlipper

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
        self.breadcrumb_position_topic = '/fused_position'
        self.imu_topic = config.IMU_TOPIC
        self.selected_location_source = 'breadcrumb'
        self.initial_goal_position = rospy.get_param('~initial_goal_position', None)
        rospy.loginfo("Moving to initial goal position {}".format(self.initial_goal_position))
        self.initial_goal_position = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0]
        self.plunge_service_server = rospy.Service('plunge_action', localization_informed_planning_sim.srv.PlungeAction, self.do_plunge_action)
        self.outside_of_operational_range = False

        self.velocity_history_length_for_smoothing = 2
        self.velocity_history = [
            [0.0, 0.0, 0.0] for _ in range(2)
        ]
        self.latest_smoothed_velocity = []

        #self.error_state_flipper = UITubePatternFlipper(
        #    patterns=[
        #        {'pattern': 'chase', 'type': 'led', 'r': 255, 'g': 0, 'b': 0},
        #        {'pattern': 'solid', 'type': 'led', 'r': 255, 'g': 0, 'b': 0},
        #    ],
        #    flip_time=1.0
        #) 

        self.max_breadcrumb_age = 2.0

        #self.okay_state_flipper = UITubePatternFlipper(
        #    patterns=[
        #        {'pattern': 'chase', 'type': 'led', 'r': 0, 'g': 255, 'b': 0},
        #        {'pattern': 'solid', 'type': 'led', 'r': 0, 'g': 255, 'b': 0},
        #    ],
        #    flip_time=1.0
        #)

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
            '/controller_state',
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

    def update_ui_tube(self):
        pass

    def initialize_live_robot_mode(self):
        self.rc_override_topic = '/mavros/rc/override'
        self.rc_override_publisher = rospy.Publisher(
            self.rc_override_topic,
            mavros_msgs.msg.OverrideRCIn,
            queue_size=1
        )

        self.pid_gains = dict(
            x_p=2.50,
            y_p=2.5,
            yaw_p=2.0, 
            x_d=-0.0, 
            y_d=-0.00,
            yaw_d=1.0,
            x_i=config.DEFAULT_X_I_GAIN / 100.0,
            y_i=config.DEFAULT_Y_I_GAIN / 100.0,
            yaw_i=0.10,
            roll_p=1.0,
            roll_i=0.03,
            roll_d=-0.50,
            z_p=1.00,
            z_i=0.05,#config.DEFAULT_Z_I_GAIN,
            z_d=0.00,
            pitch_p=-1.0,
            pitch_i=-0.03,
            pitch_d=0.50,
        )

        # top 
        #tfr 1 down
        #tbl 2 up
        #tfl 4 up
        #tbr 6 down

        #bbr 0 cw
        #bbl 3 ccw
        #bfl 5 cw
        #bfr 7 ccw
        # barbados
        #self.pid_gains = dict(
        #    x_p=2.3,
        #    y_p=2.3,
        #    yaw_p=-2.50,
        #    x_d=0.45, 
        #    y_d=0.45,
        #    yaw_d=-1.2,
        #    x_i=-0.05,
        #    y_i=-0.05,
        #    yaw_i=-0.05,#0.010,
        #    roll_p=-0.5,#0.0,
        #    roll_i=-0.01,
        #    roll_d=-0.1,#-0.50,
        #    z_p=-2.50,
        #    z_i=-0.15,
        #    z_d=0.5,
        #    pitch_p=-0.50,#-0.1,#1.0,
        #    pitch_i=-0.01,
        #    pitch_d=-0.1,#-0.50,
        #)

        # testing
        #self.pid_gains = dict(
        #    x_p=1.00,
        #    y_p=1.0,
        #    yaw_p=-1.00,
        #    x_d=0.0, 
        #    y_d=0.00,
        #    yaw_d=-0.0,
        #    x_i=-0.00,
        #    y_i=-0.00,
        #    yaw_i=-0.000,#0.010,
        #    roll_p=-0.00,#0.0,
        #    roll_i=-0.00,
        #    roll_d=0.0,#-0.50,
        #    z_p=-2.00,
        #    z_i=-0.0,
        #    z_d=0.00,
        #    pitch_p=-0.00,#-0.1,#1.0,
        #    pitch_i=-0.000,
        #    pitch_d=0.0,#-0.50,
        #)

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

        xyyaw_thrust = self.controller.get_xyyaw_thrust_vector()
        zrp_thrusts = self.controller.get_zrp_thrust_vector()
        error_integral = self.controller.error_integral
        controller_velocity = self.controller.current_velocity

        motor_intensities = self.controller.convert_thrust_vector_to_motor_intensities(
            xyyaw_thrust + zrp_thrusts
        )

        debug_data_dict = {
            'xyyaw_thrusts': xyyaw_thrust,
            'zrp_thrusts': zrp_thrusts,
            'error_integral': error_integral,
            'controller_velocity': controller_velocity,
            'motor_intensities': motor_intensities,
            'angle_error_from_imu': self.controller.get_angle_error_from_imu_reading(),
            'roll_pitch_velocity': [
                self.controller.latest_imu_reading.angular_velocity.y,
                self.controller.latest_imu_reading.angular_velocity.x
            ],
            'motor_speeds': self.controller.convert_motor_intensities_to_pwms(motor_intensities),
            'pid_gains': self.controller.get_gains_as_dict()
        }

        message = localization_informed_planning_sim.msg.GlobalPositionControllerState()
        message.error = error
        message.goal = goal
        message.current_position = position
        message.debug_data = json.dumps(debug_data_dict)

        self.state_publisher.publish(message)

    def do_plunge_action(self, message):
        rospy.loginfo("Starting plunge for {} seconds".format(message.plunge_duration))
        #self.controller.x_p = 0.0
        #self.controller.x_i = 0.0
        #self.controller.x_d = 0.0
        #self.controller.y_p = 0.0
        #self.controller.y_i = 0.0
        #self.controller.y_d = 0.0
        #self.controller.z_p = 0.0
        self.controller.z_i = 0.0
        #self.controller.z_d = 0.0
        plunge_depth = 0.125

        previous_goal = copy.deepcopy(self.controller.goal_position)
        plunge_goal = copy.deepcopy(self.controller.goal_position)
        plunge_goal[2] = plunge_goal[2] - plunge_depth
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
        #self.controller.clear_error_integrals(prev_buoyancy_input=None, axes=[2])

        if message.grasping_block:
            buoyancy_input = 8.0
        else:
            buoyancy_input = None

        self.controller.clear_error_integrals(prev_buoyancy_input=buoyancy_input)

        return True

    def start_moving_to_position(self, goal):
        rospy.loginfo("Global position controller starting move to position, {}".format(goal))
        self.latest_goal_position = goal.target_position
        self.selected_location_source = goal.position_source
        self.controller.set_goal_position(
            self.to_xyzrpy(self.latest_goal_position)
        )

        if goal.clear_error_integrals:
            rospy.loginfo("Clearing error integrals in PID controller")
            self.controller.clear_error_integrals()

        i = 0
        while self.controller.current_position is None or self.controller.goal_position is None:
            if i % 1000 == 0:
                rospy.logwarn("Controller waiting for valid sensor information")

            i = i + 1
            rospy.sleep(0.1)

        i = 0
        while not rospy.is_shutdown() and not self.error_is_in_range(self.controller.get_error()):
            if self.action_server.is_preempt_requested():
                self.action_server.set_preempted()
                return

            if not self.breadcrumb_data_is_valid():
                rospy.logwarn("aborting! No breadcrumb data for {} seconds".format(self.max_breadcrumb_age))
                #utils.set_motor_arming(False)
                self.action_server.set_aborted()
                return
            
            i = i + 1
            if i % 200 == 0:
                rospy.loginfo('move error {}'.format(self.controller.get_error()))

            rospy.sleep(0.02)

            #if self.outside_of_operational_range:
            #    self.error_state_flipper.update()
            #else:
            #    self.okay_state_flipper.update()

        rospy.loginfo("Reached goal position")
        rospy.loginfo("i {}".format(self.controller.error_integral))
        self.action_server.set_succeeded()

    def error_is_in_range(self, error):
        error_range = 0.05
        if abs(error[0]) < error_range and abs(error[1]) < error_range and abs(error[2]) < error_range*3.0 and abs(error[5]) < 0.05:
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
            #r, p, _ = transformations.euler_from_quaternion([
            #    self.latest_imu.orientation.x,     
            #    self.latest_imu.orientation.y,     
            #    self.latest_imu.orientation.z,     
            #    self.latest_imu.orientation.w
            #])
            r = self.latest_breadcrumb_position.relative_roll
            p = self.latest_breadcrumb_position.relative_pitch

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

        self.controller.set_current_velocity(
            [
                self.latest_smoothed_velocity[0], 
                self.latest_smoothed_velocity[1], 
                self.latest_smoothed_velocity[2], 
                0.0, 
                0.0, 
                -self.latest_imu.angular_velocity.z
            ]
        )

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

            if self.error_is_too_large(self.controller.get_error()):
                rospy.logwarn("Error is too large! Publishing stop message.")
                self.outside_of_operational_range = True
                rc_override_message = utils.construct_stop_rc_message()
            else:
                self.outside_of_operational_range = False

            if not self.dry_run_mode:
                self.rc_override_publisher.publish(rc_override_message)
            else:
                #rospy.loginfo("Dry run. Overrides would be {}".format(rc_override_message))
                pass

    def error_is_too_large(self, error):
        return np.linalg.norm(error[0:3]) > 1.75

    def update_smoothed_velocity(self, breadcrumb_msg):
        elapsed_time = (breadcrumb_msg.header.stamp - self.latest_breadcrumb_position.header.stamp).to_sec()

        current_velocity = (np.array(
            [
                breadcrumb_msg.position.x,
                breadcrumb_msg.position.y,
                breadcrumb_msg.position.z
            ]
        ) - np.array(
            [
                self.latest_breadcrumb_position.position.x,
                self.latest_breadcrumb_position.position.y,
                self.latest_breadcrumb_position.position.z,
            ]
        )) / elapsed_time

        self.velocity_history.append(current_velocity)

        if len(self.velocity_history) > self.velocity_history_length_for_smoothing:
            self.velocity_history.pop(0)

        mean_velocity = np.mean(
            self.velocity_history,
            axis=0
        )

        self.latest_smoothed_velocity = [
            mean_velocity[0],
            mean_velocity[1],
            mean_velocity[2],
            0,
            0,
            0,
        ]

    def breadcrumb_callback(self, msg):
        if self.latest_breadcrumb_position is not None:
            self.update_smoothed_velocity(msg)

        self.latest_breadcrumb_position = msg

    def global_position_callback(self, msg):
        self.latest_ground_truth_position = msg

    def breadcrumb_data_is_valid(self):
        #print('latest', self.latest_breadcrumb_position)
        if self.latest_breadcrumb_position is not None:
            breadcrumb_age = (rospy.Time.now() - self.latest_breadcrumb_position.header.stamp).to_sec()

            if breadcrumb_age < self.max_breadcrumb_age:
                #print('breadcrumb age: {}'.format(self.max_breadcrumb_age))
                return True
        
        return False

    def run(self):
        self.action_server.start()
        rospy.loginfo("Starting controller run loop!")

        i = 0
        while not rospy.is_shutdown():
            if self.latest_goal_position is not None:
                if not self.simulation_mode:
                    if self.breadcrumb_data_is_valid():
                        self.publish_thruster_updates()
                        self.publish_state()
                        i = 0
                    else:
                        if i % 10000 == 0:
                            rospy.logwarn("Not publishing to thrusters. No valid breadcrumb")
                        i = i + 1

            rospy.sleep(0.03)

if __name__ == '__main__':
    rospy.init_node('ground_truth_position_controller')
    controller_node = GlobalPositionController()
    controller_node.wait_for_sensor_information()
    #controller_node.arm_if_necessary()
    controller_node.run()
