import math
import rospy
import numpy as np

import smach
import smach_ros
import actionlib

import localization_informed_planning_sim.msg
import droplet_underwater_assembly_libs.utils
from localization_informed_planning_sim_libs.custom_smach_states import PauseState, UpdateMarkerPositionState


# camera center to vehicle center = 20 in right
# camera to vehicle center 6 in back
class RealWorldBuildPlanConstructor(object):

    def __init__(self):
        self.camera_center_right_meters = (20.0 * 0.0254) + 0.02
        self.camera_center_back_meters = (6.0 * 0.0254) - 0.10
        self.camera_center_up_meters = 25.0 * 0.0254
        self.slot_stride = 0.205
        self.move_action_client = actionlib.SimpleActionClient(
            'move_to_position_server',
            localization_informed_planning_sim.msg.MoveToPositionAction
        )

        self.actuate_gripper_client = actionlib.SimpleActionClient(
            'actuate_gripper',
            localization_informed_planning_sim.msg.ActuateGripperAction
        )

    def get_position_from_slot_index(self, index):
        cell_offset = np.array([
            (self.cell_side_length) / 2.0,
            (self.cell_side_length) / 2.0, 
            (self.cell_side_length) / 2.0
        ])
        cell_offset = cell_offset - np.array([0.0, 0.0, 0.0])

        return (index) + cell_offset

    def get_manipulator_test_state_machine(self):
        state_machine = smach.StateMachine(['succeeded', 'aborted', 'preempted'])
        action_number = 0
        goal_positions = [
            [(-4.0 * self.slot_stride) + self.camera_center_right_meters , self.camera_center_back_meters, 0.58, 0.0, 0.0, 0.0],
        ]

        with state_machine:
            move_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                    goal_positions[0],
                    frame_id='world',
                    seq=0,
                    stamp=rospy.Time.now()
                ),
                position_source='breadcrumb'
            )

            smach.StateMachine.add(
                'MOVE_{}'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'PAUSE_{}a'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'PAUSE_{}a'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'MOVE_{}p'.format(str(action_number))}
            )

            smach.StateMachine.add(
                'MOVE_{}p'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'PAUSE_{}c'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'PAUSE_{}c'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'CLOSE_GRIPPER_{}'.format(str(action_number))}
            )

            close_gripper_goal = localization_informed_planning_sim.msg.ActuateGripperGoal(position='close')
            smach.StateMachine.add(
                'CLOSE_GRIPPER_{}'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'actuate_gripper',
                    localization_informed_planning_sim.msg.ActuateGripperAction,
                    goal=close_gripper_goal
                ),
                transitions={'succeeded': 'PAUSE_{}'.format(str(action_number))},
            )
            smach.StateMachine.add(
                'PAUSE_{}'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'MOVE_{}a'.format(str(action_number))}
            )

            smach.StateMachine.add(
                'MOVE_{}a'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'PAUSE_{}b'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'PAUSE_{}b'.format(str(action_number)),
                PauseState(15.0),
                transitions={'succeeded': 'OPEN_GRIPPER_{}'.format(str(action_number))}
            )

            open_gripper_goal = localization_informed_planning_sim.msg.ActuateGripperGoal(position='open')
            smach.StateMachine.add(
                'OPEN_GRIPPER_{}'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'actuate_gripper',
                    localization_informed_planning_sim.msg.ActuateGripperAction,
                    goal=open_gripper_goal
                ),
            )

        return state_machine

    def get_controller_test_state_machine(self):
        state_machine = smach.StateMachine(['succeeded', 'aborted', 'preempted'])

        action_number = 0
        pause_time = 15.0
        goal_positions = [
            [(-1.0 * self.slot_stride) - self.camera_center_right_meters , -self.camera_center_back_meters, 0.58, 0.0, 0.0, math.pi],
            #[0.0, 0.0, 0.8, 0.0, 0.0, 0.0],
            #[0.0, 0.0, 0.8, 0.0, 0.0, math.pi / 4.0],
        ]
        with state_machine:
            for i, goal_pose in enumerate(goal_positions):
                move_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                    target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                        goal_pose,
                        frame_id='world',
                        seq=0,
                        stamp=rospy.Time.now()
                    ),
                    position_source='breadcrumb'
                )

                smach.StateMachine.add(
                    'MOVE_{}'.format(str(action_number)),
                    smach_ros.SimpleActionState(
                        'move_to_position_server',
                        localization_informed_planning_sim.msg.MoveToPositionAction, 
                        goal=move_goal
                    ),
                    transitions={'succeeded': 'PAUSE_{}'.format(str(action_number))},
                )

                next_state = 'MOVE_{}'.format(str(action_number+1))
                if i == len(goal_positions)-1:
                    next_state = ''

                smach.StateMachine.add(
                    'PAUSE_{}'.format(str(action_number)),
                    PauseState(pause_time),
                    transitions={'succeeded': next_state}
                )
                action_number = action_number + 1

        return state_machine

    def one_hop_plan(self):
        state_machine = smach.StateMachine(['succeeded', 'aborted', 'preempted'])
        action_number = 0
        goal_positions = [
            [(-1.0 * self.slot_stride) - self.camera_center_right_meters , -self.camera_center_back_meters - 0.10, 0.53, 0.0, 0.0, math.pi],
            [(-8.0 * self.slot_stride) + self.camera_center_right_meters + 0.06 , self.camera_center_back_meters, 0.53, 0.0, 0.0, 0.0],
            [(-4.0 * self.slot_stride) - self.camera_center_right_meters , -self.camera_center_back_meters - 0.10, 0.53, 0.0, 0.0, math.pi],
            [(-13.0 * self.slot_stride) + self.camera_center_right_meters + 0.06, self.camera_center_back_meters, 0.53, 0.0, 0.0, 0.0],
        ]

        intermediates = [
            [(-1.0 * self.slot_stride) - self.camera_center_right_meters , -self.camera_center_back_meters, 0.7, 0.0, 0.0, math.pi],
            [(-3.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi - (math.pi/6.0)],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi - (math.pi/4.0)],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi - (math.pi/3.0)],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi - (math.pi/2.0)],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi/3.0],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi/4.0],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi/6.0],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, 0.0],
        ]

        # drop 1 to pickup 2
        intermediates_2 = [
            [(-8.0 * self.slot_stride) + self.camera_center_right_meters , self.camera_center_back_meters, 0.7, 0.0, 0.0, 0.0],
            #[(-8.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, 0.0],
            [(-6.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, 0.0],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, 0.0],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi / 6.0],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi / 4.0],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi / 3.0],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi / 2.0],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, (4.0*math.pi) / 6.0],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, (3.0*math.pi) / 4.0],
            [(-5.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi],
            [(-4.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi],
            [(-4.0 * self.slot_stride) - self.camera_center_right_meters , -self.camera_center_back_meters - 0.10, 0.7, 0.0, 0.0, math.pi],
        ]

        # pickup 2 to drop 2
        intermediates_3 = [
            [(-4.0 * self.slot_stride) - self.camera_center_right_meters , -self.camera_center_back_meters - 0.10, 0.7, 0.0, 0.0, math.pi],
            #[(-4.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi],
            [(-6.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi],
            [(-8.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi],
            [(-9.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi],
            [(-9.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi - (math.pi / 6.0)],
            [(-9.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi - (math.pi / 4.0)],
            [(-9.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi - (math.pi / 3.0)],
            [(-9.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi - (math.pi / 2.0)],
            [(-9.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi / 3.0],
            [(-9.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi / 4.0],
            [(-9.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, math.pi / 6.0],
            [(-9.0 * self.slot_stride), 0.0, 0.7, 0.0, 0.0, 0.0],
            [(-11.0 * self.slot_stride) + self.camera_center_right_meters , self.camera_center_back_meters, 0.7, 0.0, 0.0, 0.0],
            [(-13.0 * self.slot_stride) + self.camera_center_right_meters , self.camera_center_back_meters, 0.7, 0.0, 0.0, 0.0],
        ]

        with state_machine:
            #########################
            ######### GRAB 1 ########
            #########################
            move_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                    goal_positions[0],
                    frame_id='world',
                    seq=0,
                    stamp=rospy.Time.now()
                ),
                position_source='breadcrumb'
            )

            smach.StateMachine.add(
                'MOVE_{}'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'PAUSE_{}a'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'PAUSE_{}a'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'MOVE_{}p'.format(str(action_number))}
            )

            smach.StateMachine.add(
                'MOVE_{}p'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'PAUSE_{}c'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'PAUSE_{}c'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'CLOSE_GRIPPER_{}'.format(str(action_number))}
            )

            close_gripper_goal = localization_informed_planning_sim.msg.ActuateGripperGoal(position='close')
            smach.StateMachine.add(
                'CLOSE_GRIPPER_{}'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'actuate_gripper',
                    localization_informed_planning_sim.msg.ActuateGripperAction,
                    goal=close_gripper_goal
                ),
                transitions={'succeeded': 'UPDATE_{}up'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'UPDATE_{}up'.format(str(action_number)),
                UpdateMarkerPositionState(
                    marker_id=2,
                    position=[-9.0 * self.slot_stride, 0.0, 0.0],
                    yaw=0.0,
                    masked=True,
                ),
                transitions={'succeeded': 'PAUSE_{}'.format(str(action_number))}
            )

            smach.StateMachine.add(
                'PAUSE_{}'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'MOVE_{}a'.format(str(action_number))}
            )

            smach.StateMachine.add(
                'MOVE_{}a'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'Move_im{}{}'.format(str(action_number), '0')},
            )

        ###########################
        ####### TRANSIT 1 #########
        ###########################

            for i, intermediate in enumerate(intermediates):
                intermediate_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                    target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                        intermediate,
                        frame_id='world',
                        seq=0,
                        stamp=rospy.Time.now()
                    ),
                    position_source='breadcrumb'
                )
                smach.StateMachine.add(
                    'Move_im{}{}'.format(action_number, i),
                    smach_ros.SimpleActionState(
                        'move_to_position_server',
                        localization_informed_planning_sim.msg.MoveToPositionAction, 
                        goal=intermediate_goal
                    ),
                    transitions={'succeeded': 'Pause_im{}{}'.format(action_number, i)}
                )

                next_state = 'MOVE_{}'.format(action_number + 1)
                if i < len(intermediates)-1:
                    next_state = 'Move_im{}{}'.format(action_number, i+1)

                smach.StateMachine.add(
                    'Pause_im{}{}'.format(str(action_number), i),
                    PauseState(5.0),
                    transitions={'succeeded': next_state}
                )

            ##############################
            ############ DROP 1 ##########
            ##############################
            action_number = action_number + 1
            move_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                    goal_positions[1],
                    frame_id='world',
                    seq=0,
                    stamp=rospy.Time.now()
                ),
                position_source='breadcrumb'
            )

            smach.StateMachine.add(
                'MOVE_{}'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'PAUSE_{}a'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'PAUSE_{}a'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'MOVE_{}p'.format(str(action_number))}
            )

            smach.StateMachine.add(
                'MOVE_{}p'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'PAUSE_{}c'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'PAUSE_{}c'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'OPEN_GRIPPER_{}'.format(str(action_number))}
            )

            open_gripper_goal = localization_informed_planning_sim.msg.ActuateGripperGoal(position='open')
            smach.StateMachine.add(
                'OPEN_GRIPPER_{}'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'actuate_gripper',
                    localization_informed_planning_sim.msg.ActuateGripperAction,
                    goal=open_gripper_goal
                ),
                transitions={'succeeded': 'UPDATE_{}up'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'UPDATE_{}up'.format(str(action_number)),
                UpdateMarkerPositionState(
                    marker_id=2,
                    position=[-9.0 * self.slot_stride, 0.0, 0.0],
                    yaw=0.0,
                    masked=False,
                ),
                transitions={'succeeded': 'PAUSE_{}'.format(str(action_number))}
            )

            smach.StateMachine.add(
                'PAUSE_{}'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'Move_im{}{}'.format(str(action_number), '0')}
            )

        #########################
        #### TRANSIT 2 ##########
        #########################

            for i, intermediate in enumerate(intermediates_2):
                intermediate_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                    target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                        intermediate,
                        frame_id='world',
                        seq=0,
                        stamp=rospy.Time.now()
                    ),
                    position_source='breadcrumb'
                )
                smach.StateMachine.add(
                    'Move_im{}{}'.format(action_number, i),
                    smach_ros.SimpleActionState(
                        'move_to_position_server',
                        localization_informed_planning_sim.msg.MoveToPositionAction, 
                        goal=intermediate_goal
                    ),
                    transitions={'succeeded': 'Pause_im{}{}'.format(action_number, i)}
                )

                next_state = 'MOVE_{}'.format(action_number + 1)
                if i < len(intermediates)-1:
                    next_state = 'Move_im{}{}'.format(action_number, i+1)

                smach.StateMachine.add(
                    'Pause_im{}{}'.format(str(action_number), i),
                    PauseState(5.0),
                    transitions={'succeeded': next_state}
                )

            ########################
            #### PICKUP 2 ##########
            ########################
            action_number = action_number + 1
            move_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                    goal_positions[2],
                    frame_id='world',
                    seq=0,
                    stamp=rospy.Time.now()
                ),
                position_source='breadcrumb'
            )

            smach.StateMachine.add(
                'MOVE_{}'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'PAUSE_{}a'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'PAUSE_{}a'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'MOVE_{}p'.format(str(action_number))}
            )

            smach.StateMachine.add(
                'MOVE_{}p'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'PAUSE_{}c'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'PAUSE_{}c'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'CLOSE_GRIPPER_{}'.format(str(action_number))}
            )

            close_gripper_goal = localization_informed_planning_sim.msg.ActuateGripperGoal(position='close')
            smach.StateMachine.add(
                'CLOSE_GRIPPER_{}'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'actuate_gripper',
                    localization_informed_planning_sim.msg.ActuateGripperAction,
                    goal=close_gripper_goal
                ),
                transitions={'succeeded': 'UPDATE_{}up'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'UPDATE_{}up'.format(str(action_number)),
                UpdateMarkerPositionState(
                    marker_id=3,
                    position=[-12.0 * self.slot_stride, 0.0, 0.0, 0.0],
                    yaw=0.0,
                    masked=True,
                ),
                transitions={'succeeded': 'PAUSE_{}'.format(str(action_number))}
            )

            smach.StateMachine.add(
                'PAUSE_{}'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'MOVE_{}a'.format(str(action_number))}
            )

            smach.StateMachine.add(
                'MOVE_{}a'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'Move_im{}{}'.format(str(action_number), '0')},
            )

        ############################
        ####### TRANSIT 3 ##########
        ############################
            for i, intermediate in enumerate(intermediates_3):
                intermediate_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                    target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                        intermediate,
                        frame_id='world',
                        seq=0,
                        stamp=rospy.Time.now()
                    ),
                    position_source='breadcrumb'
                )
                smach.StateMachine.add(
                    'Move_im{}{}'.format(action_number, i),
                    smach_ros.SimpleActionState(
                        'move_to_position_server',
                        localization_informed_planning_sim.msg.MoveToPositionAction, 
                        goal=intermediate_goal
                    ),
                    transitions={'succeeded': 'Pause_im{}{}'.format(action_number, i)}
                )

                next_state = 'MOVE_{}'.format(action_number + 1)
                if i < len(intermediates)-1:
                    next_state = 'Move_im{}{}'.format(action_number, i+1)

                smach.StateMachine.add(
                    'Pause_im{}{}'.format(str(action_number), i),
                    PauseState(5.0),
                    transitions={'succeeded': next_state}
                )

        ####################
        ### DROP 2 #########
        ####################
            action_number = action_number + 1
            move_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                    goal_positions[3],
                    frame_id='world',
                    seq=0,
                    stamp=rospy.Time.now()
                ),
                position_source='breadcrumb'
            )

            smach.StateMachine.add(
                'MOVE_{}'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'PAUSE_{}a'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'PAUSE_{}a'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'MOVE_{}p'.format(str(action_number))}
            )

            smach.StateMachine.add(
                'MOVE_{}p'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'PAUSE_{}c'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'PAUSE_{}c'.format(str(action_number)),
                PauseState(10.0),
                transitions={'succeeded': 'OPEN_GRIPPER_{}'.format(str(action_number))}
            )

            open_gripper_goal = localization_informed_planning_sim.msg.ActuateGripperGoal(position='open')
            smach.StateMachine.add(
                'OPEN_GRIPPER_{}'.format(str(action_number)),
                smach_ros.SimpleActionState(
                    'actuate_gripper',
                    localization_informed_planning_sim.msg.ActuateGripperAction,
                    goal=open_gripper_goal
                ),
                transitions={'succeeded': 'UPDATE_{}up'.format(str(action_number))},
            )

            smach.StateMachine.add(
                'UPDATE_{}up'.format(str(action_number)),
                UpdateMarkerPositionState(
                    marker_id=3,
                    position=[-12.0 * self.slot_stride, 0.0, 0.0, math.pi],
                    yaw=math.pi,
                    masked=False,
                ),
                transitions={'succeeded': 'PAUSE_{}'.format(str(action_number))}
            )

            smach.StateMachine.add(
                'PAUSE_{}'.format(str(action_number)),
                PauseState(10.0),
                #transitions={'succeeded': 'Move_im{}{}'.format(str(action_number), '0')}
            )

        return state_machine