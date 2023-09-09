import math
import rospy
import numpy as np

import smach
import smach_ros
import actionlib

import localization_informed_planning_sim.msg
import droplet_underwater_assembly_libs.utils
from localization_informed_planning_sim_libs.custom_smach_states import PauseState

class RealWorldBuildPlanConstructor(object):
    def __init__(self):
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
            [-0.36, 0.07, 0.58, 0.0, 0.0, 0.0],
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
            )

        return state_machine

    def get_controller_test_state_machine(self):
        state_machine = smach.StateMachine(['succeeded', 'aborted', 'preempted'])

        action_number = 0
        pause_time = 15.0
        goal_positions = [
            [-0.360, 0.07, 0.6, 0.0, 0.0, 0.0],
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