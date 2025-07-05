import math
import rospy
import numpy as np
import copy
import smach
import smach_ros
import actionlib

import localization_informed_planning_sim.msg
import droplet_underwater_assembly_libs.utils
from localization_informed_planning_sim_libs.custom_smach_states import PauseState, UpdateMarkerPositionState, FieldIdleState
from localization_informed_planning_sim_libs.ui_tube_wrapper import UITubeController


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

    def get_field_experiment_state_machine(self):
        state_machine = smach.StateMachine(['succeeded', 'aborted', 'preempted'])
        #pre_grab_drop_pose = [-0.26, 0, 0.85, 0, 0, 0]
        #pre_grab_drop_pose = [-0.26, 0, 0.32, 0, 0, math.pi] #tank
        #pre_grab_drop_pose = [-0.26, 0, 0.77, 0.0, 0.0, math.pi] # too high
        pre_grab_drop_pose = [-0.54, 0.00, 0.67, 0.0, 0.0, math.pi]

        with state_machine:
            move_goal_with_clear = localization_informed_planning_sim.msg.MoveToPositionGoal(
                target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                    pre_grab_drop_pose,
                    frame_id='world',
                    seq=0,
                    stamp=rospy.Time.now()
                ),
                position_source='breadcrumb',
                clear_error_integrals=True
            )

            move_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                    pre_grab_drop_pose,
                    frame_id='world',
                    seq=0,
                    stamp=rospy.Time.now()
                ),
                position_source='breadcrumb',
                clear_error_integrals=False
            )

            smach.StateMachine.add(
                'idle',
                FieldIdleState(target_position=pre_grab_drop_pose),
                transitions={'succeeded': 'move1'}
            )

            smach.StateMachine.add(
                'move1',
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal_with_clear
                ),
                transitions={'succeeded': 'pause1', 'aborted': 'idle'},
            )

            smach.StateMachine.add(
                'pause1',
                PauseState(15.0),
                transitions={'succeeded': 'move1a', 'aborted': 'idle'}
            )

            smach.StateMachine.add(
                'move1a',
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'grab', 'aborted': 'idle'}
            )

            close_gripper_goal = localization_informed_planning_sim.msg.ActuateGripperGoal(position='close', plunge=True)

            smach.StateMachine.add(
                'grab',
                smach_ros.SimpleActionState(
                    'actuate_gripper',
                    localization_informed_planning_sim.msg.ActuateGripperAction,
                    goal=close_gripper_goal
                ),
                transitions={'succeeded': 'pause2', 'aborted': 'idle'},
            )

            smach.StateMachine.add(
                'pause2',
                PauseState(15.0),
                transitions={'succeeded': 'move2', 'aborted': 'idle'}
            )

            smach.StateMachine.add(
                'move2',
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'pause3', 'aborted': 'idle'},
            )

            smach.StateMachine.add(
                'pause3',
                PauseState(15.0),
                transitions={'succeeded': 'move2a', 'aborted': 'idle'}
            )

            smach.StateMachine.add(
                'move2a',
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal
                ),
                transitions={'succeeded': 'release', 'aborted': 'idle'},
            )

            open_gripper_goal = localization_informed_planning_sim.msg.ActuateGripperGoal(position='open', plunge=True)
            smach.StateMachine.add(
                'release',
                smach_ros.SimpleActionState(
                    'actuate_gripper',
                    localization_informed_planning_sim.msg.ActuateGripperAction,
                    goal=open_gripper_goal
                ),
                transitions={'succeeded': 'idle', 'aborted': 'idle'},
            )

        return state_machine
    

    def add_move(self, position, pause_after_time, move_name, pause_name, next_action_name):
        move_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
            target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                position,
                frame_id='world',
                seq=0,
                stamp=rospy.Time.now()
            ),
            position_source='breadcrumb',
            clear_error_integrals=False
        )

        
        smach.StateMachine.add(
            move_name,
            smach_ros.SimpleActionState(
                'move_to_position_server',
                localization_informed_planning_sim.msg.MoveToPositionAction, 
                goal=move_goal,
            ),
            transitions={'succeeded': pause_name, 'aborted': 'idle'}
        )

        smach.StateMachine.add(
            pause_name,
            PauseState(pause_after_time),
            transitions={'succeeded': next_action_name}
        )


    def add_gripper_actuate(self, position, plunge, action_name, next_action_name):
        goal = localization_informed_planning_sim.msg.ActuateGripperGoal(position=position, plunge=plunge)
        smach.StateMachine.add(
            action_name,
            smach_ros.SimpleActionState(
                'actuate_gripper',
                localization_informed_planning_sim.msg.ActuateGripperAction,
                goal=goal
            ),
            transitions={'succeeded': next_action_name, 'aborted': 'idle'},
        )


    def get_puzzleflex_simple_experiment_state_machine(self):
        #q: back to m3 center: 9.5 inches (0.2413)
        #m: m3 to m2 center: 14in (0.3556)
        #l: b to s1 9cm: 0.09
        #j: s1 to s2 30.7 cm: 0.307
        #s: 1.219

        # pic1: cam_to_manip_x
        state_machine = smach.StateMachine(['succeeded', 'aborted', 'preempted'])
        cam_to_manip_y = -0.48
        cam_to_manip_x = 0.00
        #drop1 = [0.96 + cam_to_manip_x, -0.03 + cam_to_manip_y, 0.75, 0.0, 0.0, -math.pi/2.0]

        pickup_position = [-0.4656, -0.07, 0.8, 0.0, 0.0, 0.0]
        pickup_down_pos = copy.deepcopy(pickup_position)
        pickup_down_pos[2] = pickup_down_pos[2] - 0.1
        pickup1_lift = copy.deepcopy(pickup_position)
        pickup1_lift[2] = pickup1_lift[2] + 0.38

        top_predrop     = [-0.3556, 0.23, 1.07, 0.0, 0.0, 0.0]
        top_predrop_down = copy.deepcopy(top_predrop)
        top_predrop_down[2] = top_predrop_down[2] - 0.15

        pickupa_position = [-0.0656, -0.07, 0.8, 0.0, 0.0, 0.0]
        pickupa_down_pos = copy.deepcopy(pickupa_position)
        pickupa_down_pos[2] = pickupa_down_pos[2] - 0.1
        pickupa1_lift = copy.deepcopy(pickupa_position)
        pickupa1_lift[2] = pickupa1_lift[2] + 0.38

        topa_predrop     = [-0.1556, 0.23, 1.07, 0.0, 0.0, 0.0]
        topa_predrop_down = copy.deepcopy(topa_predrop)
        topa_predrop_down[2] = topa_predrop_down[2] - 0.15

        # +x is right facing tag
        # +y is up facing tag
        with state_machine:
            smach.StateMachine.add(
                'idle',
                FieldIdleState(target_position=pickup_position),
                transitions={'succeeded': 'm1'}
            )

            self.add_move(position=pickup_position, pause_after_time=10.0, move_name="m1", pause_name="p1", next_action_name='m2')
            self.add_move(position=pickup_down_pos, pause_after_time=10.0, move_name="m2", pause_name="p2", next_action_name='grab1')
            self.add_gripper_actuate(position='close', plunge=True, action_name="grab1", next_action_name="m3")
            self.add_move(position=pickup1_lift, pause_after_time=10.0, move_name="m3", pause_name="p3", next_action_name='m4')
            self.add_move(position=pickup1_lift, pause_after_time=15.0, move_name="m4", pause_name="p4", next_action_name='m5')
            self.add_move(position=top_predrop, pause_after_time=10.0, move_name="m5", pause_name="p5", next_action_name='m6')
            self.add_move(position=top_predrop, pause_after_time=15.0, move_name="m6", pause_name="p6", next_action_name='m7')
            self.add_move(position=top_predrop_down, pause_after_time=25.0, move_name="m7", pause_name="p7", next_action_name='m7a')
            self.add_move(position=top_predrop_down, pause_after_time=0.0, move_name="m7a", pause_name="p7a", next_action_name='open1')
            self.add_gripper_actuate(position='open', plunge=True, action_name="open1", next_action_name="m8")
            self.add_move(position=top_predrop, pause_after_time=25.0, move_name="m8", pause_name="p8", next_action_name='idle')

            #self.add_move(position=pickupa_position, pause_after_time=10.0, move_name="m1", pause_name="p1", next_action_name='m2')
            #self.add_move(position=pickupa_down_pos, pause_after_time=10.0, move_name="m2", pause_name="p2", next_action_name='grab1')
            #self.add_gripper_actuate(position='close', plunge=True, action_name="grab1", next_action_name="m3")
            #self.add_move(position=pickupa1_lift, pause_after_time=10.0, move_name="m3", pause_name="p3", next_action_name='m4')
            #self.add_move(position=pickupa1_lift, pause_after_time=15.0, move_name="m4", pause_name="p4", next_action_name='m5')
            #self.add_move(position=topa_predrop, pause_after_time=10.0, move_name="m5", pause_name="p5", next_action_name='m6')
            #self.add_move(position=topa_predrop, pause_after_time=15.0, move_name="m6", pause_name="p6", next_action_name='m7')
            #self.add_move(position=topa_predrop_down, pause_after_time=25.0, move_name="m7", pause_name="p7", next_action_name='m7a')
            #self.add_move(position=topa_predrop_down, pause_after_time=0.0, move_name="m7a", pause_name="p7a", next_action_name='open1')
            #self.add_gripper_actuate(position='open', plunge=True, action_name="open1", next_action_name="m8")
            #self.add_move(position=topa_predrop, pause_after_time=25.0, move_name="m8", pause_name="p8", next_action_name='idle')


            """
            move_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                    pickup_position,
                    frame_id='world',
                    seq=0,
                    stamp=rospy.Time.now()
                ),
                position_source='breadcrumb'
            )


            smach.StateMachine.add(
                'MOVE_1',
                smach_ros.SimpleActionState(
                    'move_to_position_server',
                    localization_informed_planning_sim.msg.MoveToPositionAction, 
                    goal=move_goal,
                ),
                #transitions={'succeeded': 'pause1a'}
            )
            """

        return state_machine



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
            [-0.26, 0.0, 0.38, 0.0, 0.0, 0.0],
            #[0.0, 0.0, 0.8, 0.0, 0.0, math.pi / 4.0],
        ]
        smach.StateMachine.add(
            'idle',
            FieldIdleState(),
            transitions={'succeeded': 'MOVE_0'}
        )
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
