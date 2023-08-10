#!/usr/bin/python
import collections
import numpy as np
import json
import copy
import pprint
import time
import math

import roslib
roslib.load_manifest('localization_informed_planning_sim')
import rospy
import smach
import smach_ros
import actionlib
import gazebo_msgs.srv
from geometry_msgs.msg import Pose, Quaternion, Point
import geometry_msgs.msg
import rospkg
import nav_msgs.msg

import droplet_underwater_assembly_libs.utils
import localization_informed_planning_sim.srv
import localization_informed_planning_sim.msg

Block = collections.namedtuple('Block', ['id', 'position', 'index'])
Marker = collections.namedtuple('Marker', ['id', 'position', 'index'])

class DespawnBlockState(smach.State):
    def __init__(self, target_block_id, assembly_manager, outcomes=['succeeded', 'aborted']):
        smach.State.__init__(self, outcomes=outcomes)
        self.target_block_id = target_block_id
        self.assembly_manager = assembly_manager

    def execute(self, userdata):
        self.assembly_manager.despawn_model(self.target_block_id)
        return 'succeeded'


class SpawnMarkerState(smach.State):
    def __init__(self, marker_id, assembly_manager, position, slot_index, outcomes=['succeeded', 'aborted']):
        smach.State.__init__(self, outcomes=outcomes)
        self.assembly_manager = assembly_manager
        self.position = position
        self.marker_id = marker_id
        self.slot_index = slot_index

    def execute(self, userdata):
        self.assembly_manager.spawn_markers(
            marker_positions=[self.position],
            marker_ids=[self.marker_id],
            marker_indices=[self.slot_index]
        )
        return 'succeeded'


class PauseState(smach.State):
    def __init__(self, pause_seconds, outcomes=['succeeded', 'aborted']):
        smach.State.__init__(self, outcomes=outcomes)
        self.pause_seconds = pause_seconds

    def execute(self, userdata):
        rospy.sleep(self.pause_seconds)
        return 'succeeded'


class AssemblyProcessManager():
    ocean_depth_meters = 100.0 # lowest block at -99.6
    vehicle_height = 1.0
    block_scaling_factor = 3.0
    block_height_meters = 0.4 * block_scaling_factor
    manipulator_height = 0.2 * block_scaling_factor

    lowest_block_height = -ocean_depth_meters + block_height_meters / 2.0

    def __init__(self):
        rospy.init_node('assembly_process_manager')
        self.spawn_model_service_name = '/gazebo/spawn_sdf_model'
        self.delete_model_service_name = '/gazebo/delete_model'
        ground_truth_position_topic = '/rexrov/pose_gt' # ground truth pose

        self.structure_state = {}
        self.move_action_client = actionlib.SimpleActionClient(
            'move_to_position_server',
            localization_informed_planning_sim.msg.MoveToPositionAction
        )

        self.spawn_model_client = rospy.ServiceProxy(self.spawn_model_service_name, gazebo_msgs.srv.SpawnModel)
        self.delete_model_client = rospy.ServiceProxy(self.delete_model_service_name, gazebo_msgs.srv.DeleteModel)

        #self.marker_model_path_format = rospkg.RosPack().get_path('localization_informed_planning_sim') + '/models/aruco_{marker_id}/model.sdf'
        #self.block_model_path = rospkg.RosPack().get_path('localization_informed_planning_sim') + '/models/standard_block/model.sdf'

        self.set_marker_position_client = rospy.ServiceProxy(
            'set_marker_position',
            localization_informed_planning_sim.srv.SetGlobalPosition
        )

        self.set_controller_target_client = rospy.ServiceProxy(
            '/set_goal_position',
            localization_informed_planning_sim.srv.SetControllerTarget
        )

        self.actions = []

        self.marker_reading_history = []

    def on_grasp_success(self, target_pose, block_name):
        rospy.loginfo("Successfully grasped block {} at position {}".format(block_name, target_pose))
        self.despawn_model(block_name)

    def on_grasp_fail(self, target_pose, block_name):
        rospy.loginfo("Failed to grasp block {}".format(block_name))
        raise Exception("Grasp failed! Construction process failed")

    def get_grasp_position_for_block_name(self, block_name):
        return self.offset_position_for_grasp(self.block_position_by_id[block_name])

    def offset_position_for_grasp(self, position):
        return [
            position[0],
            position[1],
            position[2] + AssemblyProcessManager.block_height_meters + (AssemblyProcessManager.vehicle_height / 2.0) + AssemblyProcessManager.manipulator_height
        ]

    def send_marker_position_to_breadcrumb(self, marker_id, position):
        self.set_marker_position_client(
            marker_id=marker_id,
            x=position[1],
            y=position[0], # intentionally done bc of frame mismatch
            z=position[2]
        )

    def get_model_path_for_marker_id(self, marker_id):
        return self.marker_model_path_format.format(marker_id=marker_id)
    
    def despawn_model(self, model_name):
        self.delete_model_client(model_name=model_name)

    def spawn_blocks(self, block_positions, block_names, block_indices):
        rospy.wait_for_service(self.spawn_model_service_name)
        for i, block_position in enumerate(block_positions):
            self.spawn_model_client(
                    model_name=block_names[i],
                    model_xml=open(self.block_model_path, 'r').read(),
                    robot_namespace='/foo',
                    initial_pose=Pose(
                        position=Point(*block_position),
                        orientation=Quaternion(0,0,0,1)
                    ),
                    reference_frame='world'
            )

            self.structure_state[block_names[i]] = Block(
                id=block_names[i],
                position=block_position,
                index=block_indices[i]
            )

    def spawn_markers(self, marker_positions, marker_ids, marker_indices):
        self.marker_state = {}
        for i, marker_position in enumerate(marker_positions):
            marker_name = 'marker_{}'.format(marker_ids[i])
            self.spawn_model_client(
                    model_name=marker_name,
                    model_xml=open(self.get_model_path_for_marker_id(marker_ids[i]), 'r').read(),
                    robot_namespace='/foo',
                    initial_pose=Pose(
                        position=Point(*marker_position),
                        orientation=Quaternion(0,0,0,1)
                    ),
                    reference_frame='world'
            )

            self.send_marker_position_to_breadcrumb(int(marker_ids[i]), marker_position)

            self.marker_state[marker_name] = Marker(
                id=marker_ids[i],
                position=marker_position,
                index=marker_indices[i]
            )

    def run_build_plan(self, actions):
        current_move = None
        while not rospy.is_shutdown() and len(actions) > 0:
            current_move = actions[0]

            if not current_move.is_started:
                print("Starting a move!!!")
                current_move.start()
            else:
                pass

            if current_move.is_completed:
                actions.pop(0)

            rospy.sleep(0.1)

    def move_to_idle_position(self):
        rospy.wait_for_service('/set_goal_position')
        self.set_controller_target_client(
            target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                [0.0, 0.0, -97.0, 0.0, 0.0, 0.0],
                frame_id='world',
                seq=0,
                stamp=rospy.Time.now()
            )
        )

    def get_index_to_name_map(self, state_dict):
        index_to_name_map = collections.defaultdict(
            lambda: collections.defaultdict(
                lambda: collections.defaultdict(
                    lambda: None
                )
            )
        )

        for key, value in state_dict.items():
            index_to_name_map[value.index[0]][value.index[1]][value.index[2]] = key

        return index_to_name_map

    def parse_build_plan(self, build_plan_json_path):
        state_machine = smach.StateMachine(['succeeded', 'aborted', 'preempted'])

        with open(build_plan_json_path) as f:
            build_plan = json.load(f)

            block_index_to_name_map = self.get_index_to_name_map(self.structure_state)
            marker_index_to_name_map = self.get_index_to_name_map(self.marker_state)

            with state_machine:
                for action_number, action in enumerate(build_plan["unload_plan"]):
                    next_move = 'MOVE_{}'.format(str(action_number + 1))
                    if action_number == len(build_plan["unload_plan"]) - 1:
                        next_move = None
                    
                    print("Action is", action)
                    from_index = [
                        action["from"]["x"],
                        action["from"]["y"],
                        action["from"]["z"]
                    ]

                    from_angle = 0.0
                    if "from_angle" in action:
                        from_angle = action["from_angle"]

                    to_angle = 0.0
                    if "to_angle" in action:
                        to_angle = action["to_angle"] 

                    grasp_position = self.offset_position_for_grasp(
                        self.get_position_from_slot_index(from_index)
                    )

                    target_block_name = block_index_to_name_map[from_index[0]][from_index[1]][from_index[2]]
                    target_marker_name = marker_index_to_name_map[from_index[0]][from_index[1]][from_index[2]]

                    target_name = target_block_name
                    animate_placement = False

                    if target_name is None:
                        animate_placement = True
                        target_name = target_marker_name

                    goal_pose = [
                        grasp_position[0],
                        grasp_position[1],
                        grasp_position[2],
                        0.0,
                        0.0,
                        from_angle
                    ]

                    move_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                        target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                            goal_pose,
                            frame_id='world',
                            seq=0,
                            stamp=rospy.Time.now()
                        ),
                        position_source='ground_truth'
                    )

                    smach.StateMachine.add(
                        'MOVE_{}'.format(str(action_number)),
                        smach_ros.SimpleActionState(
                            'move_to_position_server',
                            localization_informed_planning_sim.msg.MoveToPositionAction, 
                            goal=move_goal
                        ),
                        transitions={'succeeded': 'GRASP_{}'.format(str(action_number))},
                    )

                    grasp_goal = localization_informed_planning_sim.msg.AttemptGraspGoal(
                        x=goal_pose[0],
                        y=goal_pose[1],
                        z=goal_pose[2],
                    )

                    smach.StateMachine.add(
                        'GRASP_{}'.format(str(action_number)),
                        smach_ros.SimpleActionState(
                            'attempt_grasp_server',
                            localization_informed_planning_sim.msg.AttemptGraspAction,
                            goal=grasp_goal
                        ),
                        transitions={'succeeded': 'DESPAWN_{}'.format(str(action_number))},
                    )

                    next_action = next_move
                    if animate_placement:
                        next_action = 'MOVE_{}.0'.format(str(action_number))

                    smach.StateMachine.add(
                        'DESPAWN_{}'.format(str(action_number)),
                        DespawnBlockState(
                            target_block_id=target_name,
                            assembly_manager=self
                        ),
                        transitions={'succeeded': next_action},
                    )

                    if animate_placement:
                        to_index = [
                            action["to"]["x"],
                            action["to"]["y"],
                            action["to"]["z"],
                        ]
                        marker_index_to_name_map[from_index[0]][from_index[1]][from_index[2]] = None
                        marker_index_to_name_map[to_index[0]][to_index[1]][to_index[2]] = target_name

                        place_position = self.offset_position_for_grasp(
                            self.get_position_from_slot_index(to_index)
                        )

                        place_position_xyzrpy = [place_position[0], place_position[1], place_position[2], 0.0, 0.0, to_angle]

                        place_goal = localization_informed_planning_sim.msg.AttemptGraspGoal(
                            x=place_position[0],
                            y=place_position[1],
                            z=place_position[2],
                        )

                        move_to_place_goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                            target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                                place_position_xyzrpy,
                                frame_id='world',
                                seq=0,
                                stamp=rospy.Time.now()
                            ),
                            position_source='ground_truth'
                        )

                        smach.StateMachine.add(
                            'MOVE_{}.0'.format(str(action_number)),
                            smach_ros.SimpleActionState(
                                'move_to_position_server',
                                localization_informed_planning_sim.msg.MoveToPositionAction, 
                                goal=move_to_place_goal
                            ),
                            transitions={'succeeded': 'GRASP_{}.0'.format(str(action_number))},
                        )

                        smach.StateMachine.add(
                            'GRASP_{}.0'.format(str(action_number)),
                            smach_ros.SimpleActionState(
                                'attempt_grasp_server',
                                localization_informed_planning_sim.msg.AttemptGraspAction,
                                goal=place_goal
                            ),
                            transitions={'succeeded': 'SPAWN_{}'.format(str(action_number))},
                        )

                        smach.StateMachine.add(
                            'SPAWN_{}'.format(action_number),
                            SpawnMarkerState(
                                self.marker_state[target_name].id, 
                                self, 
                                self.get_position_from_slot_index(to_index),
                                to_index
                            ),
                            transitions={'succeeded': next_move}
                        )

        return state_machine 

    def get_position_from_slot_index(self, index):
        sf = AssemblyProcessManager.block_scaling_factor
        cell_scaling = self.cell_side_length * np.array([
            sf, 
            sf,
            sf 
        ])

        cell_offset = np.array([
            (self.cell_side_length * sf) / 2.0,
            (self.cell_side_length * sf) / 2.0, 
            (self.cell_side_length * sf) / 2.0
        ])
        cell_offset = cell_offset - np.array([0.0, 0.0, AssemblyProcessManager.ocean_depth_meters])

        return (cell_scaling * index) + cell_offset


    def get_controller_test_state_machine(self):
        state_machine = smach.StateMachine(['succeeded', 'aborted', 'preempted'])

        action_number = 0
        pause_time = 5.0
        goal_positions = [
            [0.0, 0.0, 0.8, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.8, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.8, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.8, 0.0, 0.0, math.pi/3.0],
            [0.5, 0.0, 0.8, 0.0, 0.0, math.pi/3.0],
            [-0.5, 0.0, 0.75, 0.0, 0.0, math.pi/3.0],
            [0.0, 0.0, 0.80, 0.0, 0.0, math.pi/2.0],
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

    def spawn_structure(self, structure_spec_json_path):
        with open(structure_spec_json_path, 'r') as f:
            structure_spec = json.load(f)
            self.cell_side_length = structure_spec['cell_side_length']

            normal_block_indices = [
                np.array([b['x'], b['y'], b['z']]) for b in structure_spec['structure'] if b['block_type'] == 'normal'
            ]

            marker_indices = [
                np.array([b['x'], b['y'], b['z']]) for b in structure_spec['structure'] if b['block_type'] == 'localization'
            ]

            self.block_position_by_id = {
                "block_{i}".format(i=i): self.get_position_from_slot_index(b) for i, b in enumerate(normal_block_indices)
            }

            if len(marker_indices) > 4:
                rospy.logerror("More than 4 markers are not supported!")
                raise Exception("More than 4 markers are not supported!")

            self.marker_position_by_id = {
                i: self.get_position_from_slot_index(b) for i, b in enumerate(marker_indices) 
            }

            marker_positions_and_names = [i for i in self.marker_position_by_id.items()]
            normal_positions_and_names = [i for i in self.block_position_by_id.items()]
            self.spawn_markers(
                marker_positions=[i[1] for i in marker_positions_and_names],
                marker_ids=[i[0] for i in marker_positions_and_names],
                marker_indices=[i for i in marker_indices]
            )

            self.spawn_blocks(
                block_positions=[i[1] for i in normal_positions_and_names],
                block_names=[i[0] for i in normal_positions_and_names],
                block_indices=[i for i in normal_block_indices]
            )


if __name__ == '__main__':
    manager = AssemblyProcessManager()
    #manager.move_to_idle_position()
    #manager.spawn_structure(
    #    "/home/sam/uuv_ws/src/localization_informed_planning_sim/param/pallet4.json"
    #)

    #rospy.sleep(4.0)
    #actions = manager.parse_build_plan(
    #    "/home/sam/uuv_ws/src/localization_informed_planning_sim/param/plan_pallet4.json"
    #)
    #move_client.cancel()

    #rospy.loginfo("Starting build plan execution.")
    #manager.run_build_plan(actions)
    #state_machine = manager.parse_build_plan(
    #    "/home/sam/uuv_ws/src/localization_informed_planning_sim/param/plan_hopping_four.json",
    #    #"/home/sam/uuv_ws/src/localization_informed_planning_sim/param/plan_hopping.json"
    #)
    state_machine = manager.get_controller_test_state_machine()

    introspection_server = smach_ros.IntrospectionServer('assembly_state_machine', state_machine, '/SM_ROOT')
    introspection_server.start()

    state_machine.execute()
