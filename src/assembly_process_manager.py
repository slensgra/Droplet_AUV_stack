#!/usr/bin/python3
import collections
import numpy as np
import json
import copy
import pprint

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

class MoveBehaviorClient():
    source_ground_truth = 'ground_truth'
    source_breadcrumb = 'breadcrumb'

    def __init__(self, goal_position, position_source):
        self.goal_position = goal_position
        self.position_source = position_source

        self.move_action_client = actionlib.SimpleActionClient(
            'move_to_position_server',
            localization_informed_planning_sim.msg.MoveToPositionAction,
        )

        self.is_started = False

    def cancel(self):
        self.move_action_client.cancel_goal()

    @property
    def is_completed(self):
        state = self.move_action_client.get_state()
        return state == actionlib.GoalStatus.SUCCEEDED

    def start(self):
        self.is_started = True
        print("Waiting for server...")
        self.move_action_client.wait_for_server()
        print("Found server!")

        rospy.loginfo("Sending goal position, {}".format(self.goal_position))
        self.move_action_client.send_goal(
            localization_informed_planning_sim.msg.MoveToPositionGoal(
                target_position=self.goal_position,
                position_source=self.position_source
            )
        )


class GraspBehaviorClient():
    def __init__(self, ideal_grasp_position, on_success, on_fail):
        self.grasp_action_client = actionlib.SimpleActionClient(
            'attempt_grasp_server',
            localization_informed_planning_sim.msg.AttemptGraspAction
        )
        self.ideal_grasp_position = ideal_grasp_position
        self.on_success = on_success
        self.on_fail = on_fail
        self.is_started = False
        self.terminated = False

    def start(self):
        self.is_started=True
        self.grasp_action_client.wait_for_server()

        self.grasp_action_client.send_goal(
            localization_informed_planning_sim.msg.AttemptGraspGoal(
                x=self.ideal_grasp_position[0],
                y=self.ideal_grasp_position[1],
                z=self.ideal_grasp_position[2],
            )
        )

    @property
    def is_completed(self):
        state = self.grasp_action_client.get_state()

        if state == actionlib.GoalStatus.SUCCEEDED:
            if not self.terminated:
                self.terminated = True
                self.on_success()
            return True

        if state == actionlib.GoalStatus.ABORTED:
            if not self.terminated:
                self.terminated = True
                self.on_fail()
            return False

        return False


class InfiniteLoopBehavior():
    def __init__(self):
        self.is_started = False
        self.is_completed = False

    def start(self):
        self.is_started = True


class DespawnBlockState(smach.State):
    def __init__(self, target_block_id, assembly_manager, outcomes=['succeeded', 'aborted']):
        smach.State.__init__(self, outcomes=outcomes)
        self.target_block_id = target_block_id
        self.assembly_manager = assembly_manager

    def execute(self, userdata):
        self.assembly_manager.despawn_model(self.target_block_id)
        return 'succeeded'


class AssemblyProcessManager():
    ocean_depth_meters = 100.0 # lowest block at -99.6
    block_height_meters = 0.4
    lowest_block_height = -99.8
    vehicle_height = 1.0
    selected_pallet_to_unload = './pallets/pallet_0.json'

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

        rospy.wait_for_service(self.spawn_model_service_name)

        self.spawn_model_client = rospy.ServiceProxy(self.spawn_model_service_name, gazebo_msgs.srv.SpawnModel)
        self.delete_model_client = rospy.ServiceProxy(self.delete_model_service_name, gazebo_msgs.srv.DeleteModel)

        self.marker_model_path_format = rospkg.RosPack().get_path('localization_informed_planning_sim') + '/models/aruco_{marker_id}/model.sdf'
        self.block_model_path = rospkg.RosPack().get_path('localization_informed_planning_sim') + '/models/standard_block/model.sdf'
        self.set_marker_position_client = rospy.ServiceProxy(
            'set_marker_position',
            localization_informed_planning_sim.srv.SetGlobalPosition
        )

        self.set_controller_target_client = rospy.ServiceProxy(
            'set_goal_position',
            localization_informed_planning_sim.srv.SetControllerTarget
        )

        self.actions = []

    def get_simple_plan(self):
        frame_id = 'world'
        seq = 0
        stamp = rospy.Time.now()

        largest_block_name = max((k for k in self.block_position_by_id.keys()), key=lambda x: self.block_position_by_id[x][2])
        target_pos = self.get_grasp_position_for_block_name(largest_block_name)

        rospy.loginfo("~~~~~~~~~~~~~~~~~~~~~~~~")
        rospy.loginfo("Grasp position for block, {}: {}".format(largest_block_name, target_pos))
        rospy.loginfo("~~~~~~~~~~~~~~~~~~~~~~~~")

        target_pos_xyzrpy =  [target_pos[0], target_pos[1], target_pos[2], 0.0, 0.0, 0.0]

        on_suc = lambda: self.on_grasp_success(target_pos_xyzrpy, largest_block_name)
        on_f   = lambda: self.on_grasp_fail(target_pos_xyzrpy, largest_block_name)

        actions = [
            MoveBehaviorClient(
                droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                    target_pos_xyzrpy,
                    frame_id=frame_id,
                    seq=seq,
                    stamp=stamp
                ), 
                MoveBehaviorClient.source_ground_truth
            ),
            GraspBehaviorClient(
                ideal_grasp_position=target_pos_xyzrpy,
                on_success=on_suc,
                on_fail=on_f
            ),
            MoveBehaviorClient(
                droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                    target_pos_xyzrpy,
                    frame_id=frame_id,
                    seq=seq,
                    stamp=stamp
                ), 
                MoveBehaviorClient.source_ground_truth
            ),
            InfiniteLoopBehavior(),
        ]

        return actions

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
            position[2] + AssemblyProcessManager.block_height_meters + AssemblyProcessManager.vehicle_height / 2.0 + 0.1
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
            marker_name = 'marker_{}'.format(i)
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
        rospy.wait_for_service('/controller/set_goal_position')
        self.set_controller_target_client(
            target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                [0.0, 0.0, -98.0, 0.0, 0.0, 0.0],
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
        # this is a set of pairs of indices. Move between the indices
        # and grasp. Disregard the placement if the grabbed block is a normal one.

        # maybe for now just normal grabs even.

        result_actions = []

        with open(build_plan_json_path) as f:
            build_plan = json.load(f)

            block_index_to_name_map = self.get_index_to_name_map(self.structure_state)
            marker_index_to_name_map = self.get_index_to_name_map(self.marker_state)

            for action in build_plan["unload_plan"]:
                print("Action is", action)
                from_index = [
                    action["from"]["x"],
                    action["from"]["y"],
                    action["from"]["z"]
                ]

                grasp_position = self.offset_position_for_grasp(
                    self.get_position_from_slot_index(from_index)
                )

                target_block_name = block_index_to_name_map[from_index[0]][from_index[1]][from_index[2]]
                target_marker_name = marker_index_to_name_map[from_index[0]][from_index[1]][from_index[2]]

                target_name = target_block_name
                if target_name is None:
                    target_name = target_marker_name

                if target_name is None:
                    print("Name maps:")
                    print("Block names:")
                    pprint.pprint(block_index_to_name_map)

                    print("Marker names:")
                    pprint.pprint(marker_index_to_name_map)
                    rospy.logwarn("Could not find a name for index {}".format(from_index))
                    continue
                    #raise Exception("Could not find a name for the index {}".format(from_index))

                goal_pose = [
                    grasp_position[0],
                    grasp_position[1],
                    grasp_position[2],
                    0.0,
                    0.0,
                    0.0
                ]

                result_actions.append(
                    MoveBehaviorClient(
                        droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                            goal_pose,
                            frame_id='world',
                            seq=0,
                            stamp=rospy.Time.now()
                        ),
                        MoveBehaviorClient.source_ground_truth
                    )
                )

                result_actions.append(
                    GraspBehaviorClient(
                        ideal_grasp_position=goal_pose,
                        on_success=lambda: self.on_grasp_success(goal_pose, copy.deepcopy(target_block_name)),
                        on_fail=lambda: self.on_grasp_fail(goal_pose, copy.deepcopy(target_block_name)),
                    )
                )

        return result_actions
    
    def test_state_machine_build_plan(self):
        state_machine = smach.StateMachine(['succeeded', 'aborted', 'preempted'])

        with state_machine:
            for i in range(3):
                goal_pose = [0.0, 0.0, -96.0 + float(i), 0.0, 0.0, 0.0]
                goal = localization_informed_planning_sim.msg.MoveToPositionGoal(
                    target_position=droplet_underwater_assembly_libs.utils.pose_stamped_from_xyzrpy(
                        goal_pose,
                        frame_id='world',
                        seq=0,
                        stamp=rospy.Time.now()
                    ),
                    position_source='ground_truth'
                )

                next_move = 'MOVE_{}'.format(str(i+1))
                if i == 2:
                    next_move = None

                smach.StateMachine.add(
                    'MOVE_{}'.format(str(i)),
                    smach_ros.SimpleActionState(
                        'move_to_position_server',
                        localization_informed_planning_sim.msg.MoveToPositionAction, 
                        goal=goal
                    ),
                    transitions={'succeeded': 'GRASP_{}'.format(str(i))},
                )

                grasp_goal = localization_informed_planning_sim.msg.AttemptGraspGoal(
                    x=goal_pose[0],
                    y=goal_pose[1],
                    z=goal_pose[2],
                )

                smach.StateMachine.add(
                    'GRASP_{}'.format(str(i)),
                    smach_ros.SimpleActionState(
                        'attempt_grasp_server',
                        localization_informed_planning_sim.msg.AttemptGraspAction,
                        goal=grasp_goal
                    ),
                    transitions={'succeeded': 'DESPAWN_{}'.format(str(i))},
                )

                smach.StateMachine.add(
                    'DESPAWN_{}'.format(str(i)),
                    DespawnBlockState(
                        target_block_id='block_{}'.format(str(i)),
                        assembly_manager=self
                    ),
                    transitions={'succeeded': next_move},
                )

        return state_machine
        

    def get_position_from_slot_index(self, index):
        cell_scaling = self.cell_side_length * np.array([1.0, 1.0, 1.0])
        cell_offset = np.array([self.cell_side_length / 2.0, self.cell_side_length / 2.0, self.cell_side_length / 2.0])
        cell_offset = cell_offset - np.array([0.0, 0.0, AssemblyProcessManager.ocean_depth_meters])

        return (cell_scaling * index) + cell_offset


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

            if len(marker_indices) > 2:
                rospy.logerror("More than 2 markers are not supported!")
                raise Exception("More than 2 markers are not supported!")

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
    manager.move_to_idle_position()
    manager.spawn_structure(
        "/home/sam/uuv_ws/src/localization_informed_planning_sim/param/pallet90.json"
    )

    rospy.sleep(4.0)
    #actions = manager.parse_build_plan(
    #    "/home/sam/uuv_ws/src/localization_informed_planning_sim/param/plan_pallet4.json"
    #)
    #move_client.cancel()

    #rospy.loginfo("Starting build plan execution.")
    #manager.run_build_plan(actions)

    state_machine = manager.test_state_machine_build_plan()
    state_machine.execute()