import json
import math

import roslib
roslib.load_manifest('localization_informed_planning_sim')
import rospy
import smach
import smach_ros

import droplet_underwater_assembly_libs.utils
import localization_informed_planning_sim.msg
import localization_informed_planning_sim.srv


class PauseState(smach.State):
    def __init__(self, pause_seconds, outcomes=['succeeded', 'aborted']):
        smach.State.__init__(self, outcomes=outcomes)
        self.pause_seconds = pause_seconds

    def execute(self, userdata):
        rospy.sleep(self.pause_seconds)
        return 'succeeded'

class SimulationBuildPlanConstructor():
    ocean_depth_meters = 100.0 # lowest block at -99.6
    vehicle_height = 1.0
    block_scaling_factor = 3.0
    block_height_meters = 0.4 * block_scaling_factor
    manipulator_height = 0.2 * block_scaling_factor

    lowest_block_height = -ocean_depth_meters + block_height_meters / 2.0

    def on_grasp_success(self, target_pose, block_name):
        rospy.loginfo("Successfully grasped block {} at position {}".format(block_name, target_pose))
        self.despawn_model(block_name)

    def get_grasp_position_for_block_name(self, block_name):
        return self.offset_position_for_grasp(self.block_position_by_id[block_name])

    def offset_position_for_grasp(self, position):
        return [
            position[0],
            position[1],
            position[2] + SimulationBuildPlanConstructor.block_height_meters + (SimulationBuildPlanConstructor.vehicle_height / 2.0) + SimulationBuildPlanConstructor.manipulator_height
        ]

    def on_grasp_fail(self, target_pose, block_name):
        rospy.loginfo("Failed to grasp block {}".format(block_name))
        raise Exception("Grasp failed! Construction process failed")

    def parse_json_build_plan_for_simulation(self, build_plan_json_path):
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