#!/usr/bin/python
import collections
import numpy as np

import roslib
roslib.load_manifest('localization_informed_planning_sim')
import rospy
import smach_ros

import droplet_underwater_assembly_libs.utils
import localization_informed_planning_sim.srv
import localization_informed_planning_sim.msg
from localization_informed_planning_sim_libs.real_world_build_plan_constructor import RealWorldBuildPlanConstructor

Block = collections.namedtuple('Block', ['id', 'position', 'index'])
Marker = collections.namedtuple('Marker', ['id', 'position', 'index'])

class AssemblyProcessManager():
    def __init__(self):
        rospy.init_node('assembly_process_manager')

        self.structure_state = {}

        self.set_controller_target_client = rospy.ServiceProxy(
            '/set_goal_position',
            localization_informed_planning_sim.srv.SetControllerTarget
        )

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

if __name__ == '__main__':
    manager = AssemblyProcessManager()
    real_world_build_plan_constructor = RealWorldBuildPlanConstructor()
    #state_machine = real_world_build_plan_constructor.get_manipulator_test_state_machine()
    #state_machine = real_world_build_plan_constructor.get_controller_test_state_machine()
    state_machine = real_world_build_plan_constructor.one_hop_plan()

    introspection_server = smach_ros.IntrospectionServer('assembly_state_machine', state_machine, '/SM_ROOT')
    introspection_server.start()

    state_machine.execute()