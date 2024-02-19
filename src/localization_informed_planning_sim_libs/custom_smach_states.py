import rospy
import numpy as np

import smach

import localization_informed_planning_sim.srv
import droplet_underwater_assembly_libs
import actionlib

from localization_informed_planning_sim.msg import ActuateGripperAction, ActuateGripperGoal
from localization_informed_planning_sim_libs.ui_tube_pattern_publisher import UITubePatternFlipper


class PauseState(smach.State):
    def __init__(self, pause_seconds, outcomes=['succeeded', 'aborted']):
        smach.State.__init__(self, outcomes=outcomes)
        self.pause_seconds = pause_seconds
        self.pattern_flipper = UITubePatternFlipper(
            patterns=[
                {'pattern': 'solid', 'type': 'led', 'r': 255, 'g': 255, 'b': 255},
                {'pattern': 'solid', 'type': 'led', 'r': 0, 'g': 0, 'b': 0}
            ],
            flip_time=1.0
        )

    def execute(self, userdata):
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < self.pause_seconds:
            self.pattern_flipper.update()
            rospy.sleep(0.02)

        return 'succeeded'


class DespawnBlockState(smach.State):
    def __init__(self, target_block_id, simulation_manager, outcomes=['succeeded', 'aborted']):
        smach.State.__init__(self, outcomes=outcomes)
        self.target_block_id = target_block_id
        self.simulation_manager = simulation_manager

    def execute(self, userdata):
        self.simulation_manager.despawn_model(self.target_block_id)
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


class FieldIdleState(smach.State):
    def __init__(self, target_position, outcomes=['succeeded']):
        smach.State.__init__(self, outcomes=outcomes)
        self.target_position = target_position
        self.error_range = 0.22

        self.breadcrumb_position_topic = '/fused_position'
        self.breadcrumb_subscriber = rospy.Subscriber(
            self.breadcrumb_position_topic,
            localization_informed_planning_sim.msg.BreadcrumbLocalizationResult,
            self.breadcrumb_callback,
        )
        self.last_breadcrumb_time = None
        self.pattern_flipper = UITubePatternFlipper(
            patterns=[
                {'pattern': 'solid', 'type': 'led', 'r': 0, 'g': 255, 'b': 255},
                {'pattern': 'solid', 'type': 'led', 'r': 0, 'g': 0, 'b': 255},
            ],
            flip_time=1.0
        ) 

        self.gripper_action_client = actionlib.SimpleActionClient('actuate_gripper', ActuateGripperAction)

    def error_is_in_range(self, breadcrumb):
        position_error = np.array([
            breadcrumb.position.x,
            breadcrumb.position.y,
            breadcrumb.position.z,
        ]) - np.array(self.target_position[0:3])

        position_error_norm = np.linalg.norm(position_error)
        yaw_error = droplet_underwater_assembly_libs.utils.angle_error_rads(breadcrumb.relative_yaw, self.target_position[5])
        yaw_error_norm = abs(yaw_error)

        in_range = (position_error_norm < self.error_range) and (yaw_error_norm < 0.4)

        return in_range

    def breadcrumb_is_stale(self):
        return self.last_breadcrumb_time is None or (rospy.Time.now() - self.last_breadcrumb_time).to_sec() > 1.0

    def breadcrumb_callback(self, message):
        self.last_breadcrumb_time = rospy.Time.now()
        self.last_breadcrumb = message

    def execute(self, data):
        droplet_underwater_assembly_libs.utils.set_motor_arming(False)
        self.pattern_flipper.turn_headlamps_off()

        self.gripper_action_client.wait_for_server()
        goal = ActuateGripperGoal(position='open', plunge=False)

        self.gripper_action_client.send_goal(goal)

        while self.breadcrumb_is_stale() or not self.error_is_in_range(self.last_breadcrumb):
            self.pattern_flipper.update()
            rospy.sleep(0.02)

        droplet_underwater_assembly_libs.utils.set_motor_arming(True)
        return 'succeeded'

class UpdateMarkerPositionState(smach.State):
    def __init__(self, marker_id, position, masked, yaw, outcomes=['succeeded', 'aborted']):
        smach.State.__init__(self, outcomes=outcomes)
        self.marker_id = marker_id
        self.position = position 
        self.masked = masked
        self.yaw = yaw

        self.set_marker_position_client = rospy.ServiceProxy(
            'set_marker_position',
            localization_informed_planning_sim.srv.SetGlobalPosition
        )

    def send_marker_position_to_breadcrumb(self):
        self.set_marker_position_client(
            marker_id=self.marker_id,
            x=self.position[0],
            y=self.position[1], # intentionally done bc of frame mismatch
            z=self.position[2],
            yaw=self.yaw,
            masked=self.masked
        )

    def execute(self, userdata):
        self.send_marker_position_to_breadcrumb()
        return 'succeeded'
