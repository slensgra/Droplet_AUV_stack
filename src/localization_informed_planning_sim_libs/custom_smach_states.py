import rospy

import smach

import localization_informed_planning_sim.srv


class PauseState(smach.State):
    def __init__(self, pause_seconds, outcomes=['succeeded', 'aborted']):
        smach.State.__init__(self, outcomes=outcomes)
        self.pause_seconds = pause_seconds

    def execute(self, userdata):
        rospy.sleep(self.pause_seconds)
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