import rospy
import math
import sys

from localization_informed_planning_sim_libs.custom_smach_states import UpdateMarkerPositionState

rospy.init_node('marker_updater')

slot_stride = 0.205

marker_positions = [
    UpdateMarkerPositionState(
        marker_id=2,
        position=[-9.0 * slot_stride, 0.0, 0.0],
        yaw=0.0,
        masked=True,
    ),
    UpdateMarkerPositionState(
        marker_id=2,
        position=[-9.0 * slot_stride, 0.0, 0.0],
        yaw=0.0,
        masked=False,
    ),
    UpdateMarkerPositionState(
        marker_id=3,
        position=[-12.0 * slot_stride, 0.0, 0.0, 0.0],
        yaw=0.0,
        masked=True,
    ),
    UpdateMarkerPositionState(
        marker_id=3,
        position=[-12.0 * slot_stride, 0.0, 0.0, math.pi],
        yaw=math.pi,
        masked=False,
    ),
]

move_number = int(sys.argv[1])

print("sending breadcrumb update for position number {}".format(move_number))
update_object = marker_positions[move_number]

print('executing')
update_object.execute(None)
print('done')
