import rospy

from localization_informed_planning_sim.srv import UiTubeCommand
from localization_informed_planning_sim_libs.ui_tube_pattern_publisher import UITubePatternFlipper

rospy.init_node('ui_flipper_test')

patterns = [
    {'pattern': 'fade', 'type': 'led', 'r': 255, 'g': 0, 'b': 255},
    {'pattern': 'chase', 'type': 'led', 'r': 0, 'g': 0, 'b': 255},
]

flip_time = 3.0

pattern_flipper = UITubePatternFlipper(
    flip_time=flip_time,
    patterns=patterns
)

while not rospy.is_shutdown():
    flipped, s = pattern_flipper.update()

    if flipped:
        print("Flipped to: ", s)

    rospy.Time.sleep(0.05)