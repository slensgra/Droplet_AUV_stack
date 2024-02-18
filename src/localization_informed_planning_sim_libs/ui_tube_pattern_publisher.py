import rospy
import json

from localization_informed_planning_sim.srv import UiTubeCommand


class Flipper():
    def __init__(self, flip_time, states):
        self.active_state_idx = 0
        self.states = states
        self.flip_time = flip_time
        self.last_flip = rospy.Time.now()

    @property
    def time_since_last_flip(self):
        return (rospy.Time.now() - self.last_flip).to_sec()

    def do_flip(self):
        self.active_state_idx = (self.active_state_idx + 1) % len(self.states)
        self.last_flip = rospy.Time.now()

    def should_flip(self):
        return self.time_since_last_flip > self.flip_time

    def get_state(self):
        flipped = False
        if self.should_flip():
            self.do_flip()
            flipped = True

        return flipped, self.states[self.active_state_idx]


class UITubePatternFlipper():
    def __init__(self, patterns, flip_time):
        self.flipper = Flipper(
            flip_time=flip_time,
            states=patterns
        )
        self.service_name = '/ui_tube_command'
        self.service_client = rospy.ServiceProxy(self.service_name, UiTubeCommand)

    def turn_headlamps_off(self):
        pattern_dict = {'type': 'headlamps', 'brightnesses': [0,0,0,0]}
        self.update_ui_tube(pattern_dict)

    def update_ui_tube(self, pattern_dict):
        message_string = json.dumps(pattern_dict)
        rospy.wait_for_service(self.service_name)
        response = self.service_client(data=message_string)

        if not response:
            rospy.logerr("Could not update ui tube with command: {}".format(message_string))

        return True

    def update(self):
        flipped, next_state = self.flipper.get_state()
        if flipped:
            self.update_ui_tube(next_state)

        return flipped, next_state