import threading

import rospy
from droplet_underwater_assembly_libs import gripper_handler
import actionlib
from droplet_underwater_assembly_libs import config

import localization_informed_planning_sim.srv


class GripperHandlerNode():
    def __init__(self):
        rospy.init_node('gripper_handler_node')
        self.gripper_handler = gripper_handler.GripperHandler()

        self.action_server = actionlib.SimpleActionServer(
            'actuate_gripper',
            localization_informed_planning_sim.msg.ActuateGripperAction,
            self.start_actuating_gripper,
            False
        )

        self.plunge_service_proxy = rospy.ServiceProxy('plunge_action', localization_informed_planning_sim.srv.PlungeAction)

        self.is_open = True

    def start_actuating_gripper(self, goal):
        if goal.position == 'open':
            self.open_fingers(goal)
        elif goal.position == 'close':
            self.close_fingers(goal)
        else:
            rospy.logerr('Invalid gripper position given to node: {}'.format(goal.position))
            self.action_server.set_aborted()

    def open_fingers(self, goal):
        if self.is_open:
            self.action_server.set_succeeded()
            return

        self.gripper_handler.start_opening_fingers()
        while self.gripper_handler.is_opening:
            pass

        self.is_open = True
        self.action_server.set_succeeded()

    def close_fingers(self, goal):
        plunge_thread = threading.Thread(target=self.plunge_service_proxy, args=(config.GRIPPER_OPEN_TIME,))
        plunge_thread.start()

        if not self.is_open:
            self.action_server.set_succeeded()
            return

        self.gripper_handler.start_closing_fingers()
        while self.gripper_handler.is_closing:
            pass

        self.is_open = False
        self.action_server.set_succeeded()
        plunge_thread.join()
    
    def run(self):
        while not rospy.is_shutdown():
            self.gripper_handler.update()
            rospy.sleep(0.05)


if __name__ == '__main__':
    gripper_handler_node = GripperHandlerNode()
    gripper_handler_node.run()