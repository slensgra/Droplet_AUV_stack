#!/usr/bin/python

import json

import rospy
from std_msgs.msg import String
from localization_informed_planning_sim.srv import UiTubeCommand

from localization_informed_planning_sim_libs.ui_tube_wrapper import UITubeController

class UITubeNode:
    def __init__(self):
        self.tube_controller = UITubeController()
        
        #self.command_topic = '/ui_tube_command'
        self.command_service = '/ui_tube_command'

    def command_callback(self, msg):
        rospy.logdebug("UI tube updating with state {}".format(msg.data))
        command_parsed = json.loads(msg.data)

        success = False
        if command_parsed['type'] == 'led':
            try:
                self.tube_controller.set_led(command_parsed['pattern'], command_parsed['r'], command_parsed['g'], command_parsed['b'])
                success = True
            except Exception as e:
                success = True
                rospy.logerr("failed to write! {}".format(e))
        elif command_parsed['type'] == 'headlamps':
            try:
                self.tube_controller.set_headlamps(command_parsed['brightnesses'])
                success = True
            except Exception as e:
                success = True
                rospy.logerr("failed to write! {}".format(e))
        else:
            rospy.logerr('Invalid command type: {}'.format(command_parsed['type']))

        return success

    def run(self):
        rospy.init_node('ui_tube_node')
        # self.command_subscriber = rospy.Subscriber(self.command_topic, String, self.command_callback)
        self.command_server = rospy.Service(self.command_service, UiTubeCommand, self.command_callback)
        rospy.spin()

if __name__ == '__main__':
    node = UITubeNode()
    node.run()