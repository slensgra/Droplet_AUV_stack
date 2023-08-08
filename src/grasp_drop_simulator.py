#!/usr/bin/python3
import numpy as np

import roslib
roslib.load_manifest('localization_informed_planning_sim')
import actionlib
import rospy
import nav_msgs.msg

import localization_informed_planning_sim.msg

class GraspSimulatorNode():
    grasp_duration_seconds = 2.0

    def __init__(self, acceptance_area_radius):
        self.acceptance_area_radius = acceptance_area_radius
        self.ground_truth_position_subscriber = rospy.Subscriber(
                '/rexrov/pose_gt',
                nav_msgs.msg.Odometry,
                self.ground_truth_position_callback
        )
        self.fused_position_subscriber = rospy.Subscriber(
            '/fused_position',
            localization_informed_planning_sim.msg.BreadcrumbLocalizationResult,
            self.breadcrumb_callback
        )

        self.action_server = actionlib.SimpleActionServer(
            'attempt_grasp_server',
            localization_informed_planning_sim.msg.AttemptGraspAction,
            self.attempt_grasp,
            False
        )

    def breadcrumb_callback(self, message):
        self.latest_num_markers = message.num_visible_markers

    def ground_truth_position_callback(self, message):
        self.latest_ground_truth_reading = message

    def get_grasp_error(self, goal_pose):
        robot_position = np.array([
            self.latest_ground_truth_reading.pose.pose.position.x,
            self.latest_ground_truth_reading.pose.pose.position.y,
            self.latest_ground_truth_reading.pose.pose.position.z
        ])

        goal_pose_np = np.array([
            goal_pose.x,
            goal_pose.y,
            goal_pose.z
        ])

        return np.linalg.norm(robot_position - goal_pose_np)

    def attempt_grasp(self, goal):
        rospy.loginfo("Attempting grasp with goal: " + str(goal))
        grasp_start = rospy.Time.now()

        while (rospy.Time.now() - grasp_start).to_sec() < GraspSimulatorNode.grasp_duration_seconds:
            if self.latest_num_markers < 2:
                rospy.logerr("NOT ENOUGH MARKERS DURING GRASP!!!!!!!!!!!!")

            if self.get_grasp_error(goal) > self.acceptance_area_radius:

                if self.action_server.is_active():
                    self.action_server.set_aborted()
                    rospy.sleep(0.05)

        self.action_server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('grasp_simulator_node')
    grasp_simulator_node = GraspSimulatorNode(3.5)
    grasp_simulator_node.action_server.start()

    rospy.spin()
