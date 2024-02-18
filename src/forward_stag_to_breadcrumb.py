#! /usr/bin/python

import math
import collections

from tf import transformations
import rospy
import localization_informed_planning_sim.msg
import stag_ros.msg
import geometry_msgs.msg


tracked_marker_id = 8

breadcrumb_publisher = None
stag_subscriber = None

def marker_callback(message):
    for marker in message.markers:
        if marker.id == tracked_marker_id:
            quaternion = [
                marker.pose.pose.orientation.x,
                marker.pose.pose.orientation.y,
                marker.pose.pose.orientation.z,
                marker.pose.pose.orientation.w,
            ]
            yaw = transformations.euler_from_quaternion(quaternion)[2]
            breadcrumb_message = localization_informed_planning_sim.msg.BreadcrumbLocalizationResult()

            breadcrumb_message.position = geometry_msgs.msg.Point(
                x=-marker.pose.pose.position.x,
                y=marker.pose.pose.position.y,
                z=marker.pose.pose.position.z
            )
            breadcrumb_message.relative_yaw = yaw
            breadcrumb_message.structure_yaw = yaw
            breadcrumb_message.num_visible_markers = 1
            breadcrumb_message.header.stamp = rospy.Time.now()
            breadcrumb_publisher.publish(breadcrumb_message)


def main():
    global breadcrumb_publisher, stag_subscriber
    rospy.init_node('stag_forwarder')
    breadcrumb_publisher = rospy.Publisher(
        '/fused_position',
        localization_informed_planning_sim.msg.BreadcrumbLocalizationResult,
        queue_size=1
    )

    stag_subscriber = rospy.Subscriber(
        '/bluerov_controller/ar_tag_detector_2',
        stag_ros.msg.StagMarkers,
        marker_callback,
        queue_size=1,
    )

    rospy.spin()

if __name__ == '__main__':
    main()
