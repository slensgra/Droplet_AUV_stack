import rospy

rospy.init_node('roll_pitch_republisher')

imu_subscriber = rospy.Subscriber('/mini_ahrs_ros/imu', sensor_msgs.msg.Imu, imu_callback)
