import rosbag
import json
from tf import transformations

from droplet_underwater_assembly_libs import utils

bag_file_path = "/home/sam/barbados2024/fourth_trial_in_ocean/day_4_tank_2024-02-17-14-40-45.bag"

bag_file = rosbag.Bag(bag_file_path)

start_time = None
latest_imu = None

for topic, message, time in bag_file.read_messages(['/rosout', '/controller_state', '/mini_ahrs_ros/imu']):
    if start_time is None:
        start_time = time

    if topic == '/mini_ahrs_ros/imu':
        latest_imu = message

    if topic == '/controller_state':
        parsed_debug = json.loads(message.debug_data)

        if parsed_debug['angle_error_from_imu'][0] > 0.5:
            print("~")
            print(parsed_debug['angle_error_from_imu'])
            print((time - start_time).to_sec())
            print(latest_imu)
            orientation = [
                latest_imu.orientation.x,
                latest_imu.orientation.y,
                latest_imu.orientation.z,
                latest_imu.orientation.w
            ]

            accel = [
                latest_imu.linear_acceleration.x,
                latest_imu.linear_acceleration.y,
                latest_imu.linear_acceleration.z,
            ]

            euler =  transformations.euler_from_quaternion(
                orientation, 
                'sxyz'
            )

            print('direct euler', euler)

            angle_error_roll = utils.angle_error_rads(euler[0], 0.0)
            angle_error_pitch = utils.angle_error_rads(euler[1], 0.0)

            print('error', angle_error_roll, angle_error_pitch)

            print('estimated roll, pitch', utils.get_roll_pitch_from_acceleration_vector(accel))
