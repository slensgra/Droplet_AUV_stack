import rosbag
import json
from matplotlib import pyplot

bag_file_path = "/home/sam/barbados2024/third_trial_in_ocean/day_4_tank_2024-02-16-11-49-31.bag"
#bag_file_path = "/home/sam/barbados2024/third_trial_in_ocean/day_4_tank_2024-02-16-12-10-36.bag"

bag_file = rosbag.Bag(bag_file_path)

thrusts = []
errors = []
angle_errors = []
start_time = None

for topic, message, time in bag_file.read_messages(['/controller_state', '/rosout']):
    if start_time is None:
        start_time = time

    elapsed_seconds = (time - start_time).to_sec()

    if topic == '/controller_state':
        parsed_debug_data = json.loads(message.debug_data)
        angle_errors.append((
            parsed_debug_data['angle_error_from_imu'],
            elapsed_seconds
        ))

pyplot.plot([x[1] for x in angle_errors], [x[0][0] for x in angle_errors], label='roll_e')
pyplot.plot([x[1] for x in angle_errors], [x[0][1] for x in angle_errors], label='pitch_e')
pyplot.xlabel('seconds')
pyplot.ylabel('angle')

pyplot.legend()
pyplot.show()
