import rosbag
from matplotlib import pyplot
import json

from droplet_underwater_assembly_libs.utils import angle_error_rads

#bag_file_path = "/home/sam/barbados2024/second_trial_in_ocean/day_3_ocean_2024-02-15-15-51-32.bag"

# contains a bad run where the robot lost control
#bag_file_path = "/home/sam/barbados2024/fifth_trial_in_ocean/day_5_tank_2024-02-18-15-51-00.bag"

# contains a good run
bag_file_path = "/home/sam/barbados2024/fifth_trial_in_ocean/day_5_tank_2024-02-18-15-43-46.bag"

bag_file = rosbag.Bag(bag_file_path)

yaw_errors = []
z_errors = []
positions = []
velocities = []
start_time = None

# bad run timeframe
#logging_start_seconds = 1708289560
#logging_end_seconds = 1708289612

# good run timeframe
logging_start_seconds = 1708289207
logging_end_seconds = 1708289248

target_x = -0.54

def time_in_range(time):
    return logging_start_seconds <= time.secs <= logging_end_seconds


for topic, message, time in bag_file.read_messages(['/fused_position', '/rosout', '/controller_state']):
    if start_time is None:
        start_time = time

    elapsed_seconds = (time - start_time).to_sec()

    if topic == '/fused_position' and time_in_range(time):
        yaw_error = angle_error_rads(message.relative_yaw, 3.14)
        yaw_errors.append((yaw_error, elapsed_seconds))

        z_errors.append(
            (message.position.z - 0.85, elapsed_seconds)
        )

        positions.append(((message.position.x - target_x, message.position.y, message.position.z), elapsed_seconds)) 

    if topic == '/controller_state':
        debug_data = json.loads(message.debug_data)
        velocity = debug_data['controller_velocity']

        if abs(velocity[0]) < 10:
            velocities.append((velocity, elapsed_seconds))

#pyplot.plot([i[1] for i in yaw_errors], [i[0] for i in yaw_errors])
#pyplot.plot([i[1] for i in z_errors], [i[0] for i in z_errors])

#pyplot.plot([i[1] for i in positions], [i[0][1] for i in positions], label='y')
#pyplot.plot([i[1] for i in positions], [i[0][0] for i in positions], label='x')
#pyplot.plot([i[1] for i in positions], [i[0][0] for i in positions], label='x')
pyplot.plot([i[1] for i in velocities], [i[0][0] for i in velocities], label='x vel')
pyplot.plot([i[1] for i in velocities], [i[0][1] for i in velocities], label='y vel')
pyplot.legend()

pyplot.show()
