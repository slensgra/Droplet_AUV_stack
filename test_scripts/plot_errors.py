import rosbag
from matplotlib import pyplot

from droplet_underwater_assembly_libs.utils import angle_error_rads

bag_file_path = "/home/sam/barbados2024/second_trial_in_ocean/day_3_ocean_2024-02-15-15-51-32.bag"

bag_file = rosbag.Bag(bag_file_path)

yaw_errors = []
z_errors = []
start_time = None

for topic, message, time in bag_file.read_messages(['/fused_position', '/rosout']):
    if start_time is None:
        start_time = time

    if topic == '/fused_position':
        elapsed_seconds = (time - start_time).to_sec()
        yaw_error = angle_error_rads(message.relative_yaw, 3.14)
        yaw_errors.append((yaw_error, elapsed_seconds))

        z_errors.append(
            (message.position.z - 0.85, elapsed_seconds)
        )

pyplot.plot([i[1] for i in yaw_errors], [i[0] for i in yaw_errors])
pyplot.plot([i[1] for i in z_errors], [i[0] for i in z_errors])
pyplot.show()
