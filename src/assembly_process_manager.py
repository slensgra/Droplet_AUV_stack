import rospy
import gazebo_msgs.srv
from geometry_msgs.msg import Pose, Quaternion, Point
import rospkg

class AssemblyProcessManager():
    ocean_depth_meters = 100.0 # lowest block at -99.6
    block_height_meters = 0.4
    lowest_block_height = -99.8

    def __init__(self):
        self.spawn_model_service_name = '/gazebo/spawn_sdf_model'
        self.spawn_model_client = rospy.ServiceProxy(self.spawn_model_service_name, gazebo_msgs.srv.SpawnModel)
        self.marker_model_path_format = rospkg.RosPack().get_path('localization_informed_planning_sim') + '/models/aruco_{marker_id}/model.sdf'
        self.block_model_path = rospkg.RosPack().get_path('localization_informed_planning_sim') + '/models/standard_block/model.sdf'

    def get_model_path_for_marker_id(self, marker_id):
        return self.marker_model_path_format.format(marker_id=marker_id)

    def spawn_blocks(self, block_positions, block_names):
        for i, block_position in enumerate(block_positions):
            self.spawn_model_client(
                    model_name=block_names[i],
                    model_xml=open(self.block_model_path, 'r').read(),
                    robot_namespace='/foo',
                    initial_pose=Pose(
                        position=Point(*block_position),
                        orientation=Quaternion(0,0,0,1)
                    ),
                    reference_frame='world'
            )

    def spawn_markers(self, marker_positions, marker_ids):
        for i, marker_position in enumerate(marker_positions):
            self.spawn_model_client(
                    model_name='marker_{}'.format(i),
                    model_xml=open(self.get_model_path_for_marker_id(marker_ids[i]), 'r').read(),
                    robot_namespace='/foo',
                    initial_pose=Pose(
                        position=Point(*marker_position),
                        orientation=Quaternion(0,0,0,1)
                    ),
                    reference_frame='world'
            )

if __name__ == '__main__':
    manager = AssemblyProcessManager()

    block_positions = [
        [i * 0.4, 0.0, AssemblyProcessManager.lowest_block_height] for i in range(10)
    ]

    marker_positions = [
        [i * 0.4, 0.4, AssemblyProcessManager.lowest_block_height] for i in range(2)
    ]

    block_names = list(map(str, range(len(block_positions))))
    marker_ids = list(map(str, range(len(block_positions))))

    #manager.spawn_blocks(block_positions, block_names)
    manager.spawn_markers(marker_positions, marker_ids)
