import numpy as np

import rospy

class SimulationManager(object):
    def __init__(self):
        self.spawn_model_service_name = '/gazebo/spawn_sdf_model'
        self.delete_model_service_name = '/gazebo/delete_model'

        self.spawn_model_client = rospy.ServiceProxy(self.spawn_model_service_name, gazebo_msgs.srv.SpawnModel)
        self.delete_model_client = rospy.ServiceProxy(self.delete_model_service_name, gazebo_msgs.srv.DeleteModel)

    def spawn_structure_from_plan(self, structure_spec_json_path):
        with open(structure_spec_json_path, 'r') as f:
            structure_spec = json.load(f)
            self.cell_side_length = structure_spec['cell_side_length']

            normal_block_indices = [
                np.array([b['x'], b['y'], b['z']]) for b in structure_spec['structure'] if b['block_type'] == 'normal'
            ]

            marker_indices = [
                np.array([b['x'], b['y'], b['z']]) for b in structure_spec['structure'] if b['block_type'] == 'localization'
            ]

            self.block_position_by_id = {
                "block_{i}".format(i=i): self.get_position_from_slot_index(b) for i, b in enumerate(normal_block_indices)
            }

            if len(marker_indices) > 4:
                rospy.logerror("More than 4 markers are not supported!")
                raise Exception("More than 4 markers are not supported!")

            self.marker_position_by_id = {
                i: self.get_position_from_slot_index(b) for i, b in enumerate(marker_indices) 
            }

            marker_positions_and_names = [i for i in self.marker_position_by_id.items()]
            normal_positions_and_names = [i for i in self.block_position_by_id.items()]
            self.spawn_markers(
                marker_positions=[i[1] for i in marker_positions_and_names],
                marker_ids=[i[0] for i in marker_positions_and_names],
                marker_indices=[i for i in marker_indices]
            )

            self.spawn_blocks(
                block_positions=[i[1] for i in normal_positions_and_names],
                block_names=[i[0] for i in normal_positions_and_names],
                block_indices=[i for i in normal_block_indices]
            )