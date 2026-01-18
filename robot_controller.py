from csv import reader
import numpy as np
from robot_initializer import RobotInitializer
from robot_comms_for_sim_ur3 import *

class WorkspaceObstacle:

    def __init__(self, obstacle_file_list: str, grex_location_file_list: str):

        self.grex_location_list = []

        self.load_grex_locations("grex_location_data.csv")
        self.load_rail_positions("module_data.csv")
        self.robot_initializer = RobotInitializer()
        self.robot_initializer.create_socket_and_initialize_robot()

    def load_grex_locations(self, grex_location_file_list: str) -> None:
        """
        Loads the grex locations (position and euler angles in radians) into a dictionary.
        """
        with open(grex_location_file_list, 'r', encoding='utf-8') as f:
            r = reader(f)
            next(r)  # Skip header row
            for row in r:
                grex_location_name = row[0]
                position = [float(row[1]), float(row[2]), float(row[3])]
                euler_angles_deg = [float(row[4]), float(row[5]), float(row[6])]

                self.grex_location_list.append({
                    'name': grex_location_name,
                    'position': position,
                    'euler_angles': euler_angles_deg,
                    'rail_position': -1.0, # shall populate from `module_data.csv`
                    'module_width': -1.0, # shall populate from `module_data.csv`
                })

    def load_rail_positions(self, module_data_file_list: str) -> None:
        """
        Loads the obstacles from the obstacle file list.
        """
        with open(module_data_file_list, 'r', encoding='utf-8') as f:
            r = reader(f)
            next(r)  # Skip header row
            for row in r:
                obstacle_name = row[0]
                rail_position = float(row[1] if row[1] != '-' else 0)
                module_width = float(row[2] if row[2] != '-' else 0)
                grex_to_update = next((x for x in self.grex_location_list if x['name'] == obstacle_name), None)
                if grex_to_update is None:
                    print(f"Error: Grex location {obstacle_name} not found")
                    continue
                grex_to_update['rail_position'] = rail_position
                grex_to_update['module_width'] = module_width


if __name__ == "__main__":
    wc = WorkspaceObstacle("module_data.csv", "grex_location_data.csv")
    
    while True:
        for target in wc.grex_location_list:
            move_to_base_cartesian_position(
                target['position'][0], target['position'][1], target['position'][2], 
                target['euler_angles'][0], target['euler_angles'][1], target['euler_angles'][2])
            move_to_rail_pose(target['rail_position'])
            
            print(f"Commanded move to {target['name']}: {target['position']} | euler angles {target['euler_angles']} | rail position {target['rail_position']} | module width {target['module_width']}")
            time.sleep(5)
            
