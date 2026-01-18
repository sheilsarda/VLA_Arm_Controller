from csv import reader
import numpy as np
from robot_initializer import RobotInitializer
from robot_comms_for_sim_ur3 import *

class WorkspaceObstacle:

    def __init__(self, obstacle_file_list: str, grex_location_file_list: str):
        self.obstacle_file_list = obstacle_file_list
        self.grex_location_file_list = grex_location_file_list
        self.obstacle_list = []
        self.grex_location_list = []
        self.load_obstacles()
        self.load_grex_locations()
        self.robot_initializer = RobotInitializer()
        self.robot_initializer.create_socket_and_initialize_robot()

    def load_obstacles(self) -> None:
        """
        Loads the obstacles from the obstacle file list.
        """
        with open(self.obstacle_file_list, 'r', encoding='utf-8') as f:
            r = reader(f)
            next(r)  # Skip header row
            for row in r:
                obstacle_name = row[0]
                rail_position = float(row[1] if row[1] != '-' else 0)
                module_width = float(row[2] if row[2] != '-' else 0)
                self.obstacle_list.append({
                    'name': obstacle_name,
                    'position': rail_position,
                    'width': module_width
                })

    def load_grex_locations(self) -> None:
        """
        Loads the grex locations (position and euler angles in radians) into a dictionary.
        """
        with open(self.grex_location_file_list, 'r', encoding='utf-8') as f:
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
                })
            self.grex_location_list.insert(0, {
                'name': 'start',
                'position': [0, 0, 0],
                'euler_angles': [0, 0, 0],
            })

if __name__ == "__main__":
    wc = WorkspaceObstacle("module_data.csv", "grex_location_data.csv")
    
    iters = 0
    while True:
        target = wc.grex_location_list[iters % len(wc.grex_location_list)]
        move_to_base_cartesian_position(
            target['position'][0], target['position'][1], target['position'][2], 
            target['euler_angles'][0], target['euler_angles'][1], target['euler_angles'][2])
        time.sleep(5)
        print(get_base_cartesian_pose(), get_base_cartesian_velocity())
        print(get_angular_pose(), get_angular_velocity())
        print(get_tool_voltage())
        iters += 1
    # print(wc.obstacle_list)
    # print(wc.grex_location_list)