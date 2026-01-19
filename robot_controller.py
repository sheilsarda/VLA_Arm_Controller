from csv import reader
import numpy as np
from robot_initializer import RobotInitializer
from path_planner import PathPlanner
from time import sleep
# If we want to run this in sim, then use `robot_comms_for_ur_sim.py`, else use stubbed functions in `robot_communication.py`
# from robot_comms_for_ur_sim import *
from robot_communication import * 

DEFAULT_HEIGHT = 500
DEFUALT_DEPTH_FROM_RAIL = 500
ROBOT_MODULE_WIDTH_ON_RAIL = 500

class RobotController:

    def __init__(self):

        self.grex_location_dict = {}

        self.load_grex_locations("grex_location_data.csv")
        self.load_rail_positions("module_data.csv")
        self.robot_initializer = RobotInitializer()
        self.robot_initializer.create_socket_and_initialize_robot()
        self.path_planner = PathPlanner()

        self.workspace_obstacles_to_consider_per_target = {
            'Incubator': ['Add Media', 'Output for Scientist'],
            'Output for Scientist': ['Incubator', 'Sample'],
            'Sample': ['Output for Scientist', 'Incubator'], # 'Sample' and 'Add Media' are the same position, so we don't consider the other one in obstacle checks when planning a move to either
            'Add Media': ['Output for Scientist', 'Incubator'],
        }

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

                self.grex_location_dict[grex_location_name] = {
                    'position': position,
                    'euler_angles': euler_angles_deg,
                    'rail_position': -1.0, # shall populate from `module_data.csv`
                    'module_width': -1.0, # shall populate from `module_data.csv`
                    'height': DEFAULT_HEIGHT,
                    'depth_from_rail': DEFUALT_DEPTH_FROM_RAIL,
                }

    def load_rail_positions(self, module_data_file_list: str) -> None:
        """
        Loads the obstacles from the obstacle file list.
        """
        with open(module_data_file_list, 'r', encoding='utf-8') as f:
            r = reader(f)
            next(r)  # Skip header row
            for row in r:
                obstacle_name = row[0]
                if obstacle_name not in self.grex_location_dict:
                    print(f"Note: Grex location {obstacle_name} not found")
                    continue
                rail_position = float(row[1] if row[1] != '-' else 0)
                module_width = float(row[2] if row[2] != '-' else 0)
                self.grex_location_dict[obstacle_name]['rail_position'] = rail_position
                self.grex_location_dict[obstacle_name]['module_width'] = module_width


if __name__ == "__main__":
    controller = RobotController()

    while True:
        for target_name, target_info in controller.grex_location_dict.items():
            current_joint_position = get_angular_pose()
            trajectory = controller.path_planner.plan_trajectory(
                current_joint_position, target_info['position'], target_info['euler_angles'])
            
            current_rail_bounds = [get_rail_pose() - ROBOT_MODULE_WIDTH_ON_RAIL/2, get_rail_pose() + ROBOT_MODULE_WIDTH_ON_RAIL/2]
            obstacles_to_consider = []
            for name_of_obstacle in controller.workspace_obstacles_to_consider_per_target[target_name]:
                obstacles_to_consider.append(controller.grex_location_dict[name_of_obstacle])

            controller.path_planner.check_trajectory_against_workspace_obstacles(
                trajectory,
                current_joint_position,
                current_rail_bounds,
                obstacles_to_consider)
            
            print(f"Computed waypoints for move from {current_joint_position} to {target_name}: {target_info['position']} | euler angles {target_info['euler_angles']} | rail position {target_info['rail_position']} | module width {target_info['module_width']}")
            sleep(5)
            
