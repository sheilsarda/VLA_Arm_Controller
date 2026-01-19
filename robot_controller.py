from csv import reader
import numpy as np
from robot_initializer import RobotInitializer
from path_planner import PathPlanner
from time import sleep
from time import time

# If we want to run this in sim, then use `robot_comms_for_ur_sim.py`, else use stubbed functions in `robot_communication.py`
from robot_comms_for_ur_sim import *
# from robot_communication import * 

DEFAULT_HEIGHT = 500
DEFUALT_DEPTH_FROM_RAIL = 500
ROBOT_MODULE_WIDTH_ON_RAIL = 500

class RobotController:

    def __init__(self):

        self.grex_location_dict = {}

        self.load_grex_locations("grex_location_data.csv")
        self.load_rail_positions("module_data.csv")
        self.robot_initializer = RobotInitializer()
        self.path_planner = PathPlanner()

        """
        This dictionary is used to determine which grex objects are in the same "module" based on rail position. Additionally, we rule out 'Output for Scientist' as an obstacle in the same module, because `Add Media` is at -700mm Y, and `Output for Scientist` is at +400mm, therefore on the other side of the rail

        TODOs: 
        - investigate why the trajectory_planner.check_trajectory_against_workspace_obstacles() method says the trajectory planned from approximately 'Output for Scientist' to 'Add Media' collide with the axis aligned bounding box obstacle for. Hypotheses I've investigated thus far:
            - moving the robot arm to a neutral position away from the previous target (I tried moving it to the home position), to ensure the starting state was not in collision with the obstacle bounding boxes did not fix the issue
        - sometimes the arm times out on a final target position where several joints are ~0.1 - pi/2 radians away from commanded target 
        """
        self.workspace_obstacles_to_consider_per_target = { 
            'Output for Scientist': ['Add Media'],
            'Add Media': [],
            'Incubator': ['Sample'],
            'Sample': [],
            'Neutral Position': [],
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


    def is_robot_in_collision(self) -> bool:
        collision_statuses = ['PROTECTIVE_STOP', 'SAFEGUARD_STOP', 'SYSTEM_EMERGENCY_STOP', 'ROBOT_EMERGENCY_STOP', 'FAULT', 'AUTOMATIC_MODE_SAFEGUARD_STOP']
        return self.robot_initializer.status['safetystatus'] in collision_statuses

    def move_to_target(self, trajectory: List[List[float]]) -> None:
        """
        Moves the robot to the target, and returns either after a 20 second timeout, or when the max difference between 
        any joint's current position and the planned target is < 1e-3 radians.
        """
        move_timer = time()
        for waypoint in trajectory:
            if self.is_robot_in_collision():
                print("Robot is in collision, stopping movement; TODO: autorecover here")
                return
            move_to_angular_position(waypoint[0], waypoint[1], waypoint[2], waypoint[3], waypoint[4], waypoint[5])
            sleep(0.1)

        pose_difference = np.array(get_angular_pose()) - np.array(trajectory[-1])
        while time() - move_timer < 20 and max(pose_difference) > 1e-3:
            if self.is_robot_in_collision():
                print("Robot is in collision, stopping movement; TODO: autorecover here")
                return
            sleep(0.25)
            pose_difference = np.array(get_angular_pose()) - np.array(trajectory[-1])

        print(f"Diff between current pose and planned trajectory end is {[round(x, 2) for x in pose_difference]} radians")

    def plan_and_execute_trajectory(self, target_name: str) -> None:
        current_joint_position = get_angular_pose()
        target_info = self.grex_location_dict[target_name]
        trajectory = self.path_planner.plan_trajectory(
            current_joint_position, target_info['position'], target_info['euler_angles'])

        obstacles_to_consider = []
        for name_of_obstacle in self.workspace_obstacles_to_consider_per_target[target_name]:
            obs_data = self.grex_location_dict[name_of_obstacle].copy()
            obs_data['name'] = name_of_obstacle  # Include name for collision report
            obstacles_to_consider.append(obs_data)

        print(f"Computed {len(trajectory)} waypoints for move to {target_name}: {target_info['position']} | euler angles {target_info['euler_angles']}")

        collisions = self.path_planner.check_trajectory_against_workspace_obstacles(
            trajectory,
            get_rail_pose(),
            obstacles_to_consider)
        
        if collisions:
            print(f"WARNING: Trajectory to {target_name} has collisions: {collisions}")
            exit(1)
        
        self.move_to_target(trajectory)

if __name__ == "__main__":

    print("------------------ STARTING PATH PLANNING ------------------")
    controller = RobotController()
    while True:
        
        for target_name in controller.grex_location_dict.keys():
            controller.plan_and_execute_trajectory(target_name)
