# import os
# import sys
# current_dir = os.path.dirname(os.path.abspath(__file__))
# map_folder_path = os.path.join(current_dir, "../map")
# sys.path.append(map_folder_path)

import rospy
import numpy as np
from typing import List
from map.map import local_map_grid_model
from robot.robot import robot_dict
from geometry_msgs.msg import Twist

class GridModel:
    def __init__(self, robot_id=0) -> None:
        self.robot_id = robot_id
        self.robot_name = robot_dict[robot_id]
        self.occupied_grid_matrix = np.zeros((2,2), dtype=int)
        
        self.want_to_enter_conflict = False
        self.enter_permission = False


    def decision_maker(self, twist_from_img: Twist):
        """
        Decision logic:
        """
        # If not yet in the conflict zone, return the original control command
        if not self.want_to_enter_conflict:
            return twist_from_img        
        if not self.enter_permission:
            return Twist()
        else:
            return twist_from_img

    def check_enter_permission(self, robot_info_list):
        """
        Check the status of robots in the queue:
        """
        curr_route = []

        # Step1 record which grids are already occupied
        for robot_info in robot_info_list:            
            # reccord curr route and skip self
            if robot_info.robot_id == self.robot_id:
                curr_route = [route for route in robot_info.robot_route]
                continue

            # When some one robot has already entered the conflict zone
            if robot_info.robot_enter_conflict:
                route = robot_info.robot_route
                occupied_grid = local_map_grid_model(robot_info.route[0], robot_info.route[1], robot_info.route[2])
                self.record_occupied_grid(occupied_grid=occupied_grid)

        # Step2 check grid conflict 
        enter_permission = self.check_occupied_grid_conflict(curr_route=curr_route)
        if enter_permission:
            rospy.logwarn_once(f"{self.robot_name} obtain permission to enter conflict area")
            self.occupied_grid_matrix = np.zeros((2,2), dtype=int)
        else:
            rospy.logwarn(f"{self.robot_name} must wait")  

        return enter_permission
    
    def record_occupied_grid(self, occupied_grid: List[int]):
        for grid_id in occupied_grid:
            matrix_coord = self.grid_id_to_matrix_coord(grid_id=grid_id)
            self.occupied_grid_matrix[matrix_coord[0], matrix_coord[1]] = 1
        return
    
    def check_occupied_grid_conflict(self, curr_route):
        grid_want_to_occupy = local_map_grid_model(curr_route[0], curr_route[1], curr_route[2])
        for grid_id in grid_want_to_occupy:
            matrix_coord = self.grid_id_to_matrix_coord(grid_id=grid_id)
            if self.occupied_grid_matrix[matrix_coord[0], matrix_coord[1]] == 1:
                return False
        return True

    def grid_id_to_matrix_coord(self, grid_id):
        binary_grid_id = format(grid_id - 1, '02b')
        matrix_coord = int(binary_grid_id[0]), int(binary_grid_id[1])        
        return matrix_coord


if __name__ == '__main__':
    decison_model = GridModel(robot_id=1)
    decison_model.record_occupied_grid(occupied_grid=(3, 4))
    decison_model.record_occupied_grid(occupied_grid=(1,))    
    print(decison_model.occupied_grid_matrix)
    decison_model.grid_want_to_occupy = [2]
    result = decison_model.check_occupied_grid_conflict(curr_route=[5,2,6])
    print(result)