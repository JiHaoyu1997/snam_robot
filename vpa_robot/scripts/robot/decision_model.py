# import os
# import sys
# current_dir = os.path.dirname(os.path.abspath(__file__))
# map_folder_path = os.path.join(current_dir, "../map")
# sys.path.append(map_folder_path)
# from map import local_map_grid_model
# from robot import robot_dict

import rospy
import numpy as np
from typing import List
from map.map import local_map_grid_model
from robot.robot import robot_dict
from geometry_msgs.msg import Twist

class FIFOModel:
    def __init__(self, robot_id=0) -> None:
        self.robot_id = robot_id
        self.robot_name = robot_dict[robot_id]
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
        for robot_info in robot_info_list:
            # skip self
            if robot_info.robot_id == self.robot_id:
                continue

            # A robot has already entered the conflict zone; this robot cannot pass
            if robot_info.robot_enter_conflict:
                return False 
            
        # No robots have entered the conflict zone; this robot can pass
        rospy.loginfo(f"{self.robot_name} obtain permission to enter conflict area")
        return True 
    

class GridBasedModel:
    def __init__(self, robot_id=0) -> None:
        self.robot_id = robot_id
        self.robot_name = robot_dict[robot_id]
        self.curr_route = []
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
        self.occupied_grid_matrix = np.zeros((2,2), dtype=int)

        # Step1 record which grids are already occupied
        for robot_info in robot_info_list:            
            # reccord curr route and skip self
            if robot_info.robot_id == self.robot_id:
                self.curr_route  = [route for route in robot_info.robot_route]
                continue

            # When some one robot has already entered the conflict zone
            if robot_info.robot_enter_conflict:
                route = robot_info.robot_route
                occupied_grid = local_map_grid_model(route[0], route[1], route[2])
                self.record_occupied_grid(occupied_grid=occupied_grid)

        # Step2 check grid conflict 
        enter_permission = self.check_occupied_grid_conflict(curr_route=self.curr_route)
        if enter_permission:
            rospy.logwarn(f"{self.robot_name} obtain permission to enter conflict area")
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


class GridBasedOptimalModel:
    def __init__(self, robot_id=0) -> None:
        self.robot_id = robot_id
        self.robot_name = robot_dict[robot_id]
        self.curr_route = []
        self.estimated_arrival_conflict_time = 0.0
        self.occupied_grid_matrix = np.zeros((2,2), dtype=int)
        self.occupied_grid_time_matrix = np.zeros((2,2), dtype=int)
        
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
        self.occupied_grid_matrix = np.zeros((2,2), dtype=int)

        # Step1 record which grids are already occupied
        for robot_info in robot_info_list:            
            # reccord curr route and skip self
            if robot_info.robot_id == self.robot_id:
                self.curr_route  = [route for route in robot_info.robot_route]
                continue

            # When some one robot has already entered the conflict zone
            if robot_info.robot_enter_conflict:
                route = robot_info.robot_route
                occupied_grid = local_map_grid_model(route[0], route[1], route[2])
                self.record_occupied_grid(occupied_grid=occupied_grid)

        # Step2 check grid conflict 
        enter_permission = self.check_occupied_grid_conflict(curr_route=self.curr_route)
        if enter_permission:
            rospy.logwarn(f"{self.robot_name} obtain permission to enter conflict area")
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
            else:
                continue
        return True

    def grid_id_to_matrix_coord(self, grid_id):
        binary_grid_id = format(grid_id - 1, '02b')
        matrix_coord = int(binary_grid_id[0]), int(binary_grid_id[1])        
        return matrix_coord
    

class VSCSModel:
    def __init__(self, robot_id=0) -> None:
        self.robot_id = robot_id
        self.robot_name = robot_dict[robot_id]
        self.curr_route = []
        self.init_system_dynamic_matrix()
    
    def init_system_dynamic_matrix(self):
        self.A = np.array([
            [0, -1],
            [0, -1]
            ])

        self.B = np.array([
            [0],
            [1]
            ])
        
        print("init system dynamic matrix")

    def generate_matrix_H(self, h: dict, n_t=3):
        H = np.zeros((n_t, n_t))

        for (i, j), value in h.items():
            if i < j:
                H[i, j] = -value
                H[j, i] = value
                
        for i in range(n_t):
            s = 0.0            
            for j in range(n_t):
                if i == j:
                    continue
                if i < j:
                    s += h.get((i, j), 0)
                else:
                    s += -h.get((j, i), 0)
            H[i, i] = s

        return H
    
    
    def generate_param_hij(self, robot_info_list):
        h = {}
        n = len(robot_info_list)
        
        for i in range(n - 1):
            for j in range(i+1, n):
                h[i,j]


    def calc_hij(self, i, j, robot_info_list):
        robot_info_i = robot_info_list[i]
        robot_info_j = robot_info_list[j]


    def calc_sigma_max(self, robot_info_list):
        h = {
            (0, 1):  1,
            (0, 2): -1,
            (1, 2):  0,
        }

        H = self.generate_H(h=h)
        print(H)
        U, s, Vt = np.linalg.svd(H)
        norm_H = s[0]
        print(norm_H)
        return norm_H


if __name__ == '__main__':
    decison_model = GridBasedOptimalModel(robot_id=1)
    decison_model.record_occupied_grid(occupied_grid=(4, 1))
    print(decison_model.occupied_grid_matrix)
    decison_model.grid_want_to_occupy = [2]
    result = decison_model.check_occupied_grid_conflict(curr_route=[5,3,1])
    print(result)