# import os
# import sys
# current_dir = os.path.dirname(os.path.abspath(__file__))
# map_folder_path = os.path.join(current_dir, "../map")
# sys.path.append(map_folder_path)
# from map import local_map_grid_model, find_conflict_point, find_conflict_point_coordinate
# from robot import robot_dict, RobotInfo

import math
import rospy
import numpy as np
from typing import List

from vscs import VSCS
from map.map import local_map_grid_model, find_conflict_point, find_conflict_point_coordinate
from robot.robot import robot_dict, RobotInfo

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
        
        
        self.vscs = VSCS()
        self.robot_id_list = []
        self.A = np.array([
            [0, -1],
            [0, -1]
            ])

        self.B = np.array([
            [0],
            [1]
            ])
    
    def record_robot_id_list(self, robo_id_list: List[int]):
        self.robot_id_list = robo_id_list

    def calc_control_gain(self, robot_info_list: List[RobotInfo]):
        N = len(robot_info_list)
        if N == 1:
            if robot_info_list[0].robot_id == self.robot_id:
                k = (0, 0)
                return k
            else:
                raise ValueError("agent self info not in inter_info_list")
            
        else:
            K = self.calc_k(robot_info_list, N)
            return K
        
    def calc_state_error(self, robot_info_list: List[RobotInfo]):
        for robot_info in robot_info_list:
            if robot_info.robot_id == self.robot_id:
                self.curr_coor = robot_info.robot_coordinate

        # x_i = np.array([s_i])

    
    def generate_cp_state_matrix(self, robot_info_list: List[RobotInfo]):
        N = len(robot_info_list)
        cp_flag_matrix = np.zeros((N, N))
        for i in range(N):
            for j in range(i + 1, N):
                robot_route_i = robot_info_list[i].robot_route
                robot_route_j = robot_info_list[j].robot_route
                cp = find_conflict_point(route_i=robot_route_i, route_j=robot_route_j)
                if cp != 0:
                    cp_flag_matrix[i][j] = 1

        cp_state_matrix = cp_flag_matrix + cp_flag_matrix.T
        print(cp_state_matrix)

    def calc_k(self, robot_info_list, N):
        h = self.generate_h_dict(robot_info_list=robot_info_list)
        H = self.generate_matrix_H(h=h, n_t=N)
        K = self.vscs.sol_lmi_position_based(A=self.A, B=self.B, H=H)
        print(K)
        return K

    def generate_matrix_H(self, h: dict, n_t=3):
        H = np.zeros((n_t, n_t))

        for (i, j), value in h.items():
            H[i, j] = - value
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
        print(H)
        return H
        
    def generate_h_dict(self, robot_info_list):
        h = {}
        n = len(robot_info_list)
        
        for i in range(n):
            for j in range(i + 1, n):
                h[i,j] = self.calc_hij(i=i, j=j, robot_info_list=robot_info_list)
        
        # print(h)
        return h

    def calc_hij(self, i, j, robot_info_list: List[RobotInfo]):
        robot_route_i = robot_info_list[i].robot_route
        robot_route_j = robot_info_list[j].robot_route
        inter = robot_route_i[1]
        cp = find_conflict_point(route_i=robot_route_i, route_j=robot_route_j)
        # print(cp)

        if cp == 0:
            hij = 0
        else:
            cp_coor = find_conflict_point_coordinate(inter=inter, cp=cp)
            s_i = self.calc_distance_to_cp(cp_coor, robot_info_list[i])
            s_j = self.calc_distance_to_cp(cp_coor, robot_info_list[j])
            # print(s_i, s_j)
            bij = self.calc_bij(s_i, s_j)
            # print(bij)
            rij = self.cacl_rij(s_i, s_j)
            # print(rij)
            hij = bij * rij
            
        return hij

    def calc_distance_to_cp(self, cp_coor, robot_info: RobotInfo):
        robot_coor = robot_info.robot_coordinate
        distance_to_cp = math.sqrt((robot_coor[0] - cp_coor[0])**2 + (robot_coor[1] - cp_coor[1])**2)
        return distance_to_cp
    
    def calc_bij(self, s_i, s_j):
        if s_i < s_j:
            bij = 1
        elif s_i > s_j:
            bij = -1
        else:
            # TODO
            bij = 1
        return bij
    
    def cacl_rij(self, s_i, s_j):
        lij = abs(s_i - s_j)
        lr = 40

        if lij < lr:
            rij = 1
        elif lij == lr:
            rij = 0
        else:
            rij = -1
        return rij


if __name__ == '__main__':
    decision_model = VSCSModel(robot_id=1)
    robot_info_list = [
        RobotInfo(
            name="mingna",
            robot_id=1,
            robot_route=(2, 3, 4),
            coordinate=(1.193, 1.208)
        ),
        RobotInfo(
            name="henry",
            robot_id=7,
            robot_route=(1, 3, 5),
            coordinate=(1.826, 2.216)
        ),
        RobotInfo(
            name="luna",
            robot_id=8,
            robot_route=(5, 3, 1),
            coordinate=(2.016, 0.71)
        )
    ]
    decision_model.calc_control_gain(robot_info_list=robot_info_list)
    decision_model.calc_state_error(robot_info_list=robot_info_list)