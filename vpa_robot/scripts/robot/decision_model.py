import rospy

import math
import numpy as np
from typing import List

from map.map import (
    local_mapper, 
    local_map_grid_model, 
    find_conflict_point_list, 
    find_conflict_point, 
    find_conflict_point_coordinate
)
from robot.robot import robot_dict, RobotInfo
# from vscs import VSCS

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
        self.curr_route = ()
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

    def check_enter_permission(self, curr_route, robot_info_list: List[RobotInfo]):
        """
        Check the status of robots in the queue:
        """
        self.occupied_grid_matrix = np.zeros((2,2), dtype=int)

        # Step1 record which grids are already occupied
        for robot_info in robot_info_list:            
            if robot_info.robot_enter_conflict:
                route = robot_info.robot_route
                print(route)
                if route == self.curr_route:
                    print(f"same route with {robot_info.robot_name}")
                    continue
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
        self.L = []
        self.cp_matrix = []
        self.break_virtual_spring_flag_dict = {}
        self.last_control_input = 0

    def update_break_virtual_spring_flag_dict(self, new_robot_id_list):
        if len(new_robot_id_list) <= 1:
            rospy.loginfo(f"only {self.robot_name} in Inter3")
            return

        if len(self.break_virtual_spring_flag_dict) == 0:
            rospy.loginfo("init break_virtual_spring_flag_dict")

        for other_id in new_robot_id_list:
            if other_id != self.robot_id:
                key = (self.robot_id, other_id)
                if key not in self.break_virtual_spring_flag_dict:
                    self.break_virtual_spring_flag_dict[key] = False

    def generate_pass_cp_flag_dict(self, new_route):
        route_in_tuple = tuple(new_route)
        cp_list = find_conflict_point_list(route_in_tuple)
        self.pass_cp_flag_dict = {cp: False for cp in cp_list}
    
    def calc_twist(self, twist_from_img: Twist, robot_id_list: List[int], robot_info_list: List[RobotInfo]):
        N = len(robot_id_list)
        if N <= 1 :
             return twist_from_img
        else:
            twist = Twist()
            delta_t = 0.05
            self.L, self.cp_matrix = self.generate_L(robot_info_list=robot_info_list)
            controller_gain = self.calc_control_gain(N)
            cumulative_error = self.calc_cumulative_error(robot_id_list, robot_info_list)
            # print(cumulative_error)
            calc_control_input = controller_gain @ cumulative_error
            truncated_control_input = np.clip(calc_control_input, -6, 4)
            beta = 0.75
            control_input = self.last_control_input * (1 - beta) + truncated_control_input * beta
            self.last_control_input = control_input
            # print(control_input)
            delta_v = control_input * delta_t
            # print(delta_v)
            twist.linear.x = 0.3 + delta_v
            twist.angular.z = twist_from_img.angular.z
            return twist
    
    def generate_L(self, robot_info_list: List[RobotInfo]):
        N = len(robot_info_list)
        L = np.zeros((N, N), dtype=int)
        cp_matrix = np.zeros((N, N), dtype=int)
        for i in range(N):
            for j in range(i + 1, N):
                robot_route_i = robot_info_list[i].robot_route
                robot_route_j = robot_info_list[j].robot_route
                cp = find_conflict_point(route_i=robot_route_i, route_j=robot_route_j)
                if cp != 0:
                    L[i][j] = -1
                    L[j][i] = -1
                    cp_matrix[i][j] = cp
                    cp_matrix[j][i] = cp

        for i in range(N):
            L[i][i] = -np.sum(L[i])

        # rospy.loginfo_once('laplacian: \n%s', np.array2string(L))
        # print(cp_matrix)
        return L, cp_matrix

    def calc_control_gain(self, N):
        # K = self.vscs_solver.sol_lmi(self.L)
        if N == 2:
            K = np.array([
                [-5.19884665,  8.54126546]
                ])
        else:    
            K = np.array([
                [-11.72353634,  18.81520133]
                ])
        return K

    def calc_cumulative_error(self, robot_id_list: List[int], robot_info_list: List[RobotInfo]):
        cumu_error = np.zeros((2,1))
        id_postion = robot_id_list.index(self.robot_id) 
        conflict_list = self.L[id_postion]
        cp_list = self.cp_matrix[id_postion]
        # print(conflict_list)
        # print(cp_list)
        
        for idx, conflict in enumerate(conflict_list):
            if conflict == -1:
                conflict_robot_id = robot_id_list[idx]
                cp = cp_list[idx]
                key = (self.robot_id, conflict_robot_id)
                if self.break_virtual_spring_flag_dict[key]:
                    cumu_error += 0
                else:
                    eij = self.calc_eij(conflict_robot_id, cp, robot_info_list)
                    cumu_error += eij
        
        return cumu_error
    
    def calc_eij(self, j, cp, robot_info_list: List[RobotInfo]):
        cp_coor = find_conflict_point_coordinate(inter=3, cp=cp)
        lr = 0.75
        s_i, s_j, v_i, v_j = None, None, None, None
        
        for robot_info in robot_info_list:
            if robot_info.robot_id == j:
                route_j = robot_info.robot_route
                # factor = self.find_s_scaling_factor(route_j)
                coor_j = robot_info.robot_coordinate
                s_j = self.calc_distance_to_cp(cp_coor, coor_j)
                # print(f"{robot_info.robot_name}: {s_j}")
                v_j = robot_info.robot_v

            if robot_info.robot_id == self.robot_id:
                route_i = robot_info.robot_route
                # factor = self.find_s_scaling_factor(route_i)
                coor_i = robot_info.robot_coordinate
                s_i = self.calc_distance_to_cp(cp_coor, coor_i)
                # print(f"{robot_info.robot_name}: {s_i}")
                v_i = robot_info.robot_v
        
        if s_i is None or s_j is None or v_i is None or v_j is None:
            raise ValueError("Unable to find robot info for the given robot IDs.")
        
        if abs(s_j - s_i) > 1.5:
            return 0
        
        if s_i < s_j:
            e_s = (s_j - s_i) - lr
        else:
            e_s = (s_j - s_i) + lr

        e_v = v_j - v_i

        eij = np.array([e_s, e_v]).reshape((2,1))
        error = abs(e_s)

        pass_cp_flag = self.break_virtual_spring(s_i, s_j, j, error, cp)
        return eij
    
    def break_virtual_spring(self, s_i, s_j, other_id, error, cp):
        key = (self.robot_id, other_id)

        if s_i < 0.05:
            rospy.loginfo_once(f"{self.robot_name} self pass cp={cp}")
            self.break_virtual_spring_flag_dict[key] = True
            return 

        if s_j < 0.05:
            rospy.loginfo_once(f"{self.robot_name} conflicting robot {robot_dict[other_id]} pass cp={cp}")
            self.break_virtual_spring_flag_dict[key] = True
            return

        if error < 0.05:
            rospy.loginfo_once(f"{self.robot_name} with {robot_dict[other_id]} befor cp={cp} reach harmony")
            self.break_virtual_spring_flag_dict[key] = True
            return
    
    def find_s_scaling_factor(self, route):
        action =  local_mapper(last=route[0], current=route[1], next=route[2])
        if action == 0:
            return 1
        elif action == 1:
            return 1.1
        else:
            return 1.05

    def calc_distance_to_cp(self, cp_coor, robot_coor):
        distance_to_cp = math.sqrt((robot_coor[0] - cp_coor[0])**2 + (robot_coor[1] - cp_coor[1])**2)
        return distance_to_cp
                

if __name__ == '__main__':
    vscs = VSCSModel(robot_id=1)
    robot_id_list = [1, 7]
    robot_info_list = [
        RobotInfo(
            name="luna",
            robot_id=8,
            robot_route=(5, 3, 1),
            v=0.25
        ),
        RobotInfo(
            name="mingna",
            robot_id=1,
            robot_route=(2, 3, 4),
            coordinate=(1.193, 1.208),
            v=0.28
        ),
        RobotInfo(
            name="henry",
            robot_id=7,
            robot_route=(1, 3, 5),
            coordinate=(1.826, 2.216),
            v=0.31
        ),
    ]
    vscs.calc_twist(twist_from_img=Twist(), robot_id_list=robot_id_list, robot_info_list=robot_info_list)
