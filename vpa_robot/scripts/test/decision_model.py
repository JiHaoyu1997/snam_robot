import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
map_folder_path = os.path.join(current_dir, "..")
sys.path.append(map_folder_path)
from map.map import local_map_grid_model, find_conflict_point_list, find_conflict_point, find_conflict_point_coordinate
from robot.robot import robot_dict, RobotInfo
from vscs import VSCS

import rospy
import math
import numpy as np
from typing import List

from geometry_msgs.msg import Twist


class VSCSModel:
    def __init__(self, robot_id=0) -> None:
        self.robot_id = robot_id
        self.robot_name = robot_dict[robot_id]
        self.L = []
        self.cp_matrix = []
        self.vscs_solver = VSCS()

    def generate_pass_cp_flag_dict(self, new_route):
        route_in_tuple = tuple(new_route)
        cp_list = find_conflict_point_list(route_in_tuple)
        self.pass_cp_flag_dict = {cp: False for cp in cp_list}
    
    def calc_twist(self, twist_from_img: Twist, robot_id_list: List[int], robot_info_list: List[RobotInfo]):
        for robot_info in robot_info_list:
            if robot_info.robot_id == self.robot_id:
                self.generate_pass_cp_flag_dict(robot_info.robot_route)
        
        N = len(robot_id_list)
        if N <= 1 :
             return twist_from_img
        else:
            twist = Twist()
            delta_t = 0.05
            self.L, self.cp_matrix = self.generate_L(robot_info_list=robot_info_list)
            controller_gain = self.calc_control_gain()
            cumulative_error = self.calc_cumulative_error(robot_id_list, robot_info_list)
            # print(cumulative_error)
            control_input = controller_gain @ cumulative_error
            print(control_input)
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

        rospy.loginfo_once('laplacian: \n%s', np.array2string(L))
        print(L)
        print(cp_matrix)
        return L, cp_matrix

    def calc_control_gain(self):
        K = self.vscs_solver.sol_lmi(self.L)
        K = np.array([
            [-5.19884665,  8.54126546]
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
                if self.pass_cp_flag_dict[cp]:
                    cumu_error += 0
                else:
                    eij = self.calc_eij(conflict_robot_id, cp, robot_info_list)
                    cumu_error += eij
        
        return cumu_error
    
    def calc_eij(self, j, cp, robot_info_list: List[RobotInfo]):
        cp_coor = find_conflict_point_coordinate(inter=3, cp=cp)
        lr = 0.4
        s_i, s_j, v_i, v_j = None, None, None, None
        
        for robot_info in robot_info_list:
            if robot_info.robot_id == j:
                coor_j = robot_info.robot_coordinate
                s_j = self.calc_distance_to_cp(cp_coor, coor_j)
                v_j = robot_info.robot_v

            if robot_info.robot_id == self.robot_id:
                coor_i = robot_info.robot_coordinate
                s_i = self.calc_distance_to_cp(cp_coor, coor_i)
                v_i = robot_info.robot_v
        
        if s_i is None or s_j is None or v_i is None or v_j is None:
            raise ValueError("Unable to find robot info for the given robot IDs.")
        
        if s_i < s_j:
            e_s = (s_j - s_i) - lr
        else:
            e_s = (s_j - s_i) + lr

        e_v = v_j - v_i

        eij = np.array([e_s, e_v]).reshape((2,1))
        error = np.linalg.norm(eij)
        print("error", error)

        
        pass_cp_flag = self.break_virtual_spring(s_i, s_j, error, cp)
        return eij
    
    def break_virtual_spring(self, s_i, s_j, error, cp):
        if s_i < 0.05:
            rospy.loginfo_once(f"{self.robot_name} self pass cp={cp}")
            self.pass_cp_flag_dict[cp] = True
            return 

        if s_j < 0.05:
            rospy.loginfo_once(f"{self.robot_name} conflicting robot pass cp={cp}")
            self.pass_cp_flag_dict[cp] = True
            return

        if error < 0.01:
            rospy.loginfo_once(f"{self.robot_name} befor cp={cp} reach harmony")
            self.pass_cp_flag_dict[cp] = True
            return
    
    def calc_distance_to_cp(self, cp_coor, robot_coor):
        distance_to_cp = math.sqrt((robot_coor[0] - cp_coor[0])**2 + (robot_coor[1] - cp_coor[1])**2)
        return distance_to_cp
                
                

if __name__ == '__main__':
    vscs = VSCSModel(robot_id=1)
    robot_id_list = [1, 2, 8]
    robot_info_list = [
        RobotInfo(
            name="mingna",
            robot_id=1,
            robot_route=(1, 3, 5),
            coordinate=(1.193, 1.208),
            v=0.28
        ),
        RobotInfo(
            name="vivian",
            robot_id=2,
            robot_route=(4, 3, 1),
            coordinate=(2.016, 0.71),
            v=0.25
        ),
        # RobotInfo(
        #     name="henry",
        #     robot_id=7,
        #     robot_route=(5, 3, 1),
        #     coordinate=(1.826, 2.216),
        #     v=0.31
        # ),
        RobotInfo(
            name="luna",
            robot_id=8,
            robot_route=(2, 3, 4),
            coordinate=(2.016, 0.71),
            v=0.25
        ),
    ]
    vscs.calc_twist(twist_from_img=Twist(), robot_id_list=robot_id_list, robot_info_list=robot_info_list)
    # vscs.generate_pass_cp_flag_dict(new_route=[2, 3, 4])
