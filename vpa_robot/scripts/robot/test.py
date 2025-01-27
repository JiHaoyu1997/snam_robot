import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
map_folder_path = os.path.abspath(os.path.join(current_dir, '../map'))
sys.path.append(map_folder_path)
from map import local_map_grid_model, find_lane_total_distance, local_map_path_length_in_gird_table
from robot import robot_dict, RobotInfo


import rospy

import time
import numpy as np
from typing import List
from geometry_msgs.msg import Twist

class GridBasedOptimalModel:
    def __init__(self, robot_id=0) -> None:
        self.robot_id = robot_id
        self.robot_name = robot_dict[robot_id]
        self.ranking = 0
        self.higher_ranking_robot_id_list = []
        self.curr_route = []
        self.estimated_arrival_conflict_time = 0.0
        self.occupied_grid_matrix = np.zeros((2,2), dtype=int)
        self.occupied_grid_time_matrix = np.zeros((2,2), dtype=float)        
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

    def check_enter_permission(self, local_inter_info):
        """
        Check the status of robots in the queue:
        """
        self.occupied_grid_matrix = np.zeros((2,2), dtype=int)
        robot_id_list = local_inter_info.robot_id_list
        robot_info_list = local_inter_info.robot_info

        # Step1 search higher ranking robot(s)
        self.search_higher_ranking_robot(robot_id_list=robot_id_list)

        # Step2 record which grids are already occupied
        for robot_id in self.higher_ranking_robot_id_list:
            for robot_info in robot_info_list:            
                if robot_info.robot_id == robot_id:              
                    # When some one robot has already entered the conflict zone, record occupied time
                    if robot_info.robot_enter_conflict:          
                        self.mapping_occupied_grid_and_time_matrix(robot_info)              

        
        # Step3 check grid conflict time
        enter_permission = self.check_occupied_grid_conflict(curr_route=self.curr_route)
        if enter_permission:
            rospy.logwarn(f"{self.robot_name} obtain permission to enter conflict area")
            self.occupied_grid_matrix = np.zeros((2,2), dtype=int)
        else:
            rospy.logwarn(f"{self.robot_name} must wait")  
        return enter_permission
    
    def search_higher_ranking_robot(self, robot_id_list: List[int]):
        self.ranking = robot_id_list.index(self.robot_id)
        self.higher_ranking_robot_id_list = robot_id_list[:self.ranking]
        print(self.ranking, self.higher_ranking_robot_id_list)
        return
    
    def mapping_occupied_grid_and_time_matrix(self, robot_info):
        p = robot_info.robot_p
        v = robot_info.robot_v
        route = robot_info.robot_route
        occupied_grid = local_map_grid_model(route[0], route[1], route[2])
        occupied_grid_time = self.calc_occupied_grid_time(p=p, v=v, route=route, occupied_grid=occupied_grid)

        for index, grid_id in enumerate(occupied_grid):
            matrix_coord = self.grid_id_to_matrix_coord(grid_id=grid_id)
            self.occupied_grid_matrix[matrix_coord[0], matrix_coord[1]] = 1
            self.occupied_grid_time_matrix[matrix_coord[0], matrix_coord[1]] = occupied_grid_time[index]

    def calc_occupied_grid_time(self, p, v, route, occupied_grid):
        occupied_grid_time = []
        lane_total_distance = find_lane_total_distance(last=route[0], current=route[1], next=route[2])
        distance_to_stop_line = p - lane_total_distance

        for grid in occupied_grid:
            path_length_in_gird = local_map_path_length_in_gird_table(route[0], route[1], route[2], grid)
            distance_to_exit_gird = path_length_in_gird - distance_to_stop_line
            if v == 0.0:
                v == 0.1
            duration_to_exit_grid = distance_to_exit_gird / v
            time_now = 0.0
            time_to_exit_grid = time_now + duration_to_exit_grid
            occupied_grid_time.append(round(time_to_exit_grid, 3))

        return occupied_grid_time
        
    def check_occupied_grid_conflict(self, curr_route):
        grid_want_to_occupy = local_map_grid_model(curr_route[0], curr_route[1], curr_route[2])
        for grid_id in grid_want_to_occupy:
            matrix_coord = self.grid_id_to_matrix_coord(grid_id=grid_id)
            if self.occupied_grid_matrix[matrix_coord[0], matrix_coord[1]] == 1:
                print(matrix_coord)
                conflict_grid_time = self.occupied_grid_time_matrix[matrix_coord[0], matrix_coord[1]]
                print(conflict_grid_time)
                est_arrive_conflict_grid_time = 1.1
                if est_arrive_conflict_grid_time > conflict_grid_time:
                    continue
                else:
                    return False
            else:
                continue
        return True

    def grid_id_to_matrix_coord(self, grid_id):
        binary_grid_id = format(grid_id - 1, '02b')
        matrix_coord = int(binary_grid_id[0]), int(binary_grid_id[1])        
        return matrix_coord


if __name__ == '__main__':
    decison_model = GridBasedOptimalModel(robot_id=1)
    decison_model.robot_id = 3
    decison_model.search_higher_ranking_robot([4,2,3,5,6])
    robot_info = RobotInfo(
        robot_id=4,
        robot_route=[4, 3, 5], 
        v=0.3, 
        p=0.3532, 
        enter_conflict=True,
    )
    decison_model.mapping_occupied_grid_and_time_matrix(robot_info=robot_info)
    print(decison_model.occupied_grid_matrix)
    print(decison_model.occupied_grid_time_matrix)




    # decison_model.record_occupied_grid(occupied_grid=(4, 1))
    # decison_model.record_occupied_grid(occupied_grid=(1, 3))
    # decison_model.occupied_grid_time_matrix[1,1]=0.5
    # decison_model.occupied_grid_time_matrix[0,0]=1
    # decison_model.occupied_grid_time_matrix[0,0]=1.5
    # result = decison_model.check_occupied_grid_conflict(curr_route=[1,3,5])
    # print(result)