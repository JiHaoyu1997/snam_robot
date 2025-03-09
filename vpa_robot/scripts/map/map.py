#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This function returns the action item from the last, current and future node
# action:
    # 0: through 
    # 1: left 
    # 2: right
import rospy
from map.route import ROUTE_TABLE, ROUTE_CP_TABLE, CP_COORDINATE_TABLE, CP_TABLE

def local_mapper(last: int, current: int, next: int) -> int:

    node_set = [last, current, next]

    if node_set == [1,2,3]:
        return 1
    if node_set == [1,2,5]:
        return 0
    if node_set == [1,3,5]:
        return 0
    if node_set == [1,3,2]:
        return 2 
    if node_set == [1,3,4]:
        return 1
    if node_set == [1,4,3]:
        return 2
    if node_set == [1,4,5]:
        return 0
    
    if node_set == [2,1,4]:
        return 0
    if node_set == [2,1,3]:
        return 2    
    if node_set == [2,3,4]:
        return 0
    if node_set == [2,3,1]:
        return 1
    if node_set == [2,3,5]:
        return 2    
    if node_set == [2,5,4]:
        return 0
    if node_set == [2,5,3]:
        return 1
    
    if node_set == [3,1,2]:
        return 1
    if node_set == [3,1,4]:
        return 2
    if node_set == [3,2,1]:
        return 2
    if node_set == [3,2,5]:
        return 1
    if node_set == [3,5,2]:
        return 2
    if node_set == [3,5,4]:
        return 1
    if node_set == [3,4,5]:
        return 2
    if node_set == [3,4,1]:
        return 1
    
    if node_set == [4,1,2]:
        return 0
    if node_set == [4,1,3]:
        return 1
    if node_set == [4,3,2]:
        return 0
    if node_set == [4,3,1]:
        return 2
    if node_set == [4,3,5]:
        return 1
    if node_set == [4,5,3]:
        return 2
    if node_set == [4,5,2]:
        return 0
    
    if node_set == [5,2,1]:
        return 0
    if node_set == [5,2,3]:
        return 2
    if node_set == [5,3,1]:
        return 0
    if node_set == [5,3,2]:
        return 1
    if node_set == [5,3,4]:
        return 2
    if node_set == [5,4,3]:
        return 1
    if node_set == [5,4,1]:
        return 0
    
    if node_set == [6,2,1]:
        return 1
    if node_set == [6,2,3]:
        return 0
    if node_set == [6,2,5]:
        return 2
    
    if node_set == [1,2,6]:
        return 2
    if node_set == [3,2,6]:
        return 0
    if node_set == [5,2,6]:
        return 1
    
    return None


def local_map_grid_model(last: int, current: int, next: int):

    node_set = [last, current, next]

    # Inter1
    if node_set == [2,1,3]:
        return [1]
    
    if node_set == [2,1,4]:
        return [1, 3]
    
    if node_set == [3,1,2]:
        return [3, 2]
    
    if node_set == [3,1,4]:
        return [3]
    
    if node_set == [4,1,2]:
        return [4, 2]
    
    if node_set == [4,1,3]:
        return [4, 1]

    # Inter2
    if node_set == [1,2,3]:
        return [2, 3]
    
    if node_set == [1,2,5]:
        return [2, 1]
    
    if node_set == [1,2,6]:
        return [2]

    if node_set == [3,2,1]:
        return [4]
    
    if node_set == [3,2,5]:
        return [4, 1]
    
    if node_set == [3,2,6]:
        return [4, 2]
    
    if node_set == [5,2,1]:
        return [3, 4]
    
    if node_set == [5,2,3]:
        return [3]
    
    if node_set == [5,2,6]:
        return [3, 2]
    
    if node_set == [6,2,1]:
        return [1, 4]
    
    if node_set == [6,2,3]:
        return [1, 3]
    
    if node_set == [6,2,5]:
        return [1]
    
    # Inter3
    if node_set == [1,3,2]:
        return [2]
    
    if node_set == [1,3,4]:
        return [2, 3]
    
    if node_set == [1,3,5]:
        return [2, 1]
    
    if node_set == [2,3,1]:
        return [1, 4]
    
    if node_set == [2,3,4]:
        return [1, 3]    
    
    if node_set == [2,3,5]:
        return [1]    

    if node_set == [4,3,1]:
        return [4]
    
    if node_set == [4,3,2]:
        return [4, 2]
        
    if node_set == [4,3,5]:
        return [4, 1]
    
    if node_set == [5,3,1]:
        return [3, 4]
    
    if node_set == [5,3,2]:
        return [3, 2]
    
    if node_set == [5,3,4]:
        return [3]

    # Inter4
    if node_set == [1,4,3]:
        return [2]
    
    if node_set == [1,4,5]:
        return [2, 1]
    
    if node_set == [3,4,1]:
        return [1, 4]
    
    if node_set == [3,4,5]:
        return [1]
    
    if node_set == [5,4,1]:
        return [3, 4]
    
    if node_set == [5,4,3]:
        return [3, 2]   

    # Inter5
    if node_set == [2,5,3]:
        return [1, 4]
    
    if node_set == [2,5,4]:
        return [1, 3]
    
    if node_set == [3,5,2]:
        return [2]
    
    if node_set == [3,5,4]:
        return [2, 3]

    if node_set == [4,5,2]:
        return [4, 2]
    
    if node_set == [4,5,3]:
        return [4]

    return None


def find_lane_total_distance(last: int, current: int, next: int):

    node_set = [last, current, next]

    # Inter1
    if node_set == [2,1,3]:
        return 1.0602
    
    if node_set == [2,1,4]:
        return 1.0602
    
    if node_set == [3,1,2]:
        return 0.2731
    
    if node_set == [3,1,4]:
        return 0.2731
    
    if node_set == [4,1,2]:
        return [4, 2]
    
    if node_set == [4,1,3]:
        return [4, 1]

    # Inter2
    if node_set == [1,2,3]:
        return 1.4444
    
    if node_set == [1,2,5]:
        return 1.4444
    
    if node_set == [1,2,6]:
        return 1.4444

    if node_set == [3,2,1]:
        return [4]
    
    if node_set == [3,2,5]:
        return [4, 1]
    
    if node_set == [3,2,6]:
        return [4, 2]
    
    if node_set == [5,2,1]:
        return 1.1716
    
    if node_set == [5,2,3]:
        return 1.1716
    
    if node_set == [5,2,6]:
        return 1.1716
    
    if node_set == [6,2,1]:
        return 0.9849
    
    if node_set == [6,2,3]:
        return 0.9849
    
    if node_set == [6,2,5]:
        return 0.9849
    
    # Inter3
    if node_set == [1,3,2]:
        return 0.3302
    
    if node_set == [1,3,4]:
        return 0.3302
    
    if node_set == [1,3,5]:
        return 0.3302
    
    if node_set == [2,3,1]:
        return [1, 4]
    
    if node_set == [2,3,4]:
        return [1, 3]    
    
    if node_set == [2,3,5]:
        return [1]    

    if node_set == [4,3,1]:
        return 0.2532
    
    if node_set == [4,3,2]:
        return 0.2532
        
    if node_set == [4,3,5]:
        return 0.2532
    
    if node_set == [5,3,1]:
        return [3, 4]
    
    if node_set == [5,3,2]:
        return [3, 2]
    
    if node_set == [5,3,4]:
        return [3]

    # Inter4
    if node_set == [1,4,3]:
        return [2]
    
    if node_set == [1,4,5]:
        return [2, 1]
    
    if node_set == [3,4,1]:
        return [1, 4]
    
    if node_set == [3,4,5]:
        return [1]
    
    if node_set == [5,4,1]:
        return 1.4426
    
    if node_set == [5,4,3]:
        return 1.4426

    # Inter5
    if node_set == [2,5,3]:
        return 1.5394
    
    if node_set == [2,5,4]:
        return 1.5394
    
    if node_set == [3,5,2]:
        return 0.2732
    
    if node_set == [3,5,4]:
        return 0.2732

    if node_set == [4,5,2]:
        return [4, 2]
    
    if node_set == [4,5,3]:
        return [4]

    return 0.0


def local_map_path_length_in_gird_table(last: int, current: int, next: int, grid: int):

    node_set = [last, current, next]

    # Inter1
    if node_set == [2,1,3]:
        return [1]
    
    if node_set == [2,1,4]:
        return [1, 3]
    
    if node_set == [3,1,2]:
        return 0.2731
    
    if node_set == [3,1,4]:
        return [3]
    
    if node_set == [4,1,2]:
        return [4, 2]
    
    if node_set == [4,1,3]:
        return [4, 1]

    # Inter2
    if node_set == [1,2,3]:
        return [2, 3]
    
    if node_set == [1,2,5]:
        return [2, 1]
    
    if node_set == [1,2,6]:
        return 1.4444

    if node_set == [3,2,1]:
        return [4]
    
    if node_set == [3,2,5]:
        return [4, 1]
    
    if node_set == [3,2,6]:
        return [4, 2]
    
    if node_set == [5,2,1]:
        return [3, 4]
    
    if node_set == [5,2,3]:
        return [3]
    
    if node_set == [5,2,6]:
        return [3, 2]
    
    if node_set == [6,2,1]:
        return [1, 4]
    
    if node_set == [6,2,3]:
        return [1, 3]
    
    if node_set == [6,2,5]:
        return 1.0562
    
    # Inter3
    if node_set == [1,3,2]:
        return [2]
    
    if node_set == [1,3,4]:
        return [2, 3]
    
    if node_set == [1,3,5]:
        return [2, 1]
    
    if node_set == [2,3,1]:
        return [1, 4]
    
    if node_set == [2,3,4]:
        return [1, 3]    
    
    if node_set == [2,3,5]:
        return [1]    

    if node_set == [4,3,1]:
        return 0.2532
    
    if node_set == [4,3,2]:
        return [4, 2]
        
    if node_set == [4,3,5]:
        if grid == 4:
            return 0.25
        if grid == 1:
            return 0.50
    
    if node_set == [5,3,1]:
        return [3, 4]
    
    if node_set == [5,3,2]:
        return [3, 2]
    
    if node_set == [5,3,4]:
        return [3]

    # Inter4
    if node_set == [1,4,3]:
        return [2]
    
    if node_set == [1,4,5]:
        return [2, 1]
    
    if node_set == [3,4,1]:
        return [1, 4]
    
    if node_set == [3,4,5]:
        return [1]
    
    if node_set == [5,4,1]:
        return [3, 4]
    
    if node_set == [5,4,3]:
        return 1.4426

    # Inter5
    if node_set == [2,5,3]:
        return [1, 4]
    
    if node_set == [2,5,4]:
        return 1.5394
    
    if node_set == [3,5,2]:
        return [2]
    
    if node_set == [3,5,4]:
        return [2, 3]

    if node_set == [4,5,2]:
        return [4, 2]
    
    if node_set == [4,5,3]:
        return [4]

    return 0.0


def find_conflict_point_list(route):
    inter_id = route[1]
    cp_list = CP_TABLE[inter_id][route]
    return cp_list

def find_conflict_point(route_i, route_j):
    if route_i[1] != route_j[1]:
        print(route_i, route_j)
        rospy.logwarn("Agents not in the same Intersection")
    
    
    curr_inter_id = route_i[1]
    route_i_id = ROUTE_TABLE[curr_inter_id][route_i]
    route_j_id = ROUTE_TABLE[curr_inter_id][route_j]
    conflict_point  = ROUTE_CP_TABLE[curr_inter_id][route_i_id][route_j_id]
    return conflict_point

def find_conflict_point_coordinate(inter, cp):
    cp_coor = CP_COORDINATE_TABLE[inter][cp]
    return cp_coor


if __name__ == "__main__":
    cp_list = find_conflict_point_list(inter=3, route=(2,3,5) )
    print(cp_list)