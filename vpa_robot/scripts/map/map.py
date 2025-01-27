#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This function returns the action item from the last, current and future node
# action:
    # 0: through 
    # 1: left 
    # 2: right

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