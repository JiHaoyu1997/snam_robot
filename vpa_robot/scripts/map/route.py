import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
map_folder_path = os.path.join(current_dir, "../map")
sys.path.append(map_folder_path)
from map import local_map_grid_model

import numpy as np


ROUTE_TABLE = {
    # Intersection 1
    1: {
        (2, 1, 3): 0,
        (2, 1, 4): 1,
        (3, 1, 2): 2,
        (3, 1, 4): 3,
        (4, 1, 2): 4,
        (4, 1, 3): 5,
    },
    # Intersection 2
    2: {
        (1, 2, 3): 0,
        (1, 2, 5): 1,
        (1, 2, 6): 2,
        (3, 2, 1): 3,
        (3, 2, 5): 4,
        (3, 2, 6): 5,
        (5, 2, 1): 6,
        (5, 2, 3): 7,
        (5, 2, 6): 8,
        (6, 2, 1): 9,
        (6, 2, 3): 10,
        (6, 2, 5): 11,
    },
    # Intersection 3
    3: {
        (1, 3, 2): 0,
        (1, 3, 4): 1,
        (1, 3, 5): 2,
        (2, 3, 1): 3,
        (2, 3, 4): 4,
        (2, 3, 5): 5,
        (4, 3, 1): 6,
        (4, 3, 2): 7,
        (4, 3, 5): 8,
        (5, 3, 1): 9,
        (5, 3, 2): 10,
        (5, 3, 4): 11,
    },
    # Intersection 4
    4: {
        (1, 4, 3): 0,
        (1, 4, 5): 1,
        (3, 4, 1): 2,
        (3, 4, 5): 3,
        (5, 4, 1): 4,
        (5, 4, 3): 5,
    },
    # Intersection 5
    5: {
        (2, 5, 3): 0,
        (2, 5, 4): 1,
        (3, 5, 2): 2,
        (3, 5, 4): 3,
        (4, 5, 2): 4,
        (4, 5, 3): 5,
    },
}

INTER_1_CP_TABLE = np.array([
    [0, 1, 0, 0, 0, 1],
    [1, 0, 3, 3, 0, 1],
    [0, 3, 0, 3, 2, 5],
    [0, 3, 3, 0, 0, 0],
    [0, 0, 2, 0, 0, 4],
    [1, 1, 5, 0, 4, 0]
])

INTER_2_CP_TABLE = np.array([
    [0, 2, 2, 0, 5, 2, 3, 3, 6, 5, 3, 0],
    [2, 0, 2, 0, 1, 2, 0, 0, 2, 1, 1, 1],
    [2, 2, 0, 0, 0, 2, 0, 0, 2, 0, 0, 0],
    [0, 0, 0, 0, 4, 4, 4, 0, 0, 4, 0, 0],
    [5, 1, 0, 4, 0, 4, 4, 0, 5, 6, 1, 1],
    [2, 2, 2, 4, 4, 0, 4, 0, 2, 4, 0, 0],
    [3, 0, 0, 4, 4, 4, 0, 3, 3, 4, 3, 0],
    [3, 0, 0, 0, 0, 0, 3, 0, 3, 0, 3, 0],
    [6, 2, 2, 0, 5, 2, 3, 3, 0, 5, 3, 0],
    [5, 1, 0, 4, 6, 4, 4, 0, 5, 0, 1, 1],
    [3, 1, 0, 0, 1, 0, 3, 3, 3, 1, 0, 1],
    [0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0]
])

INTER_3_CP_TABLE = np.array([
    [0, 2, 2, 0, 0, 0, 0, 2, 0, 0, 2, 0],
    [2, 0, 2, 5, 3, 0, 0, 2, 5, 3, 6, 3],
    [2, 2, 0, 1, 1, 1, 0, 2, 1, 0, 2, 0],
    [0, 5, 1, 0, 1, 1, 4, 4, 6, 4, 5, 0],
    [0, 3, 1, 1, 0, 1, 0, 0, 1, 3, 3, 3],
    [0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 4, 0, 0, 0, 4, 4, 4, 0, 0],
    [2, 2, 2, 4, 0, 0, 4, 0, 4, 4, 2, 0],
    [0, 5, 1, 6, 1, 1, 4, 4, 0, 4, 5, 0],
    [0, 3, 0, 4, 3, 0, 4, 4, 4, 0, 3, 3],
    [2, 6, 2, 5, 3, 0, 0, 2, 5, 3, 0, 3],
    [0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 3, 0]
])

INTER_4_CP_TABLE = np.array([
    [0, 2, 0, 0, 0, 2],
    [2, 0, 1, 1, 0, 2],
    [0, 1, 0, 1, 4, 5],
    [0, 1, 1, 0, 0, 0],
    [0, 0, 4, 0, 0, 3],
    [2, 2, 5, 0, 3, 0]
])

INTER_5_CP_TABLE = np.array([
    [0, 1, 0, 5, 4, 4],
    [1, 0, 0, 3, 0, 0],
    [0, 0, 0, 2, 2, 0],
    [5, 3, 2, 0, 2, 0],
    [4, 0, 2, 2, 0, 4],
    [4, 0, 0, 0, 4, 0]
])

ROUTE_CP_TABLE = {
    1: INTER_1_CP_TABLE,
    2: INTER_2_CP_TABLE,
    3: INTER_3_CP_TABLE,
    4: INTER_4_CP_TABLE,
    5: INTER_5_CP_TABLE,
}

def generate_cp_table(inter_id=1):
    INTER_ROUTE_TABLE = ROUTE_TABLE[inter_id]
    N = len(INTER_ROUTE_TABLE)
    
    matrix = np.zeros((N, N), dtype=int) 
    np.fill_diagonal(matrix, 0)

    for i in range(N):
        for j in range(i + 1, N):
            grid_matrix_i = generate_gird_matrix(table=INTER_ROUTE_TABLE, id=i)
            grid_matrix_j = generate_gird_matrix(table=INTER_ROUTE_TABLE, id=j)
            conflict_point = calc_conflict_point(grid_matrix_i, grid_matrix_j)               
            matrix[i][j] = conflict_point
    symmetric_matrix = matrix + matrix.T - np.diag(matrix.diagonal())
    print(symmetric_matrix)

    return
    
def find_route_in_route_table(table, id):
    for key, value in table.items():
        if value == id:
            return key
    return None

def grid_id_to_matrix_coord(grid_id):
        binary_grid_id = format(grid_id - 1, '02b')
        matrix_coord = int(binary_grid_id[0]), int(binary_grid_id[1])        
        return matrix_coord

def generate_gird_matrix(table, id):
    grid_matrix = np.zeros((2,2), dtype=int) 
    route = find_route_in_route_table(table=table, id=id)
    gird_id = local_map_grid_model(route[0], route[1], route[2])
    for grid_id in gird_id:
        matrix_coord = grid_id_to_matrix_coord(grid_id=grid_id)
        grid_matrix[matrix_coord[0], matrix_coord[1]] = 1
    return grid_matrix

def calc_conflict_point(grid_matrix_i, grid_matrix_j):
    grid_matrix = grid_matrix_i + grid_matrix_j
    conflict_point = 0
                 
    for m in range(2):
        for n in range(2):
            if grid_matrix[m][n] == 2:
                if check_any_diagonal_is_2(matrix=grid_matrix):
                    conflict_point = 6
                    # print(grid_matrix)
                else:
                    conflict_point = m * 2 + n + 1

    if np.all(grid_matrix == 1):
        if not check_matrix_conditions(matrix=grid_matrix_i):
            conflict_point = 5

    return conflict_point

def check_matrix_conditions(matrix):
    matrix = np.array(matrix)
    if np.any(np.all(matrix == 1, axis=1)):
        return True
    
    if np.any(np.all(matrix == 1, axis=0)):
        return True
    
def check_any_diagonal_is_2(matrix):
    matrix = np.array(matrix)
    return np.all(np.diag(matrix) == 2) or np.all(np.diag(np.fliplr(matrix)) == 2)


if __name__ == "__main__":
    generate_cp_table(inter_id=5)