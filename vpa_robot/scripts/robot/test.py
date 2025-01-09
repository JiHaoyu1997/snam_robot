import numpy as np

matrix = np.zeros((2,2), dtype=int)

num = 3
bin_num = format(num - 1, '02b')
row, col = int(bin_num[0]), int(bin_num[1])
coord = (row, col)

matrix[coord[0], coord[1]] = 1
print(matrix)