from scipy.spatial import KDTree
import numpy as np

x = [-5, 1, 4, 6]

range = 5

for i in x:
    print(max(min(i, range-1), 0))

print('\n')
for i in x:
    temp = i
    if i > range - 1:
        temp = range - 1
    if i < 0:
        temp = 0
    print(temp)





