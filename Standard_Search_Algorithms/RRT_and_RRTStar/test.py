import numpy as np


def get_new_point(goal_bias):
    '''Choose the goal or generate a random point
    arguments:
        goal_bias - the possibility of choosing the goal instead of a random point

    return:
        point - the new point
    '''
    temp = np.random.rand()
    # print(temp)

    if temp < goal_bias:
        rand_coords = (0,0)

    else:
        while True:
            randX = np.random.randint(0, 300)
            randY = np.random.randint(0, 300)

            # if (randX, randY) not in self.explored_nodes:
            rand_coords = (randX, randY)
            break

    return rand_coords

for i in range(50):
    print(get_new_point(0.05))

# print(np.random.rand())