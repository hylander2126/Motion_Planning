# Basic searching algorithms

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node


def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = True

    nrows = len(grid)
    ncols = len(grid[0])
    obstacles = []
    allVertices = []

    # Get list of ALL possible spaces AND all obstacles spaces
    for row in range(nrows):
        for col in range(ncols):
            allVertices.append([row,col])
            if grid[row][col] == 1:
                obstacles.append([row,col])


    visited = [start]
    # visited = set(tuple(start))
    tryThese = [start]
    path_dict = {}
    thisVertex = start
    nextVertex = start

    while len(tryThese) > 0:
        steps += 1
        prevVertex = thisVertex

        thisVertex = tryThese.pop(0) # Grab first node added to list, set as current node, and delete it from list
        # visited.append(thisVertex)
        # visited.add(tuple(thisVertex))

        # print('\n\nNOW AT: ', thisVertex, '. Got here from ', prevVertex)

        if thisVertex == goal:
            break

        for direction in 'RDLU': # NOTE: Reverse order, because of pop() from end of list
            if direction == 'U':
                nextVertex = move_up(thisVertex)
            elif direction == 'L':
                nextVertex = move_left(thisVertex)
            elif direction == 'D':
                nextVertex = move_down(thisVertex)
            elif direction == 'R':
                nextVertex = move_right(thisVertex)

            # If nextVertex out of map == a wall (no wrapping), skip to next loop
            if nextVertex not in allVertices:
                # print("DETECTED A WALL while checking ", direction, ' to ', nextVertex)
                continue

            elif nextVertex in obstacles:
                # print("DETECTED AN OBSTACLE while checking ", direction, ' to ', nextVertex)
                continue

            # If next possible vertex NOT visited NOR obstacle, append to end of 'frontier' list
            elif nextVertex not in visited: # and nextVertex not in obstacles: 
                tryThese.append(nextVertex)
                visited.append(nextVertex)
                # print('NextVertex ', nextVertex, ' added to TRYTHESE list\n')
                path_dict[tuple(nextVertex)] = thisVertex # We will have multiple keys with same value -> we can later order them by working backwards


    pathVertex = goal
    while pathVertex != start:
        path.append(list(path_dict[tuple(pathVertex)]))
        pathVertex = list(path_dict[tuple(pathVertex)])

    path.reverse()
    path.append(goal)

    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###

    found = True
    path = []

    nrows = len(grid)
    ncols = len(grid[0])
    obstacles = []
    allVertices = []

    # Get list of ALL possible spaces AND all obstacles spaces
    for row in range(nrows):
        for col in range(ncols):
            allVertices.append([row,col])
            if grid[row][col] == 1:
                obstacles.append([row,col])       

    # print(obstacles)
    # print(allVertices)

    visited = [start]
    tryThese = [start]
    path_dict = {}
    thisVertex = start
    nextVertex = start

    while len(tryThese) > 0:

        # prevVertex = thisVertex

        thisVertex = tryThese.pop() # Remove start from list, assign to current vertex
        # visited.append(nextVertex)
        visited.append(thisVertex)

        # print('\n\nNOW AT: ', thisVertex, '. Got here from ', prevVertex)

        if thisVertex == goal:
            break

        for direction in 'ULDR': # NOTE: Reverse order, because of pop() from end of list
            if direction == 'U':
                nextVertex = move_up(thisVertex)
            elif direction == 'L':
                nextVertex = move_left(thisVertex)
            elif direction == 'D':
                nextVertex = move_down(thisVertex)
            elif direction == 'R':
                nextVertex = move_right(thisVertex)

            # If nextVertex out of map == a wall (no wrapping), skip to next loop
            if nextVertex not in allVertices:
                # print("DETECTED A WALL while checking ", direction, ' to ', nextVertex)
                continue

            elif nextVertex in obstacles:
                # print("DETECTED AN OBSTACLE while checking ", direction, ' to ', nextVertex)
                continue

            # If next possible vertex NOT visited NOR obstacle, append to end of 'frontier' list
            elif nextVertex not in visited: # and nextVertex not in obstacles: 
                tryThese.append(nextVertex)
                # print('NextVertex ', nextVertex, ' added to TRYTHESE list\n')
                path_dict[tuple(nextVertex)] = thisVertex # We will have multiple keys with same value -> we can later order them by working backwards

    # print('REACHED GOAL!\n\n')


    # print(path_dict)

    # Now we have a path from END to START, each key's VALUE points to the reverse order to start
    
    pathVertex = goal
    while pathVertex != start:
        path.append(list(path_dict[tuple(pathVertex)]))
        pathVertex = list(path_dict[tuple(pathVertex)])

    path.reverse()
    path.append(goal)

    steps = len(path)

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps


def move_right(vertex):
    new_vertex = [vertex[0], vertex[1]+1]
    return new_vertex

def move_down(vertex):
    new_vertex = [vertex[0]+1, vertex[1]]
    return new_vertex

def move_left(vertex):
    new_vertex = [vertex[0], vertex[1]-1]
    return new_vertex

def move_up(vertex):
    new_vertex = [vertex[0]-1, vertex[1]]
    return new_vertex


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
