# Basic searching algorithms

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node

def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
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
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    steps = 0
    found = False
    path = []

    obstacles = []
    allVertices = []
    heuristic = {}

    # Calculate list of all vertices and all obstacles for checking next move
    for row in range(len(grid)):
        for col in range(len(grid[0])):
            allVertices.append((row, col))
            if grid[row][col] == 1:
                obstacles.append((row, col))


    wavefront = {node: float('inf') for node in allVertices}
    wavefront[tuple(start)] = 0
    visited = []
    revPath = {}

    while wavefront:

        # Get logical next step (minimum value next index)
        u = min(wavefront, key=wavefront.get)            
        visited.append(u)

        # print('\nVISITING',u)

        steps += 1
        if u == tuple(goal):
            found = True
            break
        elif wavefront[u] == float('inf'): # If the next min node to explore is infinity, we are stuck. Break loop
            break

        # Loop through all neighboring cells of u
        neighbors = get_neighbors(u, obstacles, allVertices)
        for neighbor in neighbors:
            # print('checking this neighbor:', neighbor)

            # If the neighbor has been visited, skip it
            if neighbor in visited:
                continue
            
            # If the path to get to this neighbor is less than its previous value, update it (and append to reverse path dict)
            new_weight = wavefront[u] + 1
            if new_weight < wavefront[neighbor]:
                wavefront[neighbor] = new_weight
                revPath[neighbor] = u
                # print('updated weight of', neighbor, 'to', wavefront[neighbor])
        
        # Remove u from list of wavefront nodes
        wavefront.pop(u)

    # We have a dictionary from goal to start, reverse it
    if tuple(goal) in revPath:
        path.append(goal)
        node = tuple(goal)
        while node != tuple(start):
            path.append(list(revPath[node]))
            node = revPath[node]
        path.reverse()

    # print('\npath:', path)
    # print('visited:', visited)

    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
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
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    obstacles = []
    allVertices = []
    heuristic = {}

    # Calculate heuristic for each node, and get list of all vertices and all obstacles for checking later
    for row in range(len(grid)):
        for col in range(len(grid[0])):
            heuristic[(row, col)] = manhattan_dist((row,col), tuple(goal))
            allVertices.append((row, col))
            if grid[row][col] == 1:
                obstacles.append((row, col))


    wavefront = {node: float('inf') for node in allVertices}
    wavefront[tuple(start)] = 0
    visited = []
    revPath = {}

    while wavefront:

        # Get logical next step (minimum value + cost-to-come)
        # NOTE: this can be improved by checking heuristic each step instead of all cells at the beginning: manhattan_dist([cell[0],cell[1]],tuple(goal))
        min_vertices = {}
        for key in wavefront:
            if key in heuristic:
                min_vertices[key] = wavefront[key] + heuristic[key]
            else:
                pass

        u = min(min_vertices, key=min_vertices.get)
        visited.append(u)

        # print('\nVISITING',u)

        steps += 1
        if u == tuple(goal):
            found = True
            break
        elif wavefront[u] == float('inf'): # If the next min node to explore is infinity, we are stuck. Break loop
            break

        # Loop through all neighboring cells of u
        neighbors = get_neighbors(u, obstacles, allVertices)
        for neighbor in neighbors:
            # print('checking this neighbor:', neighbor)

            # If the neighbor has been visited, skip it
            if neighbor in visited:
                continue
            
            # If the path to get to this neighbor is less than its previous value, update it (and append to reverse path dict)
            # new_weight = wavefront[u] + heuristic[neighbor] + 1
            new_weight = wavefront[u] + 1
            if new_weight < wavefront[neighbor]:
                wavefront[neighbor] = new_weight
                revPath[neighbor] = u
                # print('updated weight of', neighbor, 'to', wavefront[neighbor])
        
        # Remove u from list of wavefront nodes
        wavefront.pop(u)

    # We have a dictionary from goal to start, reverse it
    if tuple(goal) in revPath:
        path.append(goal)
        node = tuple(goal)
        while node != tuple(start):
            path.append(list(revPath[node]))
            node = revPath[node]
        path.reverse()


    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps


def manhattan_dist(a, b):
    return sum(abs(val1 - val2) for val1, val2 in zip(a, b))


def get_neighbors(vertex, obstacles, allVertices):
    new_neighbors = []
    new_vertex = ()
    # print('checking neighbors of:', vertex)
    for direction in 'RDLU':
        if direction == 'R':
            new_vertex = (vertex[0], vertex[1]+1)
        elif direction == 'D':
            new_vertex = (vertex[0]+1, vertex[1])
        elif direction == 'L':
            new_vertex = (vertex[0], vertex[1]-1)
        elif direction == 'U':
            new_vertex = (vertex[0]-1, vertex[1])

        if new_vertex in obstacles:
            continue
        if new_vertex in allVertices:
            new_neighbors.append(new_vertex)

    return new_neighbors

# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
