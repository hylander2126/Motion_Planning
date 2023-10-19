# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import KDTree


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag

        self.explored_nodes = {start}              # List of randomly explored (visited) nodes
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''

        if isinstance(node1, Node):
            p1_ = np.array((node1.row, node1.col))
            p2_ = np.array((node2.row, node2.col))
        else:
            p1_ = np.array((node1[0], node1[1]))
            p2_ = np.array((node2[0], node2[1]))

        euc_dist = np.linalg.norm(p1_ - p2_).round(decimals=3)

        return euc_dist


    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        in_collision = False

        # Get range of all vertices between two points
        diffX = abs(node1.row - node2.row)
        diffY = abs(node1.col - node2.col)

        if diffX > diffY:
            x = np.linspace(node1.row, node2.row, diffX + 1).astype(int)
            y = np.round(np.linspace(node1.col, node2.col, diffX + 1)).astype(int)
        else:
            x = np.round(np.linspace(node1.row, node2.row, diffY + 1)).astype(int)
            y = np.linspace(node1.col, node2.col, diffY + 1).astype(int)

        # Check if each vertex between node1 and node2 is occupied
        for i, j in zip(x, y):
            if self.map_array[i, j] == 0:
                in_collision = True

        return in_collision


    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        # Probability of selecting goal node
        if np.random.rand() < goal_bias:
            rand_coords = (self.goal.row, self.goal.col)
        # Otherwise, randomly select new node
        else:
            while True:
                randX = np.random.randint(0, self.size_row)
                randY = np.random.randint(0, self.size_col)

                if (randX, randY) not in self.explored_nodes:
                    rand_coords = (randX, randY)
                    self.explored_nodes.add(rand_coords)
                    break

        return rand_coords


    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        closest_point = None
        closest_d = math.inf
        # For all vertices in self.vertices
        for vertex in self.vertices:
            distance = np.sqrt((vertex.row - point[0])**2 + (vertex.col - point[1])**2)
            # If this distance is the new smallest, use this point
            if distance < closest_d:
                closest_d = distance
                closest_point = vertex

        return closest_point


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        # Convert 'nodes' to list of tuples of x,y coords
        samples = [(vertex.row, vertex.col) for vertex in self.vertices]
        # Use SciPy KDTree to find nearest neighbors
        tree = KDTree(samples)
        all_neighbors = tree.query_ball_point((new_node.row, new_node.col),neighbor_size)
        # Get list of neighbors (MUST be from self.vertices i.e. pre-existing nodes)
        neighbors = [self.vertices[i] for i in all_neighbors]

        return neighbors


    def extend_node(self, nearest, point, d):
        ## Extend node within distance d of 'nearest' along line from nearest to 'new' point
        # Get slope of line
        slope = np.arctan2(point[1] - nearest.col, point[0] - nearest.row)

        # Get x and y integers within distance 'd'
        x = np.clip(int(np.round(nearest.row + np.cos(slope) * d)), 0, self.size_row-1)
        y = np.clip(int(np.round(nearest.col + np.sin(slope) * d)), 0, self.size_col-1)

        return Node(x, y)


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        # Loop through neighbors
        for neighbor in neighbors:
            neighbor_cost = self.dis(new_node, neighbor) + new_node.cost
            # IF not in collision with new_node AND this cost is cheaper than neighbor cost
            if not self.check_collision(new_node, neighbor) and neighbor_cost < neighbor.cost:
                neighbor.parent = new_node
                neighbor.cost = neighbor_cost
    
    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()


    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        goalBias = 0.05
        d = 10

        for ii in range(n_pts):
            # In each step, get a new point:
            # get its nearest node:
            # extend node, check collision to decide add or drop,
            new_point = self.get_new_point(goalBias)                    # Return tuple
            nearest_node = self.get_nearest_node(new_point)             # Return NODE
            new_node = self.extend_node(nearest_node, new_point, d)     # Return NODE

            # IF extension NOT occupied
            if self.map_array[new_node.row][new_node.col]:
                # IF path between parent and new_node NOT in collision
                if not self.check_collision(new_node, nearest_node):
                    new_node.parent = nearest_node
                    new_node.cost = self.dis(new_node, nearest_node) + nearest_node.cost
                    self.vertices.append(new_node)

                    # Check if in the neighborhood of the goal.
                    dist_to_goal = self.dis(new_node, self.goal)
                    if dist_to_goal < d:
                        self.found = True
                        self.goal.parent = new_node
                        self.goal.cost = dist_to_goal + new_node.cost
                        self.vertices.append(self.goal)
                        break

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        goalBias = 0.05
        d = 10

        for ii in range(n_pts):
            # In each step, get a new point:
            # get its nearest node:
            # extend node, check collision to decide add or drop,
            new_point = self.get_new_point(goalBias)                        # Return tuple
            # First check if this new point is an occupied space
            if self.map_array[new_point[0]][new_point[1]]:
                nearest_node = self.get_nearest_node(new_point)             # Return NODE
                new_node = self.extend_node(nearest_node, new_point, d)     # Return NODE

                # IF extension NOT occupied
                if self.map_array[new_node.row][new_node.col]:
                    # IF path between parent and new_node NOT in collision
                    if not self.check_collision(new_node, nearest_node):
                        # Get neighbors
                        neighbors = self.get_neighbors(new_node, neighbor_size)
                        # Set initial closest node
                        min_cost = self.dis(new_node, nearest_node) + nearest_node.cost
                        closest_node = nearest_node
                        # Loop through neighbors to find nearest node
                        for i, neighbor in enumerate(neighbors):
                            this_cost = self.dis(new_node, neighbor) + neighbor.cost
                            if  this_cost < min_cost:
                                closest_node = neighbor
                                min_cost = this_cost

                        # If line b/w new node and closest/cheapest node is NOT in collision
                        if not self.check_collision(new_node, closest_node):
                            new_node.parent = closest_node
                            new_node.cost = self.dis(new_node, closest_node) + closest_node.cost
                            self.vertices.append(new_node)
                            # Rewire new node and neighbors
                            self.rewire(new_node, neighbors)

                        # Check if in the neighborhood of the goal.
                        dist_to_goal = self.dis(new_node, self.goal)
                        if dist_to_goal < d:
                            self.found = True
                            self.goal.parent = new_node
                            self.goal.cost = dist_to_goal + new_node.cost
                            self.vertices.append(self.goal)

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
