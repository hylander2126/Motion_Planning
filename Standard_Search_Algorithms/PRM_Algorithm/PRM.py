# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
from numpy.random import default_rng
import networkx as nx
from scipy.spatial import KDTree


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path

        self.tree = []                        # k-d tree populated later
        self.k_sample = 20                    # MAX k-nearest for sample search
        self.k_path = 20                      # MAX k-nearest for path search
        self.rng = default_rng()              # rng "Generator" object


    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''

        in_collision = False

        # Get range of all vertices between two points
        diffX = abs(p1[0] - p2[0])
        diffY = abs(p1[1] - p2[1])

        if diffX > diffY:
            x = np.linspace(p1[0], p2[0], diffX + 1).astype(int)
            y = np.round(np.linspace(p1[1], p2[1], diffX + 1)).astype(int)
        else:
            x = np.round(np.linspace(p1[0], p2[0], diffY + 1)).astype(int)
            y = np.linspace(p1[1], p2[1], diffY + 1).astype(int)

        # Try this
        # x = np.round(np.linspace(p1[0], p2[0], diffY + 1))  # .astype(int)
        # y = np.round(np.linspace(p1[1], p2[1], diffX + 1))  # .astype(int)

        # Check if each vertex between p1 and p2 is occupied
        for i, j in zip(x, y):
            if self.map_array[i, j] == 0:
                in_collision = True

        return in_collision


    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''

        p1_ = np.array(point1)
        p2_ = np.array(point2)

        euc_dist = np.linalg.norm(p1_-p2_).round(decimals=3)

        return euc_dist


    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        # Number of points PER row OR column
        nPts = round(np.sqrt(n_pts))

        # First create list of indices to check for rows and columns
        row_ = np.linspace(0,self.size_row-1,nPts).astype(int)
        col_ = np.linspace(0,self.size_col-1,nPts).astype(int)

        # Loop thru all indices and, if empty, append to list of samples
        for i in row_:
            for j in col_:
                if self.map_array[i,j]:
                    self.samples.append((i, j))

    
    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        samples = set()
        row_ = self.rng.integers(0,self.size_row, n_pts)
        col_ = self.rng.integers(0,self.size_col, n_pts)

        for (i,j) in set(zip(row_,col_)):
            if self.map_array[i][j]:
                self.samples.append((i,j))


    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        # Empty set to avoid duplicates
        samples = set()
        # Scale for gaussian sample
        scale = 12

        for i in range(n_pts):
            # Randomly sample first point
            randX = np.random.randint(0,self.size_row-1)
            randY = np.random.randint(0,self.size_col-1)

            # Gaussian sample second point about first point
            randX2 = round(np.random.normal(randX,scale))
            randY2 = round(np.random.normal(randY,scale))

            # Capture values outside graph range
            randX2 = max(0, min(randX2, self.size_row-1))
            randY2 = max(0, min(randY2, self.size_col-1))

            # Sample more points around obstacles
            if not self.map_array[randX][randY] and self.map_array[randX2][randY2]:
                samples.add((randX2, randY2))
            elif self.map_array[randX][randY] and not self.map_array[randX2][randY2]:
                samples.add((randX, randY))

        self.samples = list(samples)


    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        # Empty set to avoid duplicates
        samples = set()
        # Scale for gaussian sample
        scale = 23

        for i in range(n_pts):
            # Randomly sample first point
            randX = np.random.randint(0, self.size_row - 1)
            randY = np.random.randint(0, self.size_col - 1)

            # Only continue if first point is occupied/obstacle
            if not self.map_array[randX][randY]:
                # Gaussian sample second point about first point
                randX2 = round(np.random.normal(randX, scale))
                randY2 = round(np.random.normal(randY, scale))

                # Capture values outside graph range
                randX2 = max(0, min(randX2, self.size_row - 1))
                randY2 = max(0, min(randY2, self.size_col - 1))

                # Get midpoint between the two points if both points are occupied
                if not self.map_array[randX2][randY2]:
                    x = round((randX + randX2) / 2)
                    y = round((randY + randY2) / 2)

                    if self.map_array[x][y]:
                        samples.add((x, y))

        self.samples = list(samples)


    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict( zip( range( len(self.samples) ), node_pos) )
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])
        
        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y' ,ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12,  node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12,  node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
            self.k_sample = 20
            self.k_path = 80
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)
            self.k_sample = 25
            self.k_path = 100

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), 
        #          (p_id1, p_id2, weight_12) ...]


        # Create k-d tree using scipy for finding nearest neighbors
        self.tree = KDTree(self.samples)

        pairs = self.k_search_sample()
        # print('Sample stage searched for k =', self.k_sample, 'nearest neighbors.')


        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of 
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from([])
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" %(n_nodes, n_edges))


    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), 
        #                (start_id, p_id2, weight_s2) ...]

        start_pairs = set()
        goal_pairs = set()

        # Run k_search to find paths from start and goal
        start_pairs = self.k_search_path('start',start)
        # print('Path (start) stage searched k =', self.k_path, 'nearest neighbors.')
        goal_pairs = self.k_search_path('goal',goal)
        # print('Path (goal) stage searched k =', self.k_path, 'nearest neighbors.')


        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)
        
        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" %path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")
        
        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)


    def k_search_sample(self):
        '''Search nearest neighbors for pairs that meet the specified condition
        returns:
            unique list of edge indices and the weight that connects them
        '''
        # Create empty set to avoid duplicates
        k_pairs = set()

        ## Loop through possible k_nearest neighbors - BREAK when sufficient pairs found
        for k in range(self.k_sample + 1):
            neighbors = list(self.tree.query_pairs(k))
            # Loop through current k-nearest and append to k_pairs if condition met
            for (ii,jj) in neighbors:
                p1 = list(self.samples[ii])
                p2 = list(self.samples[jj])
                in_coll = self.check_collision(p1, p2)
                weight = self.dis(p1, p2)
                # If pair has path that DOES NOT collide AND weight is NOT 0 (a node to itself), append pairs
                if not in_coll and weight != 0:
                    k_pairs.add((ii, jj, weight))

        return list(k_pairs)


    def k_search_path(self, index_id, query_pt=(0)):
        '''Search nearest neighbors for pairs that meet the specified condition
        arguments:
            indices - which indices to compare against each other
            condition - 'string' defining what conditions to meet to append to returned list

        self.tree should be defined. self.k determines how close neighbors are, called before this fn.

        returns:
            unique list of edge indices and the weight that connects them
        '''
        # Create empty set to avoid duplicates
        k_pairs = set()

        ## Loop through possible k_nearest neighbors - BREAK when sufficient pairs found
        for k in range(self.k_path + 1):
            neighbors = list(self.tree.query_ball_point(query_pt, self.k_path))
            # Loop through current k-nearest and append to k_pairs if condition met
            for index in neighbors:
                p1 = list(query_pt)
                p2 = list(self.samples[index])
                in_coll = self.check_collision(p1, p2)
                weight = self.dis(p1, p2)
                # If pair has path that DOES NOT collide AND weight is NOT 0 (a node to itself), append pairs
                if not in_coll and weight != 0:
                    k_pairs.add((index_id, index, weight))

        return list(k_pairs)