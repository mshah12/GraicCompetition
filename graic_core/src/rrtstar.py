from turtle import left, right
import carla
import rospy
import math
import numpy as np
from graic_msgs.msg import LaneList
from collections import deque

"""
    Compute the graph for the next bounding box using RRT* algorithm
    Inputs:
        currState: [Loaction, Rotation, Velocity] the current state of vehicle
        obstacleList: List of obstacles
        waypoint: next waypoint to reach
    Outputs: list of nodes that represent optimal path to waypoint
"""
class RRTStar():
    def __init__(self, currState, obstacleList, waypoint):
        # list of nodes in the graph -- will be in tuple form (x,y)
        # list of edges in the graph -- key will be a tuple of (node1,node2) and the value will be the cost/weight of the edge
        self.nodes = []
        self.edges = {}
        # starting node is the car's current (x,y)
        # goal node is the waypoint's (x,y)
        self.start = (currState[0][0], currState[0][1])
        self.goal = (waypoint[0], waypoint[1])
        # list of obstacles
        self.obstacleList = obstacleList
        # placeholder global variables for later use
        self.leftLaneEq = []
        self.rightLaneEq = []
        self.currLeftEdge = []
        self.currRightEdge = []
        # use ROS subscription to get list of left and right edge nodes
        self.leftLaneNodes = rospy.Subscriber('/carla/%s/left_lane_markers' % 'ego_vehicle', LaneList, self.laneNodesCallback)
        self.rightLaneNodes = rospy.Subscriber('/carla/%s/right_lane_markers' % 'ego_vehicle', LaneList, self.laneNodesCallback)

    """
        Callback function called when rospy subscription is used. Does nothing...
        Inputs: None
        Outputs: None
    """
    def laneNodesCallback(self):
        # DO NOTHING -- NEED AS CALLBACK PARAMETER FOR ROS SUBSCRIPTION
        return

    """
        Update the left lane nodes and right lane nodes to contain the current area's edge points
        Inputs: None
        Outputs: None
    """
    def getLaneEdges(self):
        left_idx = None
        right_idx = None
        # get current x position of car
        curr_x = self.start[0]
        # find current index of left and right edges
        for i in range(len(self.leftLaneNodes)):
            if self.leftLaneNodes[i] == curr_x:
                left_idx = i
        for i in range(len(self.rightLaneNodes)):
            if self.rightLaneNodes[i] == curr_x:
                right_idx = i
        # get list of nodes for left and right edges up to 20 units ahead -- subject to change
        look_ahead = 20
        self.currLeftEdge = self.leftLaneNodes[left_idx : left_idx + look_ahead]
        self.currRightEdge = self.rightLaneNodes[right_idx : right_idx + look_ahead]
        # TODO: REMEMBER PREVIOUS INDEX TO USE FOR FUTURE REFERENCE **


    """
        Convert current left and right edge nodes into a polyfit line of degree 2
        Inputs: None
        Outputs: None
    """
    def calcLaneEdgeEquation(self):
        # split left lane nodes into x and y list
        left_edge_x = self.currLeftEdge[0]
        left_edge_y = self.currLeftEdge[1]
        right_edge_x = self.currRightEdge[0]
        right_edge_y = self.currRightEdge[1]
        # compute polyfit lines
        self.leftLaneEq = np.polyfit(left_edge_x, left_edge_y, 2)
        self.rightLaneEq = np.polyfit(right_edge_x, right_edge_y, 2)

    """
        Determines the distance between two nodes -- helper function
        Inputs: 
            node1: the start node
            node2: the end node
        Outputs: 
            dist: the distance between node1 and node2
    """
    def distance(self, node1, node2):
        dist =  np.linalg.norm(np.array(node1) - np.array(node2))
        return dist

    """
        Determines if a node is valid -- meaning it is not in the radius of an obstacle
        Inputs: 
            node: the node to check
        Outputs: 
            True: if node is valid
            False: if node is within the radius of an obstacle
    """
    def isValidNode(self, node):
        for obs in self.obstacleList:
            if self.isInObstacle(node, obs):
                return False 
        return True 

    # def isValidNode(self, node):
    #     for obs in self.obstacleList:
    #         # TODO: CHECK AND SEE IF IT IS EVERY PT OR JUST THE BOUNDING BOX OF THE OBSTACLE
    #         # IF BOUNDING BOX, MAKE SURE PT IS NOT INSIDE THE BOX
    #         for vertex in obs.vertices_locations:
    #             obs_x = vertex.vertex_location.x
    #             obs_y = vertex.vertex_location.y
    #             obstacle = (obs_x, obs_y)
    #             if node == obstacle:
    #                 return False
    #     return True

    """
        Returns a list of 4 points representing the top-down view of an object's bounding box 
        Inputs: 
            node: object of interest
        Outputs:
            list: of (x,y) points representing the corners of the bounding box of an object
            in the form [ul, bl, br, ur]
    """     
    def get_box(self, object): 
        # TODO: Figure out how to get a bounding box of an object
        pass 

    """
        Determines if a point lies within an obstacle + some dilation delta
        Inputs: 
            node: node of interest
            obstacle: obstacle of interest
            dilation_radius: margin of error for obstacle collistion detection
        Outputs: 
            True: if node is within an obstacle
            False: if node is not within an obstacle 
    """
    def isInObstacle(self, node, obstacle, delta=5):
        curr_x = node[0]
        curr_y = node[1]

        obs_v = self.get_box(obstacle)
        ul = obs_v[0]
        bl = obs_v[1]
        br = obs_v[2]

        y_upper = ul[1] + delta
        y_lower = bl[1] - delta
        x_left = ul[0] - delta
        x_right = br[0] + delta

        return curr_x > x_left and curr_x < x_right and curr_y > y_lower and curr_y < y_upper

    """
        Determines if 2 lines intersect with each other 
        Inputs: 
            line1: A line represented by [(x1,y1), (x2, y2)]
            line2: A line represented by [(x3,y3), (x4, y4)]
        Outputs: 
            True: if lines intersect
            False: if lines don't intersect
        Source: 
            http://www.jeffreythompson.org/collision-detection/line-line.php
    """
    def linesCollide(self, line1, line2):
        x1, y1 = line1[0]
        x2, y2 = line1[1]
        x3, y3 = line2[0]
        x4, y4 = line2[1]

        a = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
        b = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
        if a >= 0 and a <= 1 and b >= 0 and b <= 1:
            return True
        return False 

    """
        Determines if an edge between two nodes will pass through an obstacle
        Inputs: 
            node1: the start node
            node2: the end node
        Outputs: 
            True: if edge passes through obstacle
            False: if edge doesn't pass through obstacle
        Source:
            http://www.jeffreythompson.org/collision-detection/line-rect.php
    """
    def isThruObstacle(self, node1, node2):
        new_line = [(node1[0], node1[1]), (node2[0], node2[1])]
        for obs in self.obstacleList:
            # get bounding box pts 
            obs_v = self.get_box(obs) 
            # define the corners of the box
            ul = obs_v[0] 
            bl = obs_v[1]
            br = obs_v[2]
            ur = obs_v[3]
            # create the lines for the box
            top_line = [(ul[0], ul[1]),(ur[0], ur[1])] 
            bottom_line = [(bl[0], bl[1]),(br[0], br[1])]
            left_line = [(ul[0], ul[1]), (bl[0], bl[1])]
            right_line = [(ur[0], ur[1]), (br[0], br[1])]
            # do collision calculation with all sides 
            top = self.linesCollide(new_line, top_line) 
            left = self.linesCollide(new_line, left_line)
            bottom = self.linesCollide(new_line, bottom_line)
            right = self.linesCollide(new_line, right_line)
            if top or left or bottom or right:
                return True 
        return False 

    # def isThruObstacle(self, node1, node2):
    #     # create a linear line from node1 to node2
    #     x_points = [node1[0], node2[0]]
    #     y_points = [node1[1], node2[2]]
    #     line = np.polyfit(x_points, y_points, 1)
    #     # check if any obstacles' vertices fall onto the line
    #     for obs in self.obstacleList:
    #         for vertex in obs.vertices_locations:
    #             obs_x = vertex.vertex_location.x
    #             obs_y = vertex.vertex_location.y
    #             line_result = (line[0] * obs_x) + (line[1])
    #             # NOTE: May have to add some "wiggle room" or a "cushion" during this check...
    #             if line_result == obs_y:
    #                 return True
    #     return False

    """
        Determines the nearest neighboring node of a passed in node
        Inputs: 
            node: the node to check
        Outputs: 
            nearest_node: the node that is nearest to the input node
    """
    def nearestNeighbor(self, node):
        # set minimum distance and local variables
        nearest_node = None
        radius = 10
        minDist = float("inf")
        # check the distance between input node and all nodes -- return the nearest node
        for n in self.nodes:
            if node != n:
                currDist = self.distance(node, n)
                if currDist < minDist and currDist <= radius:
                    # if new edge would pass through an obstacle, continue
                    if self.isThruObstacle(node, n):
                        continue
                    else:
                        minDist = currDist
                        nearest_node = n
        return nearest_node


    """
        Converts the waypoint node into a line and returns a series of 10 points along that line
        Inputs: 
            node: the waypoint goal node
        Outputs: 
            goal_nodes: list of 10 nodes along the newly created line
    """
    def findGoalNodes(self, node):
        # init default vars
        left_node = (0,0)
        right_node = (0,0)
        # get the left and right nodes on the left and right edge at the same y-postition of the waypoint
        center_y = node[1]
        for node in self.leftLaneNodes:
            if node[1] == center_y:
                left_node = (node[0], center_y)
        for node in self.rightLaneNodes:
            if node[1] == center_y:
                right_node = (node[0], center_y)
        # create a linear line that passes through the two points computed above
        goal_line = np.polyfit([left_node[0], right_node[0]], [left_node[1], right_node[1]], 1)
        # grab a series of 10 points along the line computed
        goal_nodes = []
        x_diff = abs(right_node[0] - left_node[0])
        step = x_diff / 10
        for i in range(10):
            x_val = left_node[0] + (step * i)
            y_val = (goal_line[0] * x_val) + (goal_line[1])
            goal_nodes.append((x_val, y_val))
        return goal_nodes

    """
        Create a bounding box region where nodes can be randomly generated and connected
        Inputs: None
        Outputs: None
    """
    def calcGraph(self, max_iter):
        # add current nodes to the graph
        self.nodes.append(self.start)
        goal_nodes = self.findGoalNodes(self.goal)
        for node in goal_nodes:
            self.nodes.append(node)
        #
        # NOTE:
        # Ideally, we want to extend the goal node into a line and add all points on the line into the graph's intial nodes as the goal nodes
        # For now, I'm just adding one goal node for simplicity's sake
        #
        # compute the 4 bounding points of the region of free space
        # compute x min and max
        region_x_min = min(self.currLeftEdge, key=lambda x: x[0])
        region_x_max = max(self.currRightEdge, key=lambda x: x[0])
        # compute y min and max
        # generate max_iter number of nodes
        for i in range(max_iter):
            new_node_x = np.random.randint(region_x_min, region_x_max)
            region_y_min = (self.leftLaneEq[0] * (new_node_x ** 2)) + (self.leftLaneEq[1] * new_node_x) + (self.leftLaneEq[2])
            region_y_max = (self.rightLaneEq[0] * (new_node_x ** 2)) + (self.rightLaneEq[1] * new_node_x) + (self.rightLaneEq[2])
            new_node_y = np.random.randint(region_y_min, region_y_max)
            # generate new node and check if valid
            new_node = (new_node_x, new_node_y)
            if self.isValidNode(new_node):
                # if node is valid, connect to the nearest neighbor if applicible
                self.nodes.append(new_node)
                nearest_node = self.nearestNeighbor(new_node)
                if nearest_node is not None:
                    # add edge into graph with weight as the distance between the two nodes
                    self.edges[(new_node, nearest_node)] = self.distance(new_node, nearest_node)

    # https://github.com/dmahugh/dijkstra-algorithm/blob/master/dijkstra_algorithm.py
    def shortestPath(self):
        # initialize local variables
        start_node = self.start
        end_node = self.goal
        unvisited_nodes = self.nodes.copy()  # All nodes are initially unvisited.

        # Create a dictionary of each node's distance from start_node. We will
        # update each node's distance whenever we find a shorter path.
        distance_from_start = {
            node: (0 if node == start_node else float("inf")) for node in self.nodes
        }

        # Initialize previous_node, the dictionary that maps each node to the
        # node it was visited from when the the shortest path to it was found.
        previous_node = {node: None for node in self.nodes}

        while unvisited_nodes:
            # Set current_node to the unvisited node with shortest distance
            # calculated so far.
            current_node = min(
                unvisited_nodes, key=lambda node: distance_from_start[node]
            )
            unvisited_nodes.remove(current_node)

            # If current_node's distance is INFINITY, the remaining unvisited
            # nodes are not connected to start_node, so we're done.
            if distance_from_start[current_node] == float("inf"):
                break

            # For each neighbor of current_node, check whether the total distance
            # to the neighbor via current_node is shorter than the distance we
            # currently have for that node. If it is, update the neighbor's values
            # for distance_from_start and previous_node.
            for neighbor, distance in self.adjacency_list[current_node]:
                new_path = distance_from_start[current_node] + distance
                if new_path < distance_from_start[neighbor]:
                    distance_from_start[neighbor] = new_path
                    previous_node[neighbor] = current_node

            if current_node == end_node:
                break # we've visited the destination node, so we're done

        # To build the path to be returned, we iterate through the nodes from
        # end_node back to start_node. Note the use of a deque, which can
        # appendleft with O(1) performance.
        path = deque()
        current_node = end_node
        while previous_node[current_node] is not None:
            path.appendleft(current_node)
            current_node = previous_node[current_node]
        path.appendleft(start_node)

        return path, distance_from_start[end_node]
            






