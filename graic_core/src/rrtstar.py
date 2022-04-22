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
    def __init__(self, currState, obstacleList, waypoint, prevWaypoint, lane_info):
        # list of nodes in the graph -- will be in tuple form (x,y)
        # list of edges in the graph -- key will be a tuple of (node1,node2) and the value will be the cost/weight of the edge
        self.nodes = set()
        self.edges = dict()
        self.weights = {} 
        self.goal_nodes = set()
        
        # starting node is the car's current (x,y)
        # goal node is the waypoint's (x,y)
        self.start = (int(currState[0][0]), int(currState[0][1]))
        self.goal = waypoint
        
        # list of obstacles
        self.obstacleList = obstacleList
        # placeholder global variables for later use
        self.leftLaneEq = []
        self.rightLaneEq = []
        self.currLeftEdge = []
        self.currRightEdge = []

        self.lane_info = lane_info
        self.leftLaneNodes = lane_info.lane_markers_left
        self.rightLaneNodes = lane_info.lane_markers_right
        

        # waypoint stuff
        self.waypoint = waypoint
        self.prevWaypoint = prevWaypoint

        # rospy.init_node('race_run')
        self.host = rospy.get_param('~host', 'localhost')
        self.port = rospy.get_param('~port', 2000)

        self.client = carla.Client(self.host, self.port)
        self.world = self.client.get_world()
        self.map = self.world.get_map()

    """
        Get indeces of lane markers that correspond to the previous and current waypoint
        Inputs: bounding boxes and transforms
        Outputs: index for next and prev waypoints
    """
    def getLaneIndex(nextboundingbox, nexttransform, prevboundingbox, prevtransform):
        # loop through all center lane markers:
        nextindex = -1
        previndex = -1
        for i in range(len(self.lane_info.lane_markers_center.location)):
            if nextboundingbox.contains(self.lane_info.lane_markers_center.location[i], nexttransform):
                nextindex = i
            if prevboundingbox.contains(self.lane_info.lane_markers_center.location[i], prevtransform):
                previndex = i
            if previndex != -1 and nextindex != -1:
                break

        return nextindex, previndex



    """
        Return a list of intermediate points between current and next waypoint
        Inputs: None
        Outputs: list center intermediate waypoints
    """
    def getIntermediateNodes(self):
        next_location = carla.Location(self.prevWaypoint.location[0],
                                      self.prevWaypoint.location[1],
                                      self.prevWaypoint.location[2])
        next_rotation = self.map.get_waypoint(next_location,
                                            project_to_road=True,
                                            lane_type=carla.LaneType.Driving).transform.rotation

        next_transform = carla.Transform(next_location, next_rotation)
        next_box = carla.BoundingBox(next_location, carla.Vector3D(0, 6, 3))

        prev_location = carla.Location(self.waypoint.location[0],
                                      self.waypoint.location[1],
                                      self.waypoint.location[2])
        prev_rotation = self.map.get_waypoint(prev_location,
                                            project_to_road=True,
                                            lane_type=carla.LaneType.Driving).transform.rotation

        prev_transform = carla.Transform(prev_location, prev_rotation)
        prev_box = carla.BoundingBox(prev_location, carla.Vector3D(0, 6, 3))

        nextindex, previndex = self.getLaneIndex(next_box, next_transform, prev_box, prev_transform)

        every_n_markers = (nextindex - previndex)/10

        ret = self.lane_info.lane_markers_center.location[previndex:nextindex:every_n_markers]

        return ret


    """
        Update the left lane nodes and right lane nodes to contain the current area's edge points
        Inputs: None
        Outputs: None
    """
    def getLaneEdges(self):
        left_idx = 0
        right_idx = 0
        # get current x position of car
        curr_y = self.start[1]
        # find current index of left and right edges
        for i in range(len(self.leftLaneNodes.location)):
            if int(self.leftLaneNodes.location[i].y) == curr_y:
                left_idx = i
        for i in range(len(self.rightLaneNodes.location)):
            if int(self.rightLaneNodes.location[i].y) == curr_y:
                right_idx = i
        # get list of nodes for left and right edges up to 20 units ahead -- subject to change
        look_ahead = 20
        self.currLeftEdge = self.leftLaneNodes.location[left_idx : left_idx + look_ahead]
        self.currRightEdge = self.rightLaneNodes.location[right_idx : right_idx + look_ahead]
        # TODO: REMEMBER PREVIOUS INDEX TO USE FOR FUTURE REFERENCE **


    """
        Convert current left and right edge nodes into a polyfit line of degree 2
        Inputs: None
        Outputs: None
    """
    def calcLaneEdgeEquation(self):
        # split left lane nodes into x and y list
        left_edge_x = [n.x for n in self.leftLaneNodes.location]
        left_edge_y = [n.y for n in self.leftLaneNodes.location]
        right_edge_x = [n.x for n in self.rightLaneNodes.location]
        right_edge_y = [n.y for n in self.rightLaneNodes.location]
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

    """
        Returns a list of 4 points representing the top-down view of an object's bounding box 
        Inputs: 
            node: object of interest
        Outputs:
            list: of (x,y) points representing the corners of the bounding box of an object
            in the form [ul, ur, bl, br]
    """     
    def get_obstacle_box(self, obs):
        v = [] 
        for vertex in obs.vertices_locations:
                obs_x = vertex.vertex_location.x
                obs_y = vertex.vertex_location.y
                v.append((obs_x, obs_y))
        return [v[0], v[2], v[6], v[4]] 

    """
        Returns a list of 4 points representing the top-down view of an object's bounding box 
        Inputs: 
            node: object of interest
        Outputs:
            list: of (x,y) points representing the corners of the bounding box of an object
            in the form [ul, ur, br, bl]
    """     
    def get_waypoint_box(self, waypoint):
        location = carla.Location(waypoint.location.x, waypoint.location.y, waypoint.location.z)
        rotation = self.map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving).transform.rotation
        box = carla.BoundingBox(location, carla.Vector3D(3, 6, 3))
        v = [(n.x, n.y) for n in box.get_local_vertices()]
        return [v[0], v[2], v[6], v[4]] 

    """
        Determines if a point lies within a box or not  
        Inputs: 
            node: object of interest
        Outputs:
            True: if point lies within box
            False: if point is not in box 
        Source:
            https://stackoverflow.com/questions/63527698/determine-if-points-are-within-a-rotated-rectangle-standard-python-2-7-library    
    """    
    def isInBox(self, node, vertices):
        curr_x = node[0]
        curr_y = node[1]
        n = len(vertices)
        def is_on_right_side(x, y, xy0, xy1):
            x0, y0 = xy0
            x1, y1 = xy1 
            a = float(y1 - y0)
            b = float(x0 - x1)
            c = -a*x0 - b*y0
            val = a*x + b*y + c
            return val >= 0
        is_right = [is_on_right_side(curr_x, curr_y, (vertices[i][0], vertices[i][1]), (vertices[(i+1)%n][0],vertices[(i+1)%n][1])) for i in range(n)]
        all_left = not any (is_right)
        all_right = all(is_right) 
        return all_left or all_right 

    """
        Determines if a point lies within an obstacle
        Inputs: 
            node: node of interest
            obstacle: obstacle of interest
        Outputs: 
            True: if node is within an obstacle
            False: if node is not within an obstacle 
    """
    def isInObstacle(self, node, obstacle):
        obs_v = self.get_obstacle_box(obstacle)
        return self.isInBox(node, obs_v)
        
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
            obs_v = self.get_obstacle_box(obs) 
            # define the corners of the box
            ul = obs_v[0] 
            ur = obs_v[1]
            br = obs_v[2]
            bl = obs_v[3]
            # create the lines for the box
            top_line = [(ul[0], ul[1]),(ur[0], ur[1])] 
            bottom_line = [(br[0], br[1]), (bl[0], bl[1])]
            left_line = [(bl[0], bl[1]), (ul[0], ul[1])]
            right_line = [(ur[0], ur[1]), (br[0], br[1])]
            # do collision calculation with all sides 
            top = self.linesCollide(new_line, top_line) 
            left = self.linesCollide(new_line, left_line)
            bottom = self.linesCollide(new_line, bottom_line)
            right = self.linesCollide(new_line, right_line)
            if top or left or bottom or right:
                return True 
        return False 


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
        radius = 3
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
        Converts the waypoint node into a 4 point box
        Inputs: 
            node: the waypoint goal node
        Outputs: 
            goal_region: 4 point box
    """
    def findGoalRegion(self, node):
    	return self.get_waypoint_box(node)


    """
        Create a bounding box region where nodes can be randomly generated and connected
        Inputs: None
        Outputs: None
    """
    def calcGraph(self, max_iter):
        # add current nodes to the graph
        self.nodes.add(self.start)
        self.edges[self.start] = set()
        goal_region = self.findGoalRegion(self.goal)

        # NOTE:
        # Ideally, we want to extend the goal node into a line and add all points on the line into the graph's intial nodes as the goal nodes
        # For now, I'm just adding one goal node for simplicity's sake
        
        x_bound1 = self.goal.location.x 
        x_bound2 = self.start[0] 
        x_min = min(x_bound1, x_bound2)
        x_max = max(x_bound1, x_bound2) 
        
        # compute y min and max
        # generate max_iter number of nodes
        for i in range(max_iter):
            new_node_x = np.random.randint(x_min, x_max)
            region_y_min = (self.leftLaneEq[0] * (new_node_x ** 2)) + (self.leftLaneEq[1] * new_node_x) + (self.leftLaneEq[2])
            region_y_max = (self.rightLaneEq[0] * (new_node_x ** 2)) + (self.rightLaneEq[1] * new_node_x) + (self.rightLaneEq[2])
            new_node_y = np.random.randint(min(region_y_min, region_y_max), max(region_y_min, region_y_max))
            # generate new node and check if valid
            new_node = (new_node_x, new_node_y)
            if self.isValidNode(new_node):
                # if node is valid, connect to the nearest neighbor if applicible
                self.nodes.add(new_node)
                if new_node not in self.edges:
                    self.edges[new_node] = set()
                nearest_node = self.nearestNeighbor(new_node)
                if nearest_node is not None:
                    self.edges[new_node].add(nearest_node)
                    self.weights[(new_node, nearest_node)] = self.distance(new_node, nearest_node)
                if self.isInBox(new_node, goal_region):
                    self.goal_nodes.add(new_node)
        # connect the start node to the rest of the graph 
        nearest_node = self.nearestNeighbor(self.start) 
        self.edges[self.start].add(nearest_node)
        self.weights[(self.start, nearest_node)] = self.distance(self.start, nearest_node)

        # print("Nodes: ", self.nodes)
        #print("---------------------------------------------------------------------------------------------------------") 
        #print("Edges: ", self.edges)
                    

    # https://github.com/dmahugh/dijkstra-algorithm/blob/master/dijkstra_algorithm.py
    def shortestPath(self):
        # initialize local variables
        start_node = self.start
        # end_node = self.goal
        unvisited_nodes = self.nodes.copy()  # All nodes are initially unvisited.
        print("Goal nodes:", self.goal_nodes)
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
            #print(current_node)
            # For each neighbor of current_node, check whether the total distance
            # to the neighbor via current_node is shorter than the distance we
            # currently have for that node. If it is, update the neighbor's values
            # for distance_from_start and previous_node.
            for neighbor in self.edges[current_node]:
                new_path = distance_from_start[current_node] + self.weights[(current_node, neighbor)]
                if new_path < distance_from_start[neighbor]:
                    distance_from_start[neighbor] = new_path
                    previous_node[neighbor] = current_node

            if current_node in self.goal_nodes:
                print("Reached the end")
                end_node = current_node
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
        print(path)
        return path, distance_from_start[end_node]
            






