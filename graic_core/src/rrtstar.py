from turtle import left, right
import carla
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import random
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
    def __init__(self, currState, obstacleList, waypoint, prevWaypoint, lane_info, iteration):
        # connect with carla environment
        self.host = rospy.get_param('~host', 'localhost')
        self.port = rospy.get_param('~port', 2000)
        self.client = carla.Client(self.host, self.port)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.iteration = iteration


        # GRAPH REPRESENTATION
        # list of 'nodes' in the graph -- will be in tuple form (x,y)
        # adjencency dictionary of the graph -- key will be a node and value will be a set of nodes it's connected to 
        # list of weights in the graph -- key will be an edge (node1, node2) and the value will be the euclidean distance (weight)
        self.nodes = set()
        self.edges = dict()
        self.weights = {}
        # list of goal nodes to mark the end of dijkstra 
        self.goal_nodes = set()

        self.goal_bb_vertices = None

        # start node 
        # t = self.get_world_from_local((int(currState[0][0]), int(currState[0][1])))
        self.start = (currState[0][0], currState[0][1])
        
        # goal waypoint (i.e. the center of the green bounding box)
        self.goal = waypoint
        self.goal_box, self.goal_location, self.goal_rotation, transform2 = self.get_bounding_box((self.goal.location.x, self.goal.location.y), carla.Vector3D(.3, 6, .3))
        self.world.debug.draw_box(self.goal_box, self.goal_rotation, thickness=0.25, color=carla.Color(
            255, 255, 0, 255), life_time=0)
        self.goal_transform = transform2 #carla.Transform(self.goal_location, self.goal_rotation)
        # self.goal_vertices = self.goal_box.get_world_vertices(transform2)
        self.goal_vertices = self.goal_box.get_local_vertices()

        # for n in self.goal_vertices:
        #     print(n.x, n.y, n.z)
        # flag used to print out triangle sampling 
        self.testflag = False

        # print("Local Goal: ", waypoint.location.x, waypoint.location.y, waypoint.location.z)
        # print("Local Start: ", currState[0][0], currState[0][1])
        # print("Global Goal: ", self.goal_location)
        # print("Global start: ", self.start)
        # translational offset used to draw points in path on the track 
        self.x_off = currState[0][0]
        self.y_off = currState[0][1]
        # print("Offset: ", self.x_off, self.y_off)
        # list of obstacles
        self.obstacleList = obstacleList

    """
        Determines the distance between two nodes
        Inputs: 
            node1: the start node
            node2: the end node
        Outputs: 
            dist: the distance between node1 and node2
    """
    def distance(self, node1, node2):
        dist = np.linalg.norm(np.array(node1) - np.array(node2))
        return dist

    """
        Draw points along the path
        Inputs: 
            path: a list of nodes
        Outputs: 
            none
    """
    def drawPoints(self, path):
        for point in path:
            self.drawPoint(point, False)
            # apply rotation and translation from world -> local to draw
            # location = carla.Location(point[0], point[1], 0.1)

    def drawPoint(self, point, inGoal):
        location_point = carla.Location(point[0], point[1], 0.1)
        # transform.transform(location_point)
        # print(location_point.x, location_point.y)
        if inGoal:
            ccolor = carla.Color(0, 0, 255, 0)
        else:
            ccolor = carla.Color(255, 0, 0, 0)

        self.world.debug.draw_point(location_point, size=0.05, color=ccolor, life_time=0)
    
    def drawLine(self, point1, point2):
        point1_location = carla.Location(point1[0], point1[1], 0)
        point2_location = carla.Location(point2[0], point2[1], 0)
        self.world.debug.draw_line(point1_location, point2_location, thickness=0.1, life_time=0)

    """
        Returns a list of 4 points representing the top-down view of an object's bounding box 
        Inputs: 
            node: object of interest
        Outputs:
            list: of (x,y) points representing the corners of the bounding box of an object in world coordinates 
            in the form [ul, ur, bl, br] in world coordinates
    """
    def get_obstacle_box(self, obs):
        v = []
        for vertex in obs.vertices_locations:
            obs_x = vertex.vertex_location.x
            obs_y = vertex.vertex_location.y
            v.append((obs_x, obs_y))
        v0 = self.get_world_from_local(v[0])
        v2 = self.get_world_from_local(v[2])
        v6 = self.get_world_from_local(v[6])
        v4 = self.get_world_from_local(v[4])
        return [(v0.location.x, v0.location.y), (v2.location.x, v2.location.y),
                (v6.location.x, v6.location.y), (v4.location.x, v4.location.y)]

    """
        Determines if a point lies within an obstacle
        Inputs: 
            node: node of interest in world coordinates
            obstacle: obstacle of interest
        Outputs: 
            True: if node is within an obstacle
            False: if node is not within an obstacle 
    """
    def isInObstacle(self, node, obstacle):
        obs_v = self.get_obstacle_box(obstacle)
        return self.isInBox(node, obs_v)

    """
        Determines if a node is valid -- meaning it is not in an obstacle
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
        is_right = [is_on_right_side(curr_x, curr_y, (vertices[i][0], vertices[i][1]), (
            vertices[(i+1) % n][0], vertices[(i+1) % n][1])) for i in range(n)]
        all_left = not any(is_right)
        all_right = all(is_right)
        return all_left or all_right


    def isInGoal(self, node):
        return self.isInBox(node, self.goal_bb_vertices)

    """
        Determines if 2 lines intersect with each other et_world_vert
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
        a = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / \
            ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
        b = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / \
            ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
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
            top_line = [(ul[0], ul[1]), (ur[0], ur[1])]
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
        minDist = float("inf")
        for n in self.nodes:
            if node != n:
                dist = self.distance(node, n)
                if dist < minDist:
                    minDist = dist
                    nearest_node = n
        return nearest_node

    """
        Gets the world coordinates of a point from local coordinates
        Inputs: 
            node: the waypoint goal node
        Outputs: 
            goal_region: 4 point box
    """
    def get_world_from_local(self, node):
        location = carla.Location(node[0], node[1], 3)
        rotation = self.map.get_waypoint(
            location, project_to_road=True, lane_type=carla.LaneType.Driving).transform.rotation
        transformation = carla.Transform(location, rotation)
        transformation.transform(location)
        return location

    def get_local_from_world(self, node): 
        location = carla.Location(self.x_off - self.start[1], self.y_off + self.start[0], 0.1)
        rotation = carla.Rotation(0.0, 270, 0.0)
        transform = carla.Transform(location, rotation)
        local_point = carla.Location(node[0], node[1], 0.1)
        transform.transform(local_point)
        return local_point

    """
        Gets the bounding box, location, and rotation of a node that extends by vector3d
        Inputs: 
            node: the target node
            vector3d: half widths of the bounding box
        Outputs: 
            bounding box, location, and rotation of a node that extends by vector3d
    """
    def get_bounding_box(self, node, vector3d):
        location = carla.Location(node[0], node[1], 0)
        rotation = self.map.get_waypoint(
            location, project_to_road=True, lane_type=carla.LaneType.Driving).transform.rotation
        waypoint_box = carla.BoundingBox(location, vector3d)

        transform = self.map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving).transform
        return (waypoint_box, location, rotation, transform)

    """
        Takes in a node and waypoint and determines if the local node is in waypoint's box
        Inputs: 
            node: target node 
            goal_point: waypoint
        Outputs: 
            True if target node is in waypoint's goal
    """
    def isInGoalRegion(self, node):
        world_loc = carla.Location(node[0], node[1], .3)
        return self.goal_box.contains(world_loc, self.goal_transform)

    def uniform_triangle(self, u, v):
        s = random.random()
        t = random.random()
        in_triangle = s + t <= 1
        p = s * u + t * v if in_triangle else (1 - s) * u + (1 - t) * v
        return p

    # https://stackoverflow.com/questions/34372480/rotate-point-about-another-point-in-degrees-python
    def rotate(self, origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.

        The angle should be given in radians.
        """
        ox, oy = origin
        px, py = point

        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return qx, qy

    """
        Create a bounding box region where nodes can be randomly generated and connected
        Inputs: None
        Outputs: None
    """
    def calcGraph(self, max_iter):
        self.nodes.add(self.start)
        self.edges[self.start] = set()
        p1 = np.array([self.start[0], self.start[1]])
        p2 = np.array([self.goal_vertices[0].x, self.goal_vertices[0].y])
        p3 = np.array([self.goal_vertices[2].x, self.goal_vertices[2].y])

        bb1 = np.array([self.goal_vertices[4].x, self.goal_vertices[1].y])
        bb4 = np.array([self.goal_vertices[6].x, self.goal_vertices[3].y])

        self.goal_bb_vertices = np.array([bb1, p2, p3, bb4])

        u = p2 - p1
        v = p3 - p1
        goal_node = None
        # print(p1, p2, p3, u, v)
        # print(p1, p2, p3, u, v)
        for _ in range(max_iter):
            p = self.uniform_triangle(u, v)
            p += p1
            new_node = (p[0], p[1])
            self.drawPoint(new_node, False)
            # make sure node isn't in obstacle
            if self.isValidNode(new_node):
                self.nodes.add(new_node)
                if new_node not in self.edges:
                    self.edges[new_node] = set()
                nearest_node = self.nearestNeighbor(new_node)
                # make sure (new_node, nearest_node) doesn't cross an obstacle
                if nearest_node:
                    if not self.isThruObstacle(new_node, nearest_node):
                        self.edges[new_node].add(nearest_node)
                        self.edges[nearest_node].add(new_node)
                        self.drawLine(new_node, nearest_node)
                        self.weights[(new_node, nearest_node)] = self.distance(new_node, nearest_node)
                        self.weights[(nearest_node, new_node)] = self.distance(nearest_node, new_node)
                # add to goal set if new_node is in goal region
                # tmp = self.isInGoalRegion(new_node)
                if self.isInGoal(new_node):
                    print("Generated a goal node")
                    self.goal_nodes.add(new_node)
                    goal_node = new_node
        
        if not self.testflag:
            fig, ax = plt.subplots()
            t = list(self.nodes)
            x = [n[0] for n in t]
            y = [n[1] for n in t]
            ax.scatter(x, y, s=1)
            fig_name = "test" + str(self.iteration) + ".png"
            fig.savefig(fig_name, dpi=200)
            self.testflag = True

    # https://github.com/dmahugh/dijkstra-algorithm/blob/master/dijkstra_algorithm.py
    def shortestPath(self):
        # initialize local variables
        start_node = self.start
        end_node = None
        # All nodes are initially unvisited.
        unvisited_nodes = self.nodes.copy()
        #print("Goal nodes:", self.goal_nodes)
        # Create a dictionary of each node's distance from start_node. We will
        # update each node's distance whenever we find a shorter path.
        distance_from_start = {
            node: (0 if node == start_node else float("inf")) for node in self.nodes
        }

        # Initialize previous_node, the dictionary that maps each node to the
        # node it was visited from when the the shortest path to it was found.
        previous_node = {node: None for node in self.nodes}
        index = -1
        while unvisited_nodes:
            index += 1
            # Set current_node to the unvisited node with shortest distance
            # calculated so far.
            current_node = min(
                unvisited_nodes, key=lambda node: distance_from_start[node]
            )
            unvisited_nodes.remove(current_node)

            # If current_node's distance is INFINITY, the remaining unvisited
            # nodes are not connected to start_node, so we're done.
            # print("distance from start of current ", distance_from_start[current_node])
            print(self.edges[start_node])
            if distance_from_start[current_node] == float("inf"):
                # print("Iteration: ", index)
                break
            # print(current_node)
            # For each neighbor of current_node, check whether the total distance
            # to the neighbor via current_node is shorter than the distance we
            # currently have for that node. If it is, update the neighbor's values
            # for distance_from_start and previous_node.
            for neighbor in self.edges[current_node]:
                new_path = distance_from_start[current_node] + \
                    self.weights[(current_node, neighbor)]
                # print("current_node ", current_node, " neighbor ", neighbor, " weight ", weight)
                if new_path < distance_from_start[neighbor]:
                    distance_from_start[neighbor] = new_path
                    previous_node[neighbor] = current_node
                    # print("test")
            # print("Goal nodes: ", self.goal_nodes, " ", current_node)
            if current_node in self.goal_nodes:
                end_node = current_node
                break  # we've visited the destination node, so we're done
        # for node in self.nodes:
        #    for goal_node in self.goal_nodes:
        #        if goal_node in self.edges[node]:
        #            print("Edge exists from ", node, " to goal node ", goal_node)
        # To build the path to be returned, we iterate through the nodes from
        # end_node back to start_node. Note the use of a deque, which can
        # appendleft with O(1) performance.
        path = deque()
        # if not end_node:
        #     path.appendleft(start_node)
        #     path.append((self.goal.location.x, self.goal.location.y))
        #     self.drawPoints(path)
        #     return path, 1
        current_node = end_node
        while previous_node[current_node] is not None:
            current_node = previous_node[current_node]
        # t = self.get_local_from_world(start_node)
        t = start_node
        # t = (t.x, t.y)
        path.appendleft(t)
        # self.drawPoints(path)
        print(path)
        return path, distance_from_start[end_node]