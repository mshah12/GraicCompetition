import rospy
import rospkg
import numpy as np
import argparse
import time
import math
from graic_msgs.msg import ObstacleList, ObstacleInfo
from graic_msgs.msg import LocationInfo, WaypointInfo
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleControl
from graic_msgs.msg import LaneList
from graic_msgs.msg import LaneInfo
import carla
from rrtstar import RRTStar

class VehicleDecision():
    def __init__(self):
        self.vehicle_state = 'straight'
        self.lane_state = 0
        self.counter = 0

        self.lane_marker = None
        self.target_x = None
        self.target_y = None
        self.change_lane = False
        self.change_lane_wp_idx = 0
        self.detect_dist = 30
        self.speed = 20

        self.reachEnd = False

        # global vars for RRT algo
        self.shortestPath = []
        self.prevWaypoint = None
        self.rrt_map = None

    def calcRRTStar(self, currState, obstacleList, waypoint, prevWaypoint, lanemarkers):
        i = 0
        # init a new RRT object
        RRTObject = RRTStar(currState, obstacleList, waypoint, prevWaypoint, lanemarkers)
        self.rrt_map = RRTObject.map
        # populate internal variables with left and right lane nodes
        RRTObject.getLaneEdges()
        # calculate and store lane equations internally
        RRTObject.calcLaneEdgeEquation()
        while(1):
            # create RRT Graph with 1000 nodes
            RRTObject.calcGraph(50)
            # get shortest path from start to goal
            self.shortestPath = RRTObject.shortestPath()
            if len(self.shortestPath) > 0:
                break 
            #print(i)
            i+=1
            if i > 50:
                print("Epic fail")
                break 

    def get_map(self):
        return self.rrt_map

    def get_ref_state(self, currState, obstacleList, lane_marker, waypoint):
        """
            Get the reference state for the vehicle according to the current state and result from perception module
            Inputs:
                currState: [Loaction, Rotation, Velocity] the current state of vehicle
                obstacleList: List of obstacles
            Outputs: reference state position and velocity of the vehicle
        """
        # self.reachEnd = waypoint.reachedFinal

        self.lane_marker = lane_marker.lane_markers_center.location[-1]
        self.lane_state = lane_marker.lane_state
        self.target_x = waypoint[0]
        self.target_y = waypoint[1]
        if self.reachEnd:
            return None
        # print("Reach end: ", self.reachEnd)

        curr_x = currState[0][0]
        curr_y = currState[0][1]

        # Check whether any obstacles are in the front of the vehicle
        obs_front = False
        obs_left = False
        obs_right = False
        front_dist = 20
        if obstacleList:
            for obs in obstacleList:
                for vertex in obs.vertices_locations:
                    dy = vertex.vertex_location.y - curr_y
                    dx = vertex.vertex_location.x - curr_x
                    yaw = currState[1][2]
                    rx = np.cos(-yaw) * dx - np.sin(-yaw) * dy
                    ry = np.cos(-yaw) * dy + np.sin(-yaw) * dx

                    psi = np.arctan(ry / rx)
                    if rx > 0:
                        front_dist = np.sqrt(dy * dy + dx * dx)
                        # print("detected object is at {} away and {} radians".format(front_dist, psi))
                        if psi < 0.2 and psi > -0.2:
                            obs_front = True
                        elif psi > 0.2:
                            obs_right = True
                        elif psi < -0.2:
                            obs_left = True

        # prev_vehicle_state = self.vehicle_state
        if self.lane_state == LaneInfo.LEFT_LANE:
            if front_dist <= self.detect_dist and obs_front:
                if not obs_right:
                    self.vehicle_state = "turn-right"
                else:
                    self.vehicle_state = "stop"
            else:
                self.vehicle_state = "straight"

        elif self.lane_state == LaneInfo.RIGHT_LANE:
            if front_dist <= self.detect_dist and obs_front:
                if not obs_left:
                    self.vehicle_state = "turn-left"
                else:
                    self.vehicle_state = "stop"
            else:
                self.vehicle_state = "straight"

        elif self.lane_state == LaneInfo.CENTER_LANE:
            if front_dist > self.detect_dist:
                self.vehicle_state = "straight"
            else:
                if not obs_front:
                    self.vehicle_state = "straight"
                elif not obs_right:
                    self.vehicle_state = "turn-right"
                elif not obs_left:
                    self.vehicle_state = "turn-left"
                else:
                    self.vehicle_state = "stop"

        if self.vehicle_state == "stop":
            self.speed = 5
        else:
            self.speed = 20

        # print(front_dist, self.lane_state, self.vehicle_state, obs_front, obs_left, obs_right)

        while not self.target_x or not self.target_y:
            continue

        distToTargetX = abs(self.target_x - curr_x)
        distToTargetY = abs(self.target_y - curr_y)

        if ((distToTargetX < 5 and distToTargetY < 5)):

            prev_target_x = self.target_x
            prev_target_y = self.target_y

            self.target_x = self.lane_marker.x
            self.target_y = self.lane_marker.y

            target_orientation = np.arctan2(self.target_y - prev_target_y,
                                            self.target_x - prev_target_x)

            if self.vehicle_state == "turn-right":
                # self.change_lane = False
                tmp_x = 6
                tmp_y = 0
                x_offset = np.cos(target_orientation + np.pi /
                                  2) * tmp_x - np.sin(target_orientation +
                                                      np.pi / 2) * tmp_y
                y_offset = np.sin(target_orientation + np.pi /
                                  2) * tmp_x + np.cos(target_orientation +
                                                      np.pi / 2) * tmp_y
                self.target_x = self.target_x + x_offset
                self.target_y = self.target_y + y_offset
            elif self.vehicle_state == "turn-left":
                # self.change_lane = False
                tmp_x = 6
                tmp_y = 0
                x_offset = np.cos(target_orientation - np.pi /
                                  2) * tmp_x - np.sin(target_orientation -
                                                      np.pi / 2) * tmp_y
                y_offset = np.sin(target_orientation - np.pi /
                                  2) * tmp_x + np.cos(target_orientation -
                                                      np.pi / 2) * tmp_y
                self.target_x = self.target_x + x_offset
                self.target_y = self.target_y + y_offset

        else:
            self.counter += 1
        #print(self.target_x, self.target_y, waypoint[0], waypoint[1])
        return [self.target_x, self.target_y, self.speed]


class VehicleController():
    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.acceleration = -20
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = 0
        return newAckermannCmd

    def execute(self, currentPose, targetPose):
        """
            This function takes the current state of the vehicle and
            the target state to compute low-level control input to the vehicle
            Inputs:
                currentPose: ModelState, the current state of vehicle
                targetPose: The desired state of the vehicle
        """

        currentEuler = currentPose[1]
        curr_x = currentPose[0][0]
        curr_y = currentPose[0][1]

        target_x = targetPose[0]
        target_y = targetPose[1]
        target_v = targetPose[2]

        k_s = 0.1
        k_ds = 1
        k_n = 0.1
        k_theta = 1

        # compute errors
        dx = target_x - curr_x
        dy = target_y - curr_y
        xError = (target_x - curr_x) * np.cos(
            currentEuler[2]) + (target_y - curr_y) * np.sin(currentEuler[2])
        yError = -(target_x - curr_x) * np.sin(
            currentEuler[2]) + (target_y - curr_y) * np.cos(currentEuler[2])
        curr_v = np.sqrt(currentPose[2][0]**2 + currentPose[2][1]**2)
        vError = target_v - curr_v

        delta = k_n * yError
        # Checking if the vehicle need to stop
        if target_v > 0:
            v = xError * k_s + vError * k_ds
            #Send computed control input to vehicle
            newAckermannCmd = AckermannDrive()
            newAckermannCmd.speed = v
            newAckermannCmd.steering_angle = delta
            return newAckermannCmd
        else:
            return self.stop()


class Controller(object):
    """docstring for Controller"""
    def __init__(self):
        super(Controller, self).__init__()
        self.decisionModule = VehicleDecision()
        self.controlModule = VehicleController()
        self.map = None 
        self.nextWaypoint = None 
        self.curr_path = []
        self.next_ref_state = None 
        self.prevWaypoint = None 
        self.nextWaypoint = None 

    """
        Determines the distance between two nodes -- helper function
        Inputs: 
            node1: the start node
            node2: the end node
        Outputs: 
            dist: the distance between node1 and node2
    """
    def distance(self, node1, node2):
        x1, y1 = float(node1[0]), float(node1[1])
        x2, y2 = float(node2[0]), float(node2[1])
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

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

    def get_obstacle_box(self, obs):
        v = [] 
        for vertex in obs.vertices_locations:
                obs_x = vertex.vertex_location.x
                obs_y = vertex.vertex_location.y
                v.append((obs_x, obs_y))
        return [v[0], v[2], v[6], v[4]] 

    def isInObstacle(self, node, obstacle):
        obs_v = self.get_obstacle_box(obstacle)
        return self.isInBox(node, obs_v)

    def stop(self):
        return self.controlModule.stop()
        
    def get_waypoint_box(self, waypoint):
        location = carla.Location(waypoint[0], waypoint[1], waypoint[2])
        rotation = self.map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving).transform.rotation
        # print(rotation)
        box = carla.BoundingBox(location, carla.Vector3D(0.5, 6, 3))
        v = [(n.x, n.y) for n in box.get_local_vertices()]
        return [v[0], v[2], v[6], v[4]] 
     
    def execute(self, currState, obstacleList, lane_marker, waypoint): 
        self.prevWaypoint = (currState[0][0], currState[0][1])
        # only update if we passed it
        if self.nextWaypoint != waypoint:
            if self.nextWaypoint:
                print("reached: ", (self.nextWaypoint.location.x, self.nextWaypoint.location.y))
            self.nextWaypoint = waypoint
            self.decisionModule.calcRRTStar(currState, obstacleList, self.nextWaypoint, self.nextWaypoint, lane_marker)



    # def execute(self, currState, obstacleList, lane_marker, waypoint):
    #     # needs graph recomputation
    #     if self.nextWaypoint != waypoint:
    #         self.nextWaypoint = waypoint
    #         self.decisionModule.calcRRTStar(currState, obstacleList, waypoint, waypoint, lane_marker)
    #         if not self.map:
    #             self.map = self.decisionModule.get_map()
    #         self.curr_path = self.decisionModule.shortestPath[0]
    #         self.next_ref_state = self.curr_path.popleft()

    #     target_point_box = self.get_waypoint_box((self.next_ref_state[0],self.next_ref_state[1],3))

    #     waypoint_location = carla.Location(self.next_ref_state[0],self.next_ref_state[1],3)
    #     waypoint_rotation = self.map.get_waypoint(waypoint_location, project_to_road=True, lane_type=carla.LaneType.Driving).transform.rotation
    #     waypoint_box = carla.BoundingBox(waypoint_location, carla.Vector3D(1.5, 6, 3))

    #     # get car's world point 
    #     cars_location = carla.Location(currState[0][0],currState[0][1], 3)
    #     cars_rotation = self.map.get_waypoint(cars_location, project_to_road=True, lane_type=carla.LaneType.Driving).transform.rotation
    #     cars_transformation = carla.Transform(cars_location, cars_rotation)
    #     cars_transformation.transform(cars_location)
    #     crossed = waypoint_box.contains(cars_location, carla.Transform(waypoint_location, waypoint_rotation))
    #     print(waypoint, self.next_ref_state, self.curr_path, crossed)
    #     if crossed and len(self.curr_path) >= 1:
    #         self.next_ref_state = self.curr_path.popleft()

    #     refState = self.decisionModule.get_ref_state(currState, obstacleList, lane_marker, self.next_ref_state)

    #     if not refState:
    #         return None
    #     return self.controlModule.execute(currState, refState)
