#!/usr/bin/env python
from operator import sub
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid

from heapq import heapify
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import tf2_ros
import heapq

# Sensor message types
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Quaternion, Point, PointStamped
from std_msgs.msg import Header
from tf2_geometry_msgs import PoseStamped

# the velocity command message
from geometry_msgs.msg import Twist


class GlobalPlanner(object):
    def __init__(self):
        # block movement
        self.move_block = True
        # block path planning
        self.path_block = False

        # data
        self.map_data = None
        self.cost_map = None
        self.odom_pos = None
        self.goal_pos = (-6, 0)
        # self.goal_pos = (7, 1)

        self.start_pos = None
        self.sub_goal = None

        # subscribers
        self.lidar_sub = rospy.Subscriber(
            '/scan', LaserScan, self.lidar_callback)
        self.map_sub = rospy.Subscriber(
            '/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # publishers
        self.map_pub = rospy.Publisher('/recieved_map', Bool, queue_size=10)
        self.mark_pub = rospy.Publisher('/viz_marker', Marker, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def lidar_callback(self, scan_msg):
        # Let's make a new twist message
        command = Twist()

        # Fill in the fields.  Field values are unspecified
        # until they are actually assigned. The Twist message
        # holds linear and angular velocities.
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0

        # if we are waiting for a path, we publish a hold still command
        if self.move_block:
            self.cmd_pub.publish(command)
            return
        
        # Lidar properties (unpacked for your ease of use)
        # find current laser angle, max scan length, distance array for all scans, and number of laser scans
        maxAngle = scan_msg.angle_max
        minAngle = scan_msg.angle_min
        angleIncrement = scan_msg.angle_increment

        maxScanLength = scan_msg.range_max
        distances = scan_msg.ranges
        numScans = len(distances)
        percent_increment = angleIncrement / maxAngle

        goal = ()
        if self.sub_goal == None:
            goal = self.goal_pos
        else:
            goal = self.sub_goal
        # print("GOAL: {}".format(goal))
        
        # we are not at the goal
        if not self.is_at_location(goal[0], goal[1]):
            #turn towards the goal
            inc_x = goal[0] - self.odom_pos[0]
            inc_y = goal[1] - self.odom_pos[1]
            angle_to_goal = np.arctan2(inc_y, inc_x)
            
            # if we are not looking at the goal, rotate towards it
            if abs(angle_to_goal - self.odom_pos[2]) > 0.2:
                # print(angle_to_goal)
                if np.sign(angle_to_goal - self.odom_pos[2]) == 1:
                    command.angular.z += 1 * abs(angle_to_goal) / 2
                    command.linear.x = 0.1
                elif np.sign(angle_to_goal - self.odom_pos[2]) == -1:
                    command.angular.z += -1 * abs(angle_to_goal) / 2
                    command.linear.x = 0.1
            else: # if we are looking at the goal, begin moving
                command.linear.x += 0.5

        currentLaserTheta = minAngle
        # for each laser scan
        for i, scan in enumerate(distances):
            # for each laser scan, the angle is currentLaserTheta, the index is i, and the distance is scan
            # Problem 2: avoid obstacles based on laser scan readings
            # TODO YOUR CODE HERE
            d = 0.3
            # print(d / scan)
            object_in_front = currentLaserTheta >= (
                maxAngle - 18 * angleIncrement) and currentLaserTheta <= (minAngle + 18 * angleIncrement) and scan <= d
            object_on_left = currentLaserTheta >= (
                maxAngle - 36 * angleIncrement) and scan <= d
            object_on_right = currentLaserTheta <= (
                minAngle + 36 * angleIncrement) and scan <= d
            # Goal may be a wall or pillar, stop near it
            if object_in_front:
                print("front")
                command.linear.x -= 3.0 * (d / scan)
            if object_on_right:
                # turn left, object to the right
                print("right")
                command.linear.x -= 0.1 * (d / scan)
                command.angular.z += -0.5 * (d / scan)
            if object_on_left:
                print("left")
                # turn right, object to the left
                command.linear.x -= 0.1 * (d / scan)
                command.angular.z += 0.5 * (d / scan)
            # End problem 2

            # After this loop is done, we increment the currentLaserTheta
            currentLaserTheta = currentLaserTheta + angleIncrement

        # print(command)
        self.cmd_pub.publish(command)

    def odom_callback(self, msg):
        """
        Subscribes to the odom message, unpacks and transforms the relevent information, and places it in the global variable ODOM
        ODOM is structured as follows:
        ODOM = (x, y, yaw)

        :param: msg: Odometry message
        :returns: None
        """
        position = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        (r, p, yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.odom_pos = (position.x, position.y, yaw)

        if self.start_pos == None:
            self.start_pos = self.odom_pos[:2]
            self.move_block = True

        if self.sub_goal != None:
            # check to see if we are at subgoal
            if self.is_at_location(self.sub_goal[0], self.sub_goal[1]):
                print("at sub_goal")
                # rechart path to goal, find new subgoal
                self.mark_path_to_goal()
        else:
            # check to see if we are at goal
            if self.is_at_location(self.goal_pos[0], self.goal_pos[1]):
                self.move_block = True # stop moving indefinitely
                print("at goal")
            elif self.is_at_location(self.start_pos[0], self.start_pos[1]):
                print("at start")
                # if we are at the start, chart a path to the goal
                self.mark_path_to_goal()

    def map_callback(self, msg):
        self.map_data = msg
        shape = (self.map_data.info.width, self.map_data.info.height)
        self.map_array = np.reshape(
            np.array(self.map_data.data),
            shape
        )
        self.map_pub.publish(True)

    def is_at_location(self, goal_x, goal_y):
        # if we are not at the goal
        if abs(goal_x - self.odom_pos[0]) > 0.25 or abs(goal_y - self.odom_pos[1]) > 0.25:
            return False
        else:
            return True

    def mark_path_to_goal(self):
        if self.odom_pos == None or self.goal_pos == None or self.map_data == None:
            return

        points = self.get_path()
        points = [(self.x_array_to_rviz(b), self.y_array_to_rviz(a)) for a, b in points]
        points.reverse()
        
        # if the path is short enough
        threshold = 50
        print("points in path:", len(points))
        approx_path_len = len(points)
        if approx_path_len >= threshold:
            sub_goal = approx_path_len // 3 
            self.sub_goal = points[sub_goal]

            print(self.sub_goal)
        else: 
            self.sub_goal = None

        # path_marker = Marker()
        # # Marker header specifies what (and when) it is drawn relative to
        # path_marker.header.frame_id = "map"
        # path_marker.header.stamp = rospy.Time.now()
        # # uint8 POINTS=8
        # path_marker.type = 8
        # # Disappear after 1sec. Comment this line out to make them persist indefinitely
        # # path_marker.lifetime = rospy.rostime.Duration(1, 0)
        # # Set marker visual properties
        # path_marker.color.b = 1.0
        # path_marker.color.a = 1.0
        # path_marker.scale.x = 0.1
        # path_marker.scale.y = 0.1
        # for x, y in points:
        #     # set current point
        #     p = Point()
        #     p.x = x
        #     p.y = y
        #     p.z = 0.1
        #     path_marker.points.append(p)
        # self.mark_pub.publish(path_marker)

        path_marker = Marker()
        # Marker header specifies what (and when) it is drawn relative to
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = rospy.Time.now()
        # uint8 POINTS=8
        path_marker.type = 8
        # Disappear after 1sec. Comment this line out to make them persist indefinitely
        # path_marker.lifetime = rospy.rostime.Duration(1, 0)
        # Set marker visual properties
        path_marker.color.r = 1.0
        path_marker.color.a = 1.0
        path_marker.scale.x = 0.1
        path_marker.scale.y = 0.1
        # set current point
        p = Point()
        if self.sub_goal:
            p.x = self.sub_goal[0]
            p.y = self.sub_goal[1]
        else:
            p.x = self.goal_pos[0]
            p.y = self.goal_pos[1]
        p.z = 0.1
        path_marker.points.append(p)
        self.mark_pub.publish(path_marker)

        self.move_block = False

    def get_path(self):
        print("in get_path")
        pos = self.odom_pos[:2]
        goal = self.goal_pos
        map_grid = self.map_data

        map_w = map_grid.info.width
        map_h = map_grid.info.height
        map_res = map_grid.info.resolution

        # convert current pos into map_grid coordinates
        current_y = self.x_rviz_to_array(pos[0])
        current_x = self.y_rviz_to_array(pos[1])

        # convert goal pos into map_grid coordinates
        goal_y = self.x_rviz_to_array(goal[0])
        goal_x = self.y_rviz_to_array(goal[1])

        # build cost map (flood fill)
        self.cost_map = np.full(self.map_array.shape, fill_value=np.inf)
        if self.flood_fill(current_x, current_y, goal_x, goal_y):
            # get path
            path = self.find_path(current_x, current_y, goal_x, goal_y)
            return path
        else:
            return []
       
    def near_wall(self, img, x, y, w):
        width = w

        start = np.array([x, y])
        start_x = x - width
        start_y = y - width

        end_x = x + width
        end_y = y + width

        for i in range(start_x, end_x):
            for j in range(start_y, end_y):
                curr = np.array([i, j])
                if np.linalg.norm(start - curr) <= width:  # circle radius width
                    if img[i][j] == 100:
                        return True

        # print("not near wall")
        return False

    def find_path(self, x_0, y_0, x_1, y_1):
        print("In find_path")
        cost_map = self.cost_map
        img = np.flipud(self.map_array.copy())

        # work backwards towards goal
        node = cost_map[x_1][y_1] #node = distance
        c_x = x_1
        c_y = y_1

        path = [(c_x, c_y)]
        while node:
            # print("{}, {}".format(c_x, c_y), node)
            # get neighbors
            neighbors = self.get_neighbors(img, c_x, c_y, 8)
            # print(neighbors)

            # find the neighbor with the lowest distance value
            # shortest_distance = np.inf
            for neighbor in neighbors:
                n_x = neighbor[0]
                n_y = neighbor[1]

                if cost_map[n_x][n_y] <= node and not (n_x, n_y) in path:
                    c_x = n_x
                    c_y = n_y
                    node = cost_map[n_x][n_y]

            path.append((c_x, c_y))

            if node == 0:
                print("we found the path")
                path.append((x_0, y_0))
                return path

        print("no path")
        return []

    def x_rviz_to_array(self, x_pos):
        x = round((x_pos - self.map_data.info.origin.position.x) /
                  self.map_data.info.resolution)
        return int(x)

    def y_rviz_to_array(self, y_pos):
        y = round(self.map_data.info.height - ((y_pos -
                                                self.map_data.info.origin.position.y) / self.map_data.info.resolution))
        return int(y)

    def x_array_to_rviz(self, x_pos):
        x = (x_pos * self.map_data.info.resolution) + \
            self.map_data.info.origin.position.x
        return x

    def y_array_to_rviz(self, y_pos):
        y = (-1 * self.map_data.info.resolution * (y_pos -
                                                   self.map_data.info.height)) + self.map_data.info.origin.position.y
        return y

    def flood_fill(self, x, y, final_x, final_y):
        print("In Flood Fill")
        map_img = np.flipud(self.map_array.copy())
        unknown_spaces = np.where(map_img == -1)  # -1 means unknown
        map_img[unknown_spaces] = 100  # turn everything unknown into a wall
        # plt.imshow(map_img, cmap='gray', vmin=-1, vmax=100)
        # plt.show(True)
        # quit()

        nodes = [(0, (x, y))]
        heapify(nodes)

        while nodes:
            # pop the node
            node = heapq.heappop(nodes)
            # get its values
            cur_p = node[0]
            cur_x, cur_y = node[1]

            # update the map object
            self.cost_map[cur_x][cur_y] = cur_p
            
            # get neighbors
            neighbors = self.get_neighbors(map_img, cur_x, cur_y, 5)
            
            # already skipping occupied spaces
            for neighbor in neighbors:
                n_x = neighbor[0]
                n_y = neighbor[1]
                n = np.array([n_x, n_y])

                # make the path distance the priority
                new_p = (cur_p + np.linalg.norm(np.array([cur_x, cur_y]) - n))

                if (n_x, n_y) == (final_x, final_y):
                    self.cost_map[n_x][n_y] = new_p
                    print("found the goal")
                    return True

                # if self.near_wall(map_img, n_x, n_y):
                #     new_p *= new_p  # square the distance to disuade path's near walls

                # if the block is already seen, possibly update priority
                if self.cost_map[n_x][n_y] != np.inf:
                    # get the p value that matches the node in the heap
                    old_p = [a for a, b in nodes if (n_x, n_y) == b]
                    if len(old_p) > 1:
                        print("Error - quiting")
                        quit()

                    if new_p < old_p:  # if new distance is shorter than before, update it
                        idx = nodes.index((old_p[0], (n_x, n_y)))
                        nodes[idx] = (new_p, (n_x, n_y))
                        heapify(nodes)  # rebalance the heap
                else:
                    self.cost_map[n_x][n_y] = new_p
                    heapq.heappush(nodes, (new_p, (n_x, n_y)))

            # plt.imsave("flood.png", self.cost_map, cmap='gray', vmin=0, vmax=300)
            # plt.show()
        print("could not find goal")
        return False

    def is_wall(self, image, row, column):
        if image[row][column] == 100:
            return True
        else:
            return False

    def get_neighbors(self, image, row, column, w):
        ret_pixels = []
        #get all immediate neighbors not near a wall (include diagnols)

        if not self.near_wall(image, row - 1, column, w):
            ret_pixels.append((row - 1, column))

        if not self.near_wall(image, row + 1, column, w):
            ret_pixels.append((row + 1, column))

        if not self.near_wall(image, row, column - 1, w):
            ret_pixels.append((row, column - 1))

        if not self.near_wall(image, row, column + 1, w):
            ret_pixels.append((row, column + 1))

        if not self.near_wall(image, row - 1, column - 1, w):
            ret_pixels.append((row - 1, column - 1))

        if not self.near_wall(image, row - 1, column + 1, w):
            ret_pixels.append((row - 1, column + 1))

        if not self.near_wall(image, row + 1, column - 1, w):
            ret_pixels.append((row + 1, column - 1))

        if not self.near_wall(image, row + 1, column + 1, w):
            ret_pixels.append((row + 1, column + 1))

        return ret_pixels


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('part2', log_level=rospy.DEBUG)

    # call our planner
    gp = GlobalPlanner()

    # Main Loop:
    
    # Turn control over to ROS
    rospy.spin()
