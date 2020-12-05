#!/usr/bin/env python
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
        # data
        self.map_data = None
        self.cost_map = None
        self.odom_pos = None
        self.goal_pos = (-6, 3)

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

        # Problem 1: move the robot toward the goal
        # YOUR CODE HERE

        # we are not at the goal
        if abs(self.goal_pos[0] - self.odom_posp[0] > 0.25) or abs(self.goal_pos[1] - self.odom_posp[1] > 0.25):
            #turn towards the goal
            inc_x = self.goal_pos[0] - self.odom_posp[0]
            inc_y = self.goal_pos[1] - self.odom_posp[1]
            angle_to_goal = np.arctan2(inc_y, inc_x)
            # print(angle_to_goal)
            if abs(angle_to_goal - self.odom_posp[2]) > 0.1:
                # print(angle_to_goal)
                if np.sign(angle_to_goal - self.odom_posp[2]) == 1:
                    # print("a")
                    command.angular.z += angle_to_goal / 2
                    command.linear.x = 0.0
                elif np.sign(angle_to_goal - self.odom_posp[2]) == -1:
                    # print("b")
                    command.angular.z += -1 * angle_to_goal / 2
                    command.linear.x = 0.0
            else:
                command.linear.x += 0.2
        # End problem 1

        # currentLaserTheta = minAngle
        # # for each laser scan
        # for i, scan in enumerate(distances):
        #     # for each laser scan, the angle is currentLaserTheta, the index is i, and the distance is scan
        #     # Problem 2: avoid obstacles based on laser scan readings
        #     # TODO YOUR CODE HERE
        #     d = 0.1
        #     # print(d / scan)
        #     object_in_front = currentLaserTheta >= (
        #         maxAngle - 18 * angleIncrement) and currentLaserTheta <= (minAngle + 18 * angleIncrement) and scan <= d
        #     object_on_left = currentLaserTheta >= (
        #         maxAngle - 36 * angleIncrement) and scan <= d
        #     object_on_right = currentLaserTheta <= (
        #         minAngle + 36 * angleIncrement) and scan <= d
        #     # Goal may be a wall or pillar, stop near it
        #     if object_in_front:
        #         command.linear.x -= 3.0 * (d / scan)
        #     if object_on_right:
        #         # turn left, object to the right
        #         command.linear.x -= 0.1 * (d / scan)
        #         command.angular.z += -0.75 * (d / scan)
        #     elif object_on_left:
        #         # turn right, object to the left
        #         command.linear.x -= 0.1 * (d / scan)
        #         command.angular.z += 0.75 * (d / scan)
        #     if abs(self.goal_pos[0] - ODOM[0] <= 0.25) and abs(self.goal_pos[1] - ODOM[1] <= 0.25):
        #         command.linear.x = 0.0
        #         command.linear.y = 0.0
        #         command.linear.z = 0.0
        #         command.angular.x = 0.0
        #         command.angular.y = 0.0
        #         command.angular.z = 0.0
        #     # End problem 2

        #     # After this loop is done, we increment the currentLaserTheta
        #     currentLaserTheta = currentLaserTheta + angleIncrement
        # print(command)
        pub.publish(command)

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

    def map_callback(self, msg):
        print("In Map Callback")
        self.map_data = msg
        shape = (self.map_data.info.width, self.map_data.info.height)
        self.map_array = np.reshape(
            np.array(self.map_data.data),
            shape
        )
        self.map_pub.publish(True)

    def is_at_goal(self):
        # if we are not at the goal
        if abs(self.goal_pos[0] - self.odom_pos[0] > 0.25) or abs(self.goal_pos[1] - self.odom_pos[1] > 0.25):
            return False
        else:
            return True

    def mark_path_to_goal(self):
        if self.odom_pos == None or self.goal_pos == None or self.map_data == None:
            return

        points = self.get_path()
        # convert points back into rviz units
        points = [(self.x_array_to_rviz(a), self.y_array_to_rviz(b))
                  for a, b in points]

        new_marker = Marker()
        # Marker header specifies what (and when) it is drawn relative to
        new_marker.header.frame_id = "map"
        new_marker.header.stamp = rospy.Time.now()
        # uint8 POINTS=8
        new_marker.type = 8
        # Disappear after 1sec. Comment this line out to make them persist indefinitely
        # new_marker.lifetime = rospy.rostime.Duration(1, 0)
        # Set marker visual properties
        new_marker.color.b = 1.0
        new_marker.color.a = 1.0
        new_marker.scale.x = 0.2
        new_marker.scale.y = 0.2
        for (x, y) in points:
            # set current point
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.1
            new_marker.points.append(p)
        self.mark_pub.publish(new_marker)

    def get_path(self):
        pos = self.odom_pos[:2]
        goal = self.goal_pos
        map_grid = self.map_data

        map_w = map_grid.info.width
        map_h = map_grid.info.height
        map_res = map_grid.info.resolution

        # convert current pos into map_grid coordinates
        current_y = self.x_rviz_to_array(pos[0])
        current_x = self.y_rviz_to_array(pos[1])
        # print(current_x, current_y)

        # convert goal pos into map_grid coordinates
        goal_y = self.x_rviz_to_array(goal[0])
        goal_x = self.y_rviz_to_array(goal[1])
        # print(goal_x, goal_y)

        # build cost map (flood fill)
        self.cost_map = np.full(self.map_array.shape, fill_value=np.inf)
        self.flood_fill(current_y, current_x, goal_y, goal_x)

        # get path
        return self.find_path(current_y, current_x, goal_y, goal_x)

    def find_path(self, x_0, y_0, x_1, y_1):
        cost_map = self.cost_map
        img = self.map_array

        # work backwards towards goal
        node = cost_map[x_1][y_1]
        c_x = x_1
        c_y = y_1

        path = [(c_x, c_y)]
        while node:
            # get neighbors
            neighbors = self.get_neighbors(img, c_x, c_y)

            # find the neighbor with the lowest distance value
            # shortest_distance = np.inf
            for neighbor in neighbors:
                n_x = neighbor[0]
                n_y = neighbor[1]

                if cost_map[n_x][n_y] < node and not (n_x, n_y) in path:
                    c_x = n_x
                    c_y = n_y
                    node = cost_map[n_x][n_y]

            path.append((c_x, c_y))

            if node == 0:
                # print("we found the path")
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
        map_img = self.map_array.copy()
        unknown_spaces = np.where(map_img == -1)
        map_img[unknown_spaces] = 100  # turn everything unknown into a wall

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
            neighbors = self.get_neighbors(map_img, cur_x, cur_y)

            # already skipping occupied spaces
            for neighbor in neighbors:
                n_x = neighbor[0]
                n_y = neighbor[1]
                n = np.array([n_x, n_y])
                # make the path distance the priority
                new_p = cur_p + np.linalg.norm(np.array([cur_x, cur_y]) - n)

                if (n_x, n_y) == (final_x, final_y):
                    self.cost_map[n_x][n_y] = new_p
                    # print("found the goal")
                    return

                # if the block is already seen, possibly update priority
                if self.cost_map[n_x][n_y] != np.inf:
                    # print("exists")

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

        # plt.imsave("flood.png", map_obj, cmap='gray', vmin=0, vmax=400)
        print("could not find goal")

    def get_neighbors(self, image, row, column):
        up = 0
        if row != 0:
            up = image[row - 1][column]

        down = 0
        if row != image.shape[0] - 1:
            down = image[row + 1][column]

        left = 0
        if column != 0:
            left = image[row][column - 1]

        right = 0
        if column != image.shape[1] - 1:
            right = image[row][column + 1]
        
        top_left = 0
        if column != image.shape[1] - 1:
            top_left = image[row - 1][column - 1]

        top_right = 0
        if column != image.shape[1] - 1:
            top_right = image[row - 1][column + 1]
        
        bottom_left = 0
        if column != image.shape[1] - 1:
            bottom_left = image[row + 1][column - 1]

        bottom_right = 0
        if column != image.shape[1] - 1:
            bottom_right = image[row + 1][column + 1]

        ret_pixels = []
        if up != 0 or len(self.get_neighbors(image, row - 1, column)) == 8:
            ret_pixels.append((row - 1, column))
        if down != 0 or len(self.get_neighbors(image, row + 1, column)) == 8:
            ret_pixels.append((row + 1, column))
        if left != 0 or len(self.get_neighbors(image, row, column - 1)) == 8:
            ret_pixels.append((row, column - 1))
        if right != 0 or len(self.get_neighbors(image, row, column + 1)) == 8:
            ret_pixels.append((row, column + 1))
        if top_left != 0 or len(self.get_neighbors(image, row - 1, column - 1)) == 8:
            ret_pixels.append((row - 1, column - 1))
        if top_right != 0 or len(self.get_neighbors(image, row - 1, column + 1)) == 8:
            ret_pixels.append((row - 1, column + 1))
        if bottom_left != 0 or len(self.get_neighbors(image, row + 1, column - 1)) == 8:
            ret_pixels.append((row + 1, column - 1))
        if bottom_right != 0 or len(self.get_neighbors(image, row + 1, column + 1)) == 8:
            ret_pixels.append((row + 1, column + 1))

        return ret_pixels


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('part2', log_level=rospy.DEBUG)

    # call our planner
    gp = GlobalPlanner()

    # Main Loop:
    # part 1 and 2 - chart a path and navigate towards the goal
    while not rospy.is_shutdown():
        gp.mark_path_to_goal()

    # Turn control over to ROS
    rospy.spin()
