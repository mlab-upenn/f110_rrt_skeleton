"""
ESE 680
RRT assignment
Author: Hongrui Zheng

This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math
from copy import deepcopy

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
from tf import transform_listener

# class def for tree nodes
class Node(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = None # only used in RRT*
        self.is_root = false

# class def for RRT
class RRT(object):
    def __init__(self):
        # get params from yaml file
        # topics, not saved as attributes
        sim_drive_topic = rospy.get_param('sim_drive_topic')
        drive_topic = rospy.get_param('drive_topic')
        pf_topic = rospy.get_param('pf_pose_topic')
        static_topic = rospy.get_param('static_layer_topic')
        dynamic_topic = rospy.get_param('dynamic_layer_topic')
        env_topic = rospy.get_param('env_layer_topic')
        # wpt_topic = rospy.get_param('wayoint_topic')
        path_topic = rospy.get_param('path_topic')
        tree_topic = rospy.get_param('tree_topic')

        use_sim = rospy.get_param('use_sim')
        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # subscribers
        rospy.Subscriber(pf_topic, PoseStamped, self.pf_callback)
        rospy.Subscriber(static_topic, OccupancyGrid, self.static_callback)
        rospy.Subscriber(dynamic_topic, OccupancyGrid, self.dynamic_callback)
        rospy.Subscriber(env_topic, OccupancyGrid, self.env_callback)

        # publishers
        # drive
        if use_sim:
            self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        else:
            self.drive_pub = rospy.Publisher(sim_drive_topic, AckermannDriveStamped, queue_size=10)
        # visualization
        self.tree_pub = rospy.Publisher(tree_topic, Float64MultiArray, queue_size=10)
        # self.wpt_pub = rospy.Publisher(wpt_topic, Point, queue_size=10)
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size=10)

        # class attributes
        # TODO: load list of waypoints from csv
        self.waypoints = None
        # current occupancy grid layers
        self.current_env_layer = None
        self.current_static_layer = None
        self.current_dynamic_layer = None
        # map metadata
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.origin_x = None
        self.origin_y = None

        # wait for all topics to start publishing before starting RRT
        # wait for map metadata
        map_metadata_msg = rospy.wait_for_message('/map_metadata', MapMetaData)
        self.map_resolution = map_metadata_msg.resolution
        self.map_width = map_metadata_msg.width
        self.map_height = map_metadata_msg.height
        self.origin_x = map_metadata_msg.origin.position.x
        self.origin_y = map_metadata_msg.origin.position.y
        rospy.log_info('Map Metadata Loaded')
        dynamic_msg = rospy.wait_for_message(dynamic_topic, OccupancyGrid)
        static_msg = rospy.wait_for_message(static_topic, OccupancyGrid)
        env_msg = rospy.wait_for_message(env_topic, OccupancyGrid)
        self.dynamic_callback(dynamic_msg)
        self.static_callback(static_msg)
        self.env_callback(env_msg)
        rospy.log_info('Occupancy Grid layers loaded.')

    def env_callback(self, env_msg):
        self.current_env_layer = deepcopy(env_msg)

    def static_callback(self, static_msg):
        self.current_static_layer = deepcopy(static_msg)

    def dynamic_callback(self, dynamic_msg):
        self.current_dynamic_layer = deepcopy(dynamic_msg)

    def pf_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        tree = [] # tree as list of Nodes

        path_msg = Path()
        # fill in the path_msg with the path rrt finds
        self.path_pub.publish(path_msg)
        # visualization
        self.pub_tree(tree)
        return None

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        x = None
        y = None
        return (x, y)

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = 0
        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        new_node = None
        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        return path



    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood


    def pub_tree(self, tree):
        """
        This method publishes the tree as a Float64MultiArray message for visualization
        """
        tree_msg = Float64MultiArray()
        for node in tree:
            tree_msg.data.append(node.x)
            tree_msg.data.append(node.y)
            tree_msg.data.append(tree[node.parent].x)
            tree_msg.data.append(tree[node.parent].y)
        self.tree_pub.publish(tree_msg)