// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// CSV reader class
// Destructor
CSVReader::~CSVReader() {
    ROS_INFO("CSVReader shutting down.");
}
// Constructor
CSVReader::CSVReader(std::string file,
                     std::string delim)
    : file_name(file), delimeter(delim) {
    ROS_INFO("Starting CSVReader");
}
// CSVReader member functions
std::vector<std::vector<double>> CSVReader::get_wpts() {
    // Returns the waypoints stored in the file used to create
    // this instance of CSVReader as a std vector of std arrays.
    // You can change the size of the std arrays to keep more
    // information from the waypoint csv file. The defualt of
    // size 2 only keeps the x and y coordinate of the waypoints.
    ROS_INFO("Reading from: %s", file_name.c_str());
    std::ifstream file(file_name);
    std::vector<std::vector<double>> waypoints;
    std::string line = "";
    std::string::size_type sz;
    while(getline(file, line)) {
        std::vector<std::string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
        // change below if you need more information from the file
        double x = std::stod(vec[0], &sz);
        double y = std::stod(vec[1], &sz);
        std::vector<double> point {x, y};
        waypoints.push_back(point);
    }
    file.close();
    ROS_INFO("Waypoints loaded, length: %zd", waypoints.size());
    return waypoints;
}

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) {

    // TODO: Load parameters from yaml file
    std::string path_topic, tree_topic, pf_topic, static_topic, env_topic, dynamic_topic;
    nh_.getParam("path_topic", path_topic);
    nh_.getParam("tree_topic", tree_topic);
    nh_.getParam("pf_pose_topic", pf_topic);
    nh_.getParam("static_layer_topic", static_topic);
    nh_.getParam("env_layer_topic", env_topic);
    nh_.getParam("dynamic_layer_topic", dynamic_topic);
    

    // ROS publishers
    // TODO: create publishers for the the drive topic
    
    
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic, 10);
    tree_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(tree_topic, 10);

    // ROS subscribers
    // subscribe to particle filter
    pf_sub_ = nh_.subscribe(pf_topic, 10, &RRT::pf_callback, this);
    // subscribe to occupancy grid layers
    stat_sub_ = nh_.subscribe(static_topic, 10, &RRT::static_callback, this);
    env_sub_ = nh_.subscribe(env_topic, 10, &RRT::env_callback, this);
    dynamic_sub_ = nh_.subscribe(dynamic_topic, 10, &RRT::dynamic_callback, this);

    // Wait for map metatdata message to update class member
    boost::shared_ptr<nav_msgs::MapMetaData const> env_metadata_ptr;
    env_metadata_ptr = ros::topic::waitForMessage<nav_msgs::MapMetaData>("/map_metadata");
    if (env_metadata_ptr != NULL) {
        map_resolution = env_metadata_ptr->resolution;
        map_width = env_metadata_ptr->width;
        map_height = env_metadata_ptr->height;
        origin_x = env_metadata_ptr->origin.position.x;
        origin_y = env_metadata_ptr->origin.position.y;
        ROS_INFO("Map Metadata Loaded.");
    } else {
        ROS_ERROR("Map Metadata not loaded.");
    }

    // Wait for occupancy grid messages to arrive
    boost::shared_ptr<nav_msgs::OccupancyGrid const> dynamic_ptr;
    boost::shared_ptr<nav_msgs::OccupancyGrid const> static_ptr;
    boost::shared_ptr<nav_msgs::OccupancyGrid const> env_ptr;
    dynamic_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(dynamic_topic);
    static_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(static_topic);
    env_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(env_topic);
    if (dynamic_ptr != NULL && static_ptr != NULL && env_ptr != NULL) {
        static_callback(static_ptr);
        dynamic_callback(dynamic_ptr);
        env_callback(env_ptr);
        ROS_INFO("Occupancy Grid messages arrived.");
    } else {
        ROS_ERROR("Occupancy Grid layers not loaded.");
    }
    ROS_INFO("Created new RRT Object.");
}


void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<Node> tree;
    std::vector<Node> path;

    // TODO: fill in the RRT main loop



    // path found as Path message
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "/map";
    // TODO: fill message with your found path
    
    path_pub_.publish(path_msg);
    
    // visualization
    pub_tree(tree);
}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // TODO: fill in this method
    
    return sampled_point;
}


int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    // TODO: fill in this method

    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering

    Node new_node;
    // TODO: fill in this method

    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TODO: fill in this method

    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method

    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<Node> found_path;
    // TODO: fill in this method

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}





// Occupancy grid callbacks, update class members that stores layer info
// The three layers of Occupancy Grid are stored as: stat, env, and dynamic
void RRT::static_callback(const nav_msgs::OccupancyGrid::ConstPtr &static_layer) {
    // ROS_INFO("updating static layer.");
    nav_msgs::OccupancyGrid new_msg;
    new_msg.header = static_layer->header;
    new_msg.info = static_layer->info;
    new_msg.data = static_layer->data;
    // update class member
    stat = new_msg;
}
void RRT::env_callback(const nav_msgs::OccupancyGrid::ConstPtr &env_layer) {
    // ROS_INFO("updating env layer.");
    nav_msgs::OccupancyGrid new_msg;
    new_msg.header = env_layer->header;
    new_msg.info = env_layer->info;
    new_msg.data = env_layer->data;
    // update class member
    env = new_msg;
}
void RRT::dynamic_callback(const nav_msgs::OccupancyGrid::ConstPtr &dynamic_layer) {
    // ROS_INFO("updating dynamic layer.");
    nav_msgs::OccupancyGrid new_msg;
    new_msg.header = dynamic_layer->header;
    new_msg.info = dynamic_layer->info;
    new_msg.data = dynamic_layer->data;
    // update class member
    dynamic = new_msg;
}
// For visualization
void RRT::pub_tree(std::vector<Node> &tree) {
    // publish the current tree as a float array topic
    // published as [n1.x, n1.y, n1.parent.x, n1.parent.y, ......]
    int tree_length = tree.size();
    std_msgs::Float64MultiArray tree_msg;
    for (int i=1; i<tree_length; i++) {
        double x = tree[i].x, y = tree[i].y;
        double px, py;
        if (tree[i].parent == -1) {
            px = 0.0;
            py = 0.0;
        } else {
            px = tree[tree[i].parent].x, py = tree[tree[i].parent].y;
        }
        tree_msg.data.push_back(x);
        tree_msg.data.push_back(y);
        tree_msg.data.push_back(px);
        tree_msg.data.push_back(py);
    }
    tree_pub_.publish(tree_msg);
}