// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

// Eigen
#include <Eigen/Dense>

// Occupancy grid cell value threshold to be considered static.
static const int STATIC_THRESH = 50;
// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
typedef struct Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} Node;


class CSVReader {
public:
    CSVReader(std::string file, std::string delim=",");
    virtual ~CSVReader();
    // attr
    std::string file_name;
    std::string delimeter;

    // methods
    std::vector<std::vector<double>> get_wpts();
};

class RRT {
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();
private:
    ros::NodeHandle nh_;

    // ros pub/sub
    // TODO: add the publishers and subscribers you need
    ros::Publisher path_pub_;
    ros::Publisher tree_pub_;

    ros::Subscriber pf_sub_;
    ros::Subscriber stat_sub_;
    ros::Subscriber dynamic_sub_;
    ros::Subscriber env_sub_;

    // tf stuff
    tf::TransformListener listener;

    // TODO: create RRT params

    // Map Metadata
    float map_resolution;
    int map_width, map_height;
    float origin_x, origin_y;

    // Stored most recent Occupancy grid layers
    nav_msgs::OccupancyGrid stat;
    nav_msgs::OccupancyGrid env;
    nav_msgs::OccupancyGrid dynamic;

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;
    

    // callbacks
    // where rrt actually happens
    void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void goal_callback(const geometry_msgs::Point::ConstPtr& goal_msg);
    // updates occupancy grid
    void static_callback(const nav_msgs::OccupancyGrid::ConstPtr& static_layer);
    void env_callback(const nav_msgs::OccupancyGrid::ConstPtr& env_layer);
    void dynamic_callback(const nav_msgs::OccupancyGrid::ConstPtr& dynamic_layer);

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);

    // visualization
    void pub_tree(std::vector<Node> &tree);
};

