#include <ros/ros.h>
#include <sstream>
#include <math.h>
#include <stdio.h> 
#include <stdlib.h>
#include <bits/stdc++.h>

// Include messages
#include "nav_msgs/Path.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"

#include "evaluation.h"

class Wall {
public:
    std::vector<float> min_ranges;
    Wall();
};

class WallFollower {
    ros::NodeHandle n;

    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;

    ros::Publisher mov_pub;
    ros::Publisher traj_pub;

    nav_msgs::Path traj_msg;

    int state;
    bool do_eval;
    std::string side;
    Wall closest_wall;
    Evaluation eval;

    const float threshold;
    static constexpr double max_angle = 1.82;
    static constexpr double max_linear = 0.26;
    static constexpr float t_dist = 1.0;
public:
    WallFollower();
    int get_state() const;
    void set_eval(bool do_eval);
    void find_closest_walls(const std::vector<float> ranges);
    float extract_min(int left, int right, const std::vector<float> ranges);
    float min_range(int left, int right, const std::vector<float> ranges);
    float find_min_radius();

    void do_smth();
    void find_wall();
    void turn_left();
    void follow_wall();
    void correct_movement();

    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

    void movement_publisher(double linear, double angular);
    void trajectory_publisher(std_msgs::Header header, geometry_msgs::PoseStamped pose);
};