#include <ros/ros.h>
#include <sstream>
#include <math.h>
#include <stdlib.h>

// Include messages
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

class Wall {
public:
    float range;
    int index;
    double angle;
    std::string heading;
    Wall();
};

class WallFollower {
    ros::NodeHandle n;
    ros::Subscriber laser_sub;
    ros::Publisher mov_pub;

    int state;
    Wall closest_wall;
    const double max_angle;
    static constexpr float t_dist = 0.5;

    vector<int> regions;
public:
    WallFollower();
    int getState() const;
    void find_closest_wall(const std::vector<float> ranges);
    void move_towards_wall();
    void turn_to_wall();
    void follow_wall();
    void move_away_wall();
    void wall_heading(double precision);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void movement_publisher(double linear, double angular);
};