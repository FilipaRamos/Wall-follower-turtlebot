#include "wallFollower.h"

Wall::Wall(){}

/*
* WallFollower constructor
*
* @attrib state - initialized to 0 to start in state 'find the wall'
*/
WallFollower::WallFollower() : eval(), threshold(0.1) {
    state = 0;
    side = "none";
    // Init subscribers
    laser_sub = n.subscribe("/scan", 1, &WallFollower::laser_callback, this);
    odom_sub = n.subscribe("/odom", 1, &WallFollower::odom_callback, this);
    // Init publishers
    mov_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    traj_pub = n.advertise<nav_msgs::Path>("trajectory", 1000);
}

int WallFollower::get_state() const {
    return state;
}

void WallFollower::set_eval(bool do_eval) {
    do_eval = do_eval;
}

float WallFollower::min_range(int left, int right, const std::vector<float> ranges) {
    float min = std::numeric_limits<float>::max();
    for (; left < right; left++)
        if (ranges[left] < min) min = ranges[left];
    return min;
}

float WallFollower::extract_min(int left, int right, const std::vector<float> ranges) {
    if (left > right) {
        return std::min(min_range(left, left+right, ranges), min_range(0, right, ranges));
    } return min_range(left, right, ranges);
}

/*
* This function finds the closest wall to the robot on the laser scanner
*/
void WallFollower::find_closest_walls(const std::vector<float> ranges) {
    // Calculate nr of bins per area
    int nr_bins_area = ranges.size() / 8;
    std::vector<float> min_distances_per_bin; // front, left, fleft, right, fright

    min_distances_per_bin.push_back(extract_min(ranges.size()-nr_bins_area/2, nr_bins_area/2, ranges));
    min_distances_per_bin.push_back(extract_min(nr_bins_area/2 + nr_bins_area, nr_bins_area/2 + 2*nr_bins_area, ranges));
    min_distances_per_bin.push_back(extract_min(nr_bins_area/2, nr_bins_area/2 + nr_bins_area, ranges));
    min_distances_per_bin.push_back(extract_min(nr_bins_area/2 + 5*nr_bins_area, nr_bins_area/2 + 6*nr_bins_area, ranges));
    min_distances_per_bin.push_back(extract_min(nr_bins_area/2 + 6*nr_bins_area, nr_bins_area/2 + 7*nr_bins_area, ranges));
    closest_wall.min_ranges = min_distances_per_bin;
}

float WallFollower::find_min_radius() {
    auto min = std::numeric_limits<float>::max();
    for (auto dist : closest_wall.min_ranges)
        if (dist < min) min = dist;
    return min;
}

void WallFollower::find_wall() {
    movement_publisher(max_linear, -5*max_linear/find_min_radius());
}

void WallFollower::turn_left() {
    movement_publisher(max_linear, 5*max_linear/find_min_radius());
}

void WallFollower::follow_wall() {
    if (side == "none") movement_publisher(max_linear, 0);
    else if (side == "left") movement_publisher(max_linear, -M_PI/8);
    else movement_publisher(max_linear, M_PI/8);
}

void WallFollower::correct_movement() {
    if (side == "left") movement_publisher(max_linear, -M_PI/2);
    else movement_publisher(max_linear, M_PI/2);
}

bool WallFollower::check_inf(float dist) {
    return dist > 3.5 ? true : false;
}

void WallFollower::do_smth() {
    // front, left, fleft, right, fright
    if (closest_wall.min_ranges[0] > t_dist && closest_wall.min_ranges[2] > t_dist && closest_wall.min_ranges[4] > t_dist) {
        state = 0;
    } else if (closest_wall.min_ranges[0] < t_dist && closest_wall.min_ranges[2] > t_dist && closest_wall.min_ranges[4] > t_dist) {
        state = 1;
    } else if (closest_wall.min_ranges[0] > t_dist && closest_wall.min_ranges[2] > t_dist && closest_wall.min_ranges[4] < t_dist) {
        auto dist = find_min_radius();
        if (dist < t_dist - 0.05) { side = "right"; }
        else if (dist > t_dist + 0.05) { side = "left"; }
        else { side = "none"; }
        state = 2;
    } else if (closest_wall.min_ranges[0] > t_dist && closest_wall.min_ranges[2] < t_dist && closest_wall.min_ranges[4] > t_dist) {
        state = 0;
    } else if (closest_wall.min_ranges[0] < t_dist && closest_wall.min_ranges[2] > t_dist && closest_wall.min_ranges[4] < t_dist) {
        state = 1;
    } else if (closest_wall.min_ranges[0] < t_dist && closest_wall.min_ranges[2] < t_dist && closest_wall.min_ranges[4] > t_dist) {
        state = 1;
    } else if (closest_wall.min_ranges[0] < t_dist && closest_wall.min_ranges[2] < t_dist && closest_wall.min_ranges[4] < t_dist) {
        state = 1;
    } else if (closest_wall.min_ranges[0] > t_dist && closest_wall.min_ranges[2] < t_dist && closest_wall.min_ranges[4] < t_dist) {
        state = 0;
    }
    ROS_INFO_STREAM("[State] Changing to state " << state);
    if (do_eval) eval.eval_iter(find_min_radius());
}

void WallFollower::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Find the closest walls to our robot
    find_closest_walls(msg->ranges);
    do_smth();
}

void WallFollower::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose = msg->pose.pose;

    trajectory_publisher(msg->header, pose_msg);
}

void WallFollower::movement_publisher(double linear, double angular) {
    geometry_msgs::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;

    mov_pub.publish(msg);
}

void WallFollower::trajectory_publisher(std_msgs::Header header, geometry_msgs::PoseStamped pose) {
    traj_msg.header = header;
    traj_msg.poses.push_back(pose);

    traj_pub.publish(traj_msg);
}

/*********************************
              Main
**********************************/
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "follow_wall");

    ROS_INFO_STREAM("Starting up the Wall Follower Robot!");
    WallFollower r;
    r.set_eval(true);

    ros::Rate rate(50);
    while(ros::ok()) {
        int state = r.get_state();
        if (state == 0) r.find_wall();
        else if (state == 1) r.turn_left();
        else if (state == 2) r.follow_wall();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}