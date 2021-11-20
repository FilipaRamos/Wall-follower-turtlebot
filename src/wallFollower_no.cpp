#include "wallFollower.h"

Wall::Wall(){}

WallFollower::WallFollower() : max_angle(1.82) {
    state = 0;
    regions = {0, 0, 0, 0, 0} // {'right', 'fright', 'front', 'fleft', 'left'}

    laser_sub = n.subscribe("/scan", 1, &WallFollower::laser_callback, this);
    mov_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
}

int WallFollower::getState() const {
    return state;
}

void WallFollower::get_laser_min(std::string region, const std::vector<float> ranges) {
    int start = 0, end = 0;
    if (region == "front")
    float min = std::numeric_limits<float>::max();
    for (int i = 0; i < ranges.size(); i++) {
        if (ranges[i] < min) {
            min = ranges[i];
            index = i;
        }
    }
}

void WallFollower::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    regions = {
        std::min(std::min(msg->ranges[msg->ranges.begin()+]))
    }
}

void WallFollower::movement_publisher(double linear, double angular) {
    geometry_msgs::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;

    mov_pub.publish(msg);
}

/*********************************
              Main
**********************************/
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "follow_wall");


    //WallFollower r(std::atof(argv[0]));
    ROS_INFO_STREAM("Starting the Wall Follower Robot!");
    WallFollower r;
    // Check for robot at initial position. while pose != orig_pose
    ros::Rate rate(1);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}