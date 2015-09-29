#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<msgs/BoolStamped.h>

using namespace std;
using namespace ros;

string disable_topic;
string odom_topic;
double dist_end;
double dist_begin;
bool disable_safety;
Publisher disabl_pub;

void odom_cb(const nav_msgs::Odometry msg)
{
    double x = msg.pose.pose.position.x;
    ROS_INFO("x = %f", x);
    msgs::BoolStamped disable_msg;
    disable_msg.header.stamp = Time::now();
    disable_msg.data = (x > dist_begin && x < dist_end);
    disabl_pub.publish(disable_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_safety_disabler");
    NodeHandle nh("~");
    nh.param<string>("disable_pub", disable_topic, "/disable_safety");
    nh.param<string>("odom_sub", odom_topic, "/odom");
    nh.param<double>("dist_to_disable_begin", dist_begin, .5);
    nh.param<double>("dist_disable_end", dist_end, 1.);

    disable_safety = false;

    Subscriber odom_sub = nh.subscribe(odom_topic, 1, odom_cb);
    disabl_pub = nh.advertise<msgs::BoolStamped>(disable_topic, 1);

    spin();
    return 0;
}
