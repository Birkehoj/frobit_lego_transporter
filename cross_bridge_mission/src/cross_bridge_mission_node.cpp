#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<msgs/BoolStamped.h>
#include<msgs/IntStamped.h>
#include<geometry_msgs/TwistStamped.h>
using namespace std;
using namespace ros;

const double loop_rate = 20.0;

string straight_topic;
string follow_topic;
string odom_topic;
string automode_topic;
string vel_cmd_topic;
string disable_safety_topic;
double dist_end;
double dist_begin;

double x_pos;
bool bridge_start_reached;
bool at_ramp;
bool automode_is_enb;

void odom_cb(const nav_msgs::Odometry msg)
{
    x_pos = msg.pose.pose.position.x;
}

void automode_cb(const msgs::IntStamped msg)
{
    automode_is_enb = msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_safety_disabler");
    NodeHandle nh("~");

    nh.param<string>("automode_sub", automode_topic, "/fmPlan/automode");
    nh.param<string>("odom_sub", odom_topic, "/odom");

    nh.param<string>("straight_drive_pub", straight_topic, "/enable_straigh");
    nh.param<string>("wall_follow_drive_pub", follow_topic, "/enable_wall_follow");
    nh.param<string>("vel_cmd_pub", vel_cmd_topic, "/fmCommand/vel_cmd");
    nh.param<string>("disable_safety_pub", disable_safety_topic, "/disable_safety");
    nh.param<double>("dist_to_nonsafe_begin", dist_begin, .5);
    nh.param<double>("dist_to_nonsafe_end", dist_end, 1.);

    bridge_start_reached = false;
    at_ramp = false;
    automode_is_enb = false;
    Subscriber odom_sub = nh.subscribe(odom_topic, 1, odom_cb);
    Subscriber automode_sub = nh.subscribe(automode_topic, 1, automode_cb);
    Publisher straight_pub = nh.advertise<msgs::BoolStamped>(straight_topic, 1);
    Publisher follow_pub = nh.advertise<msgs::BoolStamped>(follow_topic, 1);
    Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>(vel_cmd_topic,1);
    Publisher disable_safety_pub = nh.advertise<msgs::BoolStamped>(disable_safety_topic,1);
    Rate r(loop_rate);

    while (ok()) {
        msgs::BoolStamped enb_straight_msg;
        msgs::BoolStamped enb_follow_msg;
        if(automode_is_enb) {
            if(x_pos <= dist_end) {
                enb_straight_msg.data = true;
                enb_follow_msg.data = false;
            } else {
                enb_straight_msg.data = false;
                enb_follow_msg.data = true;
            }


        } else {
            // stop
            geometry_msgs::TwistStamped vel_msg; // initilized to zero
            vel_msg.header.stamp = ros::Time::now();
            vel_pub.publish(vel_msg);
            enb_straight_msg.data = false;
            enb_follow_msg.data = false;
        }
        enb_straight_msg.header.stamp = Time::now();
        straight_pub.publish(enb_straight_msg);
        enb_follow_msg.header.stamp = Time::now();
        follow_pub.publish(enb_follow_msg);

        msgs::BoolStamped disable_safety_msg;
        disable_safety_msg.header.stamp = ros::Time::now();
        disable_safety_msg.data = (x_pos >= dist_begin && x_pos <= dist_end);
        disable_safety_pub.publish(disable_safety_msg);

        spinOnce();
        r.sleep();
    }

    return 0;
}
