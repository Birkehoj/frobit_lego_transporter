#include<ros/ros.h>
#include<msgs/BoolStamped.h>
#include<geometry_msgs/TwistStamped.h>
using namespace std;

const double loop_rate = 20.0;

bool drive_is_enb;
double drive_speed;
string drive_enb_topic;
string vel_topic;

void drive_enb_cb(const msgs::BoolStamped msg)
{
    drive_is_enb = msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drive_straight");
    ros::NodeHandle nh("~");

    nh.param<double>("drive_speed", drive_speed, 0.4);
    nh.param<string>("straight_drive_enb_sub", drive_enb_topic, "/enable_straigh");

    nh.param<string>("straight_vel_pub", vel_topic, "/vel_cmd");

    ros::Subscriber drive_enb_sub = nh.subscribe(drive_enb_topic, 1, drive_enb_cb);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>(vel_topic, 1);
    ros::Rate r(loop_rate);
    while(ros::ok()) {
        if(drive_is_enb) {
            geometry_msgs::TwistStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.twist.linear.x = drive_speed;
            vel_pub.publish(msg);
        }

       ros::spinOnce();
       r.sleep();
    }

    return 0;
}

