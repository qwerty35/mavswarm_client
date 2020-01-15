#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "fake_pose_node");
    ros::NodeHandle nh;


    std::string pose_topic_name;
    nh.param<std::string>("pose_topic_name", pose_topic_name, "/vicon/cf1/cf1");

    geometry_msgs::PoseStamped fake_pose;

    ros::Publisher pub_fake_pose = nh.advertise<geometry_msgs::PoseStamped>(pose_topic_name, 1);

    ros::Rate rate(100);
    while(ros::ok()){
        pub_fake_pose.publish(fake_pose);
        ros::spinOnce();
        rate.sleep();
    }

}