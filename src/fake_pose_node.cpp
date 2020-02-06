#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "fake_pose_node");
    ros::NodeHandle nh;


    std::string pose_topic_name;
    //nh.param<std::string>("pose_topic_name", pose_topic_name, "/mavros/vision_pose/pose");

    geometry_msgs::PoseStamped fake_pose;

    ros::Publisher pub_vision_pose = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    ros::Publisher pub_setpoint = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);

    ros::Rate rate(40);
    while(ros::ok()){
        fake_pose.header.stamp = ros::Time::now();
        fake_pose.pose.position.z = 0;
        pub_vision_pose.publish(fake_pose);
        fake_pose.pose.position.z = 0.3;
        pub_setpoint.publish(fake_pose);
        ros::spinOnce();
        rate.sleep();
    }
}
