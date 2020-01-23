#include <geometry_msgs/PoseStamped.h>
#include "client.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh_node("~");

    std::string uri, frame_id;
    int mav_id;
    nh_node.param<std::string>("uri", uri, "radio://0/50/2M/E7E7E7E700");
    nh_node.param<int>("mav_id", mav_id, 0);
    nh_node.param<std::string>("frame_id", frame_id, "/world");

    Client client(uri, mav_id, frame_id);
    client.run();
}
