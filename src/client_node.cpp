#include <geometry_msgs/PoseStamped.h>
#include "client.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh_node("~");

    std::string uri, frame_id;
    int mav_id, kill_switch_channel;
    nh_node.param<std::string>("uri", uri, "radio://0/50/2M/E7E7E7E7E7");
    nh_node.param<int>("mav_id", mav_id, 1);
    nh_node.param<std::string>("frame_id", frame_id, "/world");
    nh_node.param<int>("kill_switch_channel", kill_switch_channel, 4);

    Client client(uri, mav_id, frame_id, kill_switch_channel);
    client.run();
}
