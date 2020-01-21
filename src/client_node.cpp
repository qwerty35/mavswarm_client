#include <geometry_msgs/PoseStamped.h>
#include "client.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;

    std::string uri;
    int mav_id;
    nh.param<std::string>("uri", uri, "radio://0/50/2M/E7E7E7E701");
    nh.param<int>("mav_id", mav_id, 1);

    Client client(nh, uri, mav_id);
    client.run();
}
