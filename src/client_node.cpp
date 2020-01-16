#include <geometry_msgs/PoseStamped.h>
#include "client.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;

    std::string uri;
    nh.param<std::string>("uri", uri, "radio://1/50/2M/E7E7E7E701");

    Client client(nh, uri);

    while(ros::ok()){
        client.listen();
        ros::spinOnce();
    }

}