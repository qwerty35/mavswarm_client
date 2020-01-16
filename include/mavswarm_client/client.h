#pragma once
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <cstring>
#include <sstream>
#include <functional>
#include <math.h>

#include "Crazyradio.h"
#include "crtp.h"
#include <list>
#include <set>
#include <map>
#include <chrono>

#define ENABLE_SAFELINK 1

class Client {
public:
    Client(ros::NodeHandle nh, const std::string& link_uri);

    void listen();
    void publishVisionPose();
private:
    ros::NodeHandle m_rosNodeHandle;
    ros::Publisher m_pub_externalPose;

    Crazyradio* m_radio;
    ITransport* m_transport;
    int m_devId;

    uint8_t m_channel;
    uint64_t m_address;
    Crazyradio::Datarate m_datarate;
    bool m_pingPassed;
    int m_count;

    void handleData(const uint8_t* data);
    void publishExternalPose(const uint8_t* data);
};

