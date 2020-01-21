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
    Client(ros::NodeHandle nh, const std::string& link_uri, int dev_id);

    void run();
private:
    int m_mavId;

    // ros
    ros::NodeHandle m_rosNodeHandle;
    ros::Publisher m_pub_externalPose;
    std::string m_frame_id;

    // crazyradio
    Crazyradio* m_radio;
    int m_devId;
    ITransport* m_transport;
    uint8_t m_channel;
    uint64_t m_address;
    Crazyradio::Datarate m_datarate;
    bool m_isBroadcast;
    int m_broadcast_fail_count;

    void handleData(const uint8_t* data);
    void publishExternalPose(const uint8_t* data);
    void quatextract(uint32_t quat, float* q);
};

