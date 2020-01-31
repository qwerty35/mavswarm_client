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

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define ENABLE_SAFELINK 1

class Client {
public:
    Client(const std::string& link_uri, int mav_id, const std::string& frame_id);

    void run();
private:
    int m_mavId;

    ros::NodeHandle m_rosNodeHandle;
    ros::Publisher m_pub_externalPose;
    ros::Publisher m_pub_setpoint;
    ros::Subscriber m_sub_current_state;
    ros::ServiceClient m_arming_client;
    ros::ServiceClient m_set_mode_client;

    mavros_msgs::State m_current_state;
    std::string m_frame_id;

    geometry_msgs::PoseStamped m_setpoint_msgs;
    geometry_msgs::PoseStamped m_extPose_msgs;

    // crazyradio
    Crazyradio* m_radio;
    int m_devId;
    ITransport* m_transport;
    uint8_t m_channel;
    uint64_t m_address;
    Crazyradio::Datarate m_datarate;
    bool m_is_extPose_received;
    int m_broadcast_fail_count;

    void handleData(const uint8_t* data);
    void publishExternalPose(const uint8_t* data);
    void takeoff(const uint8_t* data);
    void mavros_state_callback(const mavros_msgs::State::ConstPtr& msg);
    void quatextract(uint32_t quat, float* q);
};

