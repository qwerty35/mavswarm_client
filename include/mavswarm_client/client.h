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
#include <mavros_msgs/OverrideRCIn.h>

#define ENABLE_SAFELINK 1

class Client {
public:
    Client(const std::string& link_uri, int mav_id, const std::string& frame_id, int kill_switch_channel);

    void run();
private:
    int m_mavId;
    std::string m_frame_id;

    ros::NodeHandle m_rosNodeHandle;
    ros::Publisher m_pub_externalPose;
    ros::Publisher m_pub_setpoint;
    ros::Publisher m_pub_emergencyStop;
    ros::Subscriber m_sub_current_state;
    ros::ServiceClient m_arming_client;
    ros::ServiceClient m_set_mode_client;

    mavros_msgs::State m_current_state;
    geometry_msgs::PoseStamped m_msgs_setpoint;
    geometry_msgs::PoseStamped m_msgs_extPose;
    mavros_msgs::OverrideRCIn m_msgs_emergencyStop;

    ros::Time m_last_pub_time;

    // crazyradio
    Crazyradio* m_radio;
    int m_devId;
    int m_kill_switch_channel;
    ITransport* m_transport;
    uint8_t m_channel;
    uint64_t m_address;
    Crazyradio::Datarate m_datarate;
    double m_link_quality;
    double m_link_rate;

    // state

    bool m_is_extPose_received;
    bool m_is_emergency;


    void handleData(const uint8_t* data);
    void publishMsgs(ros::Rate rate_max);

    void receiveExternalPose(const uint8_t* data);
    void takeoff(const uint8_t* data);
    void emergencyStop();

    void mavros_state_callback(const mavros_msgs::State::ConstPtr& msg);
    void quatextract(uint32_t quat, float* q);
};

