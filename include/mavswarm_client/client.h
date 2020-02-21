#pragma once
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>

#include <cstring>
#include <sstream>
#include <functional>
#include <cmath>

#include "Crazyradio.h"
#include "Crazyflie.h"
#include "crtp.h"
#include <list>
#include <set>
#include <map>
#include <chrono>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#define ENABLE_SAFELINK 1

class Client {
public:
    Client(const std::string& link_uri, int mav_id, const std::string& frame_id, int kill_switch_channel);

    void run();
private:
    int m_mavId;
    std::string m_frame_id;

    ros::NodeHandle m_rosNodeHandle;
    ros::Publisher m_pub_external_pose;
    ros::Publisher m_pub_setpoint;
    ros::Publisher m_pub_setpoint_raw;
    ros::Publisher m_pub_camera_odom;
    ros::Subscriber m_sub_current_state;
    ros::ServiceClient m_arming_client;
    ros::ServiceClient m_set_mode_client;
    ros::ServiceClient m_emergency_stop_client;
    tf::TransformListener tf_listener;

    mavros_msgs::State m_current_state;
    geometry_msgs::PoseStamped m_msgs_setpoint;
    mavros_msgs::PositionTarget m_msgs_setpoint_raw;
    geometry_msgs::PoseStamped m_msgs_extPose;

    ros::Time m_last_pub_time;
    ros::Time m_traj_start_time;
    ros::Time m_latest_goto_time;

    // trajectory
    std::vector<std::array<uint8_t, 24>> m_traj_rawdata;
    std::vector<Crazyflie::poly4d> m_traj_coef;

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
    uint8_t m_link;

    // state
    bool m_is_extPose_received;
    bool m_startTrajectory;
    bool m_haveTrajectory;

    void handleData(uint8_t* data);
    void publishMsgs(ros::Rate rate_max);

    void receiveExternalPose(const uint8_t* data);
    void writeMemory(const uint8_t* data);
    void startTrajectory(const uint8_t* data);
    void uploadTrajectory(const uint8_t* data);
    void takeoff(const uint8_t* data);
    void goTo(const uint8_t* data);
    void land(const uint8_t* data);
    void emergencyStop();
    void updateSetpoints();

    void mavros_state_callback(const mavros_msgs::State::ConstPtr& msg);
    void quatextract(uint32_t quat, float* q);
    bool is_duplicated_message(const uint8_t* data);
};

