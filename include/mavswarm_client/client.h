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
    Client();

    void run();
private:
    enum Mission{
        LAND,
        NONE,
        TAKEOFF,
        GOTO,
        TRAJECTORY,
    };

    //mav info
    int m_mav_id;
    std::string m_frame_id;
    bool m_mocap_exist;
    bool m_camera_exist;
    bool m_use_vio;
    bool m_vision_pose_updated;

    //ros pub and sub
    ros::NodeHandle m_nh;
    ros::Publisher m_pub_mocap_pose;
    ros::Publisher m_pub_setpoint;
    ros::Publisher m_pub_setpoint_raw;
    ros::Publisher m_pub_camera_odom;
    ros::Publisher m_pub_vision_pose;
    ros::Subscriber m_sub_current_state;
    ros::ServiceClient m_arming_client;
    ros::ServiceClient m_set_mode_client;
    ros::ServiceClient m_emergency_stop_client;
    tf::TransformListener tf_listener;

    //ros msgs
    mavros_msgs::State m_current_state;
    geometry_msgs::PoseStamped m_msgs_setpoint;
    mavros_msgs::PositionTarget m_msgs_setpoint_raw;
    geometry_msgs::PoseStamped m_msgs_mocap_pose; //pose from motion capture system
    geometry_msgs::PoseStamped m_msgs_camera_pose; //pose from depth camera
    geometry_msgs::PoseStamped m_msgs_vision_pose; //if use_vio, it is camera_pose. else it is mocap_pose

    //ros time info
    ros::Time m_last_pub_time;
    ros::Time m_last_command_time;

    // mission
    Mission m_current_mission;
    std::vector<std::array<uint8_t, 24>> m_traj_rawdata;
    std::vector<Crazyflie::poly4d> m_traj_coef;
    geometry_msgs::PoseStamped m_desired_pose; // GOTO, TAKEOFF
    geometry_msgs::PoseStamped m_start_pose; // GOTO, TAKEOFF
    bool m_haveTrajectory;

    // crazyradio
    Crazyradio* m_radio;
    std::string m_mav_uri;
    int m_dev_id;
    uint8_t m_channel;
    uint64_t m_address;
    Crazyradio::Datarate m_datarate;
    double m_link_quality;
    double m_link_rate;
    uint8_t m_link;
    ITransport* m_transport;

    // functions
    void handleData(uint8_t* data);
    void publishMsgs(ros::Rate rate_max);
    void receiveMocapPose(const uint8_t* data);
    void writeMemory(const uint8_t* data);
    void startTrajectory(const uint8_t* data);
    void uploadTrajectory(const uint8_t* data);
    void takeoff(const uint8_t* data);
    void goTo(const uint8_t* data);
    void land(const uint8_t* data);
    void emergencyStop();
    void updateSetpoints();
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
    bool isDuplicatedMessage(const uint8_t* data);
    void initializeSetpoint();
    void quatextract(uint32_t quat, float* q);
    double extractYaw(geometry_msgs::PoseStamped poseStamped);
    geometry_msgs::Quaternion yaw2quat(double yaw);
};

