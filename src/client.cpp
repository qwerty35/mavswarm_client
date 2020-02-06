#include <mutex>

#include "client.h"
#include "crtp.h"
#include "crtpBootloader.h"
#include "crtpNRF51.h"

#include "Crazyradio.h"
#include "CrazyflieUSB.h"

#include <ros/console.h>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <cmath>
#include <inttypes.h>
#include <iostream>

const static int MAX_RADIOS = 1;

Client::Client(
        const std::string& link_uri,
        int mav_id,
        const std::string& frame_id,
        int kill_switch_channel)
        : m_radio(nullptr)
        , m_transport(nullptr)
        , m_mavId(mav_id)
        , m_devId(0)
        , m_channel(0)
        , m_address(0)
        , m_datarate(Crazyradio::Datarate_250KPS)
        , m_frame_id(frame_id)
        , m_kill_switch_channel(kill_switch_channel)
{
    m_pub_setpoint = m_rosNodeHandle.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    m_pub_externalPose = m_rosNodeHandle.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
    
    m_sub_current_state = m_rosNodeHandle.subscribe<mavros_msgs::State>("mavros/state", 10, &Client::mavros_state_callback, this);
    m_arming_client = m_rosNodeHandle.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    m_set_mode_client = m_rosNodeHandle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    m_emergency_stop_client = m_rosNodeHandle.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/emergency_stop");

    int datarate;
    int channel;
    char datarateType;
    bool success = false;

    success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c/%" SCNx64,
            &m_devId, &channel, &datarate,
            &datarateType, &m_address) == 5;
    if (!success) {
        throw std::runtime_error("Uri is not valid!");
    }

    if (success)
    {
        m_channel = channel;
        if (datarate == 250 && datarateType == 'K') {
            m_datarate = Crazyradio::Datarate_250KPS;
        }
        else if (datarate == 1 && datarateType == 'M') {
            m_datarate = Crazyradio::Datarate_1MPS;
        }
        else if (datarate == 2 && datarateType == 'M') {
            m_datarate = Crazyradio::Datarate_2MPS;
        }

        if (m_devId >= MAX_RADIOS) {
            throw std::runtime_error("This version does not support that many radios. Adjust MAX_RADIOS and recompile!");
        }

        {
            m_radio = new Crazyradio(m_devId);
            m_radio->setAckEnable(true);
            m_radio->setArc(0);
            m_radio->setChannel(m_channel);
            m_radio->setAddress(m_address); // mav address
            m_radio->setAddr2(0xFFE7E7E7E7); // broadcast address
            m_radio->setMode(m_radio->Mode_PRX);
            m_radio->setDatarate(m_datarate);
        }
    }

    m_is_extPose_received = false;
    m_is_emergency = false;

    for(int i = 0; i < 8; i++){
        if(i == m_kill_switch_channel)
            m_msgs_emergencyStop.channels[i] = 2070;
        else
            m_msgs_emergencyStop.channels[i] = 1500;
    }
}

void Client::run() {
    uint32_t length;
    uint8_t data[32];
    ros::Rate rate_max = 80;

    int count_max = 500;
    int trial_count = 0;
    int fail_count = 0;
    ros::Time start_time = ros::Time::now();
    m_last_pub_time = ros::Time::now();    

    ROS_INFO("[MAVSWARM_CLIENT] Start mavswarm_client");
    while(ros::ok()){
        if (!m_radio->receivePacket(data, length)) {
            ROS_WARN("[MAVSWARM_CLIENT] ping failed");
            m_is_extPose_received = false;
            fail_count++;
        }
        else{
            if(!m_is_extPose_received) {
                m_is_extPose_received = true;
            }
            handleData(data);
        }
        trial_count++;

        // link quality and rate
        if(trial_count == count_max){
            m_link_quality = 1 - ((double)fail_count/(double)trial_count);
            m_link_rate = (double)count_max / (ros::Time::now() - start_time).toSec();

            ROS_INFO_STREAM("[MAVSWARM_CLIENT] link_quality: " << m_link_quality << ", link_rate: " << m_link_rate << "Hz");

            trial_count = 0;
            fail_count = 0;
            start_time = ros::Time::now();
        }

        publishMsgs(rate_max);
        ros::spinOnce();
    }
}

void Client::handleData(const uint8_t* data){
    // external pose
    if(crtp(data[0]) == crtp(6, 1) && data[1] == 9){
        receiveExternalPose(data);
        return;
    }
    // emergency stop
    else if(crtp(data[0]) == crtp(6, 1) && data[1] == 3){
        emergencyStop();
        return;
    }

    uint8_t ack[32];
    uint32_t ack_size = 0;
    // ping
    if(crtp(data[0]) == crtp(15, 3)) {
        ack[0] = 0x22; //console ack
        ack_size = 4;

    }
    // log reset
    else if(crtp(data[0]) == crtp(5, 1)){
        ack[0] = 0x51;
        ack[1] = 0x05;
        ack_size = 4;
    }
    // request memory
    else if(crtp(data[0]) == crtp(4, 0)) {
        ack[0] = 0x40;
        ack[1] = 0x01;
        ack_size = 3;
    }
    // High level setpoint - take off
    else if(crtp(data[0]) == crtp(8, 0) && data[1] == 1){
        takeoff(data);
        return; //TODO: ack??
    }
    // High level setpoint - land
    else if(crtp(data[0]) == crtp(8, 0) && data[1] == 2){

    }
    // High level setpoint - stop
    else if(crtp(data[0]) == crtp(8, 0) && data[1] == 3){
        
    }
    // High level setpoint - goto
    else if(crtp(data[0]) == crtp(8, 0) && data[1] == 4){

    }
    // High level setpoint - start trajectory
    else if(crtp(data[0]) == crtp(8, 0) && data[1] == 5){

    }
    else{
        ROS_ERROR("[MAVSWARM_CLIENT] unsupported crtp message");
        return;
    }
    m_radio->sendPacketNoAck((uint8_t*)&ack, ack_size);
}

void Client::receiveExternalPose(const uint8_t* data) {
    auto extPose_crtp = (crtpExternalPosePacked*)data;
    for(int i = 0; i < 2; i++) {
        if (m_mavId == extPose_crtp->poses[i].id) {
            m_msgs_extPose.header.frame_id = m_frame_id;
            m_msgs_extPose.header.stamp = ros::Time::now();

            m_msgs_extPose.pose.position.x = (float)extPose_crtp->poses[i].x/1000;
            m_msgs_extPose.pose.position.y = (float)extPose_crtp->poses[i].y/1000;
            m_msgs_extPose.pose.position.z = (float)extPose_crtp->poses[i].z/1000;

            float q[4];
            quatextract(extPose_crtp->poses[i].quat, q);
            m_msgs_extPose.pose.orientation.x = q[0];
            m_msgs_extPose.pose.orientation.y = q[1];
            m_msgs_extPose.pose.orientation.z = q[2];
            m_msgs_extPose.pose.orientation.w = q[3];
        }
    }
}

void Client::takeoff(const uint8_t* data) {
    auto cmd_takeoff_crtp = (crtpCommanderHighLevelTakeoffRequest*)data;

    if(!m_is_extPose_received){
        ROS_WARN("[MAVSWARM_CLIENT] No external pose, do not takeoff");
        return;
    }
    // initialize setpoint
    m_msgs_setpoint = m_msgs_extPose;
    m_msgs_setpoint.pose.position.z = cmd_takeoff_crtp->height;
    ROS_INFO_STREAM("Initilize setpoint, x: " << m_msgs_setpoint.pose.position.x
                                   << ", y: " << m_msgs_setpoint.pose.position.y
                                   << ", z: " << m_msgs_setpoint.pose.position.z);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if( m_current_state.mode != "OFFBOARD"){
        if( m_set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
            ROS_INFO("[MAVSWARM_CLIENT] Offboard enabled");
        }
        else{
            ROS_ERROR("[MAVSWARM_CLIENT] Offboard failed");
            return;
    	}
    }

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if( !m_current_state.armed ){
        if( m_arming_client.call(arm_cmd) && arm_cmd.response.success){
            ROS_INFO("[MAVSWARM_CLIENT] Vehicle armed");
        }
        else{
            ROS_ERROR("[MAVSWARM_CLIENT] Arming failed");
            return;
        }
    }
}

void Client::emergencyStop() {
    mavros_msgs::CommandBool emergency_stop_cmd;
    emergency_stop_cmd.request.value = false;
    if( m_emergency_stop_client.call(emergency_stop_cmd) && emergency_stop_cmd.response.success){
        ROS_WARN("[MAVSWARM_CLIENT] Emergency stop success");
    }
    else{
        ROS_ERROR("[MAVSWARM_CLIENT] Emergency stop failed");
        return;
    }
}

void Client::mavros_state_callback(const mavros_msgs::State::ConstPtr& msg){
    m_current_state = *msg;
}

void Client::publishMsgs(ros::Rate rate_max) {
    if (ros::Time::now() - m_last_pub_time < rate_max.expectedCycleTime())
        return;

    // publish external pose
    m_pub_externalPose.publish(m_msgs_extPose);

    // publish setpoint
    m_msgs_setpoint.header.stamp = ros::Time::now();
    m_pub_setpoint.publish(m_msgs_setpoint);

    m_last_pub_time = ros::Time::now();
}

void Client::quatextract(uint32_t quat, float* q){
    float ssum = 0;

    int i_largest = quat >> 30;
    for(int i = 3; i >= 0; i--){
        if(i != i_largest){
            unsigned negbit = quat >> 9 & 0x01;
            unsigned mag = quat & ((1 << 9) - 1);
            quat = quat >> 10;

            q[i] = (negbit ? -1.0f : 1.0f) * (float)mag * (float)M_SQRT1_2 / (float)((1 << 9) - 1);
            ssum += q[i] * q[i];
        }
    }
    q[i_largest] = sqrt(1 - ssum);
}


