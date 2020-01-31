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
        const std::string& frame_id)
        : m_radio(nullptr)
        , m_transport(nullptr)
        , m_mavId(mav_id)
        , m_devId(0)
        , m_channel(0)
        , m_address(0)
        , m_datarate(Crazyradio::Datarate_250KPS)
        , m_frame_id(frame_id)
{
    m_pub_externalPose = m_rosNodeHandle.advertise<geometry_msgs::PoseStamped>("mavswarm_client/pose", 10);
    m_sub_current_state = m_rosNodeHandle.subscribe<mavros_msgs::State>("mavros/state", 10, &Client::mavros_state_callback, this);
    m_arming_client = m_rosNodeHandle.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    m_set_mode_client = m_rosNodeHandle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

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
    m_broadcast_fail_count = 0;
}

void Client::run() {
    uint32_t length;
    uint8_t data[32];

    ROS_INFO("[MAVSWARM_CLIENT] Start mavswarm_client");
    while(ros::ok()){
        if (!m_radio->receivePacket(data, length)) {
            ROS_WARN("[MAVSWARM_CLIENT] ping failed");
            m_is_extPose_received = false;
            m_broadcast_fail_count++;
        }
        else{
            m_is_extPose_received = true;
            handleData(data);
        }

        m_pub_setpoint.publish(m_setpoint_msgs);
        ros::spinOnce();
    }
}

void Client::handleData(const uint8_t* data){
    // external pose
    if(crtp(data[0]) == crtp(6, 1)){
        publishExternalPose(data);
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

void Client::publishExternalPose(const uint8_t* data) {
    auto extPose_crtp = (crtpExternalPosePacked*)data;
    for(int i = 0; i < 2; i++) {
        if (m_mavId == extPose_crtp->poses[i].id) {
            m_extPose_msgs.header.frame_id = m_frame_id;
            m_extPose_msgs.header.stamp = ros::Time::now();

            m_extPose_msgs.pose.position.x = (float)extPose_crtp->poses[i].x/1000;
            m_extPose_msgs.pose.position.y = (float)extPose_crtp->poses[i].y/1000;
            m_extPose_msgs.pose.position.z = (float)extPose_crtp->poses[i].z/1000;

            float q[4];
            quatextract(extPose_crtp->poses[i].quat, q);
            m_extPose_msgs.pose.orientation.x = q[0];
            m_extPose_msgs.pose.orientation.y = q[1];
            m_extPose_msgs.pose.orientation.z = q[2];
            m_extPose_msgs.pose.orientation.w = q[3];

            m_pub_externalPose.publish(m_extPose_msgs);
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
    m_setpoint_msgs = m_extPose_msgs;
    m_setpoint_msgs.pose.position.z = cmd_takeoff_crtp->height;
    ROS_INFO_STREAM("Initilize setpoint, x: " << m_setpoint_msgs.pose.position.x
                                   << ", y: " << m_setpoint_msgs.pose.position.y
                                   << ", z: " << m_setpoint_msgs.pose.position.z);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if( m_set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
        ROS_INFO("[MAVSWARM_CLIENT] Offboard enabled");
    }
    else{
        ROS_ERROR("[MAVSWARM_CLIENT] Offboard failed");
        return;
    }

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

void Client::mavros_state_callback(const mavros_msgs::State::ConstPtr& msg){
    m_current_state = *msg;
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


