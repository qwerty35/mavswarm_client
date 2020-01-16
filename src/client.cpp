#include <mutex>

#include "client.h"
#include "crtp.h"
#include "crtpBootloader.h"
#include "crtpNRF51.h"

#include "Crazyradio.h"
#include "CrazyflieUSB.h"

#include <ros/console.h>
#include <sstream>
#include <fstream>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <cmath>
#include <inttypes.h>
#include <client.h>
#include <crtp.h>


const static int MAX_RADIOS = 1;
const static bool LOG_COMMUNICATION = 0;

//Crazyradio* g_crazyradios[MAX_RADIOS];
//std::mutex g_radioMutex[MAX_RADIOS];
//
//CrazyflieUSB* g_crazyflieUSB[MAX_USB];
//std::mutex g_crazyflieusbMutex[MAX_USB];

Client::Client(
        ros::NodeHandle nh,
        const std::string& link_uri)
        : m_rosNodeHandle(nh)
        , m_radio(nullptr)
        , m_transport(nullptr)
        , m_devId(0)
        , m_channel(0)
        , m_address(0)
        , m_datarate(Crazyradio::Datarate_250KPS)
        , m_pingPassed(false)
        , m_count(0)
{
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

        if (m_devId > MAX_RADIOS) {
            throw std::runtime_error("This version does not support that many radios. Adjust MAX_RADIOS and recompile!");
        }

        {
            m_radio = new Crazyradio(0);
            m_radio->setAckEnable(true);
            m_radio->setArc(0);
            m_radio->setChannel(m_channel);
            m_radio->setAddress(m_address);
            m_radio->setMode(m_radio->Mode_PRX);
        }
    }

    m_pub_externalPose = m_rosNodeHandle.advertise<geometry_msgs::PoseStamped>("/vicon/cf1/cf1", 5); //TODO: fix topic name
}

void Client::listen() {
    uint32_t length;
    uint8_t data[32];

    // wait for packet
    while(ros::ok()){
        m_radio->setAddress(m_address);
        if(m_radio->receivePacket(data, length)) {
            handleData(data);
            break;
        }
        m_radio->setAddress(0xFFE7E7E7E7);
        m_radio->setAckEnable(false);
        if(m_radio->receivePacket(data, length)){
            handleData(data);
            break;
        }
    }
}

void Client::handleData(const uint8_t* data){
    // external pose
    if(crtp(data[0]) == crtp(6, 1)){
        publishExternalPose(data);
        return;
    }

    uint8_t ack[32];
    uint32_t ack_size;
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
    else if(crtp(data[0]) == crtp(4, 0)){
        ack[0] = 0x40;
        ack[1] = 0x01;
        ack_size = 3;
    }
    m_radio->sendPacketNoAck((uint8_t*)&ack, ack_size);
}

void Client::publishExternalPose(const uint8_t* data) {
    crtpExternalPosePacked* extPose_crtp = (crtpExternalPosePacked*)data;
    for(int i = 0; i < 2; i++) {
        if (m_devId == extPose_crtp->poses[i].id) {
            geometry_msgs::PoseStamped extPose_msgs;
            extPose_msgs.pose.position.x = (float)extPose_crtp->poses[i].x/1000;
            extPose_msgs.pose.position.y = (float)extPose_crtp->poses[i].y/1000;
            extPose_msgs.pose.position.z = (float)extPose_crtp->poses[i].z/1000;

            m_pub_externalPose.publish(extPose_msgs);
        }
    }
}


