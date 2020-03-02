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
#include <crtp.h>

const static int MAX_RADIOS = 1;

Client::Client()
        : m_radio(nullptr)
        , m_transport(nullptr)
        , m_dev_id(0)
        , m_channel(0)
        , m_address(0)
        , m_datarate(Crazyradio::Datarate_250KPS)
{
    // ros
    m_nh = ros::NodeHandle("~");
    m_pub_setpoint = m_nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    m_pub_setpoint_raw = m_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    m_pub_mocap_pose = m_nh.advertise<geometry_msgs::PoseStamped>("/mavswarm_client/mocap_pose", 10);
    m_pub_camera_odom = m_nh.advertise<geometry_msgs::PoseStamped>("/mavswarm_client/camera_pose", 10);
    m_pub_vision_pose = m_nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    
    m_sub_current_state = m_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &Client::mavrosStateCallback, this);
    m_arming_client = m_nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    m_set_mode_client = m_nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    m_emergency_stop_client = m_nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/emergency_stop");

    m_nh.param<std::string>("uri", m_mav_uri, "radio://0/50/2M/E7E7E7E701");
    m_nh.param<int>("mav_id", m_mav_id, 1);
    m_nh.param<std::string>("frame_id", m_frame_id, "/world");
    m_nh.param<bool>("mocap_exist", m_mocap_exist, false);
    m_nh.param<bool>("camera_exist", m_camera_exist, false);
    m_nh.param<bool>("use_vio", m_use_vio, false);

    // get radio info from uri
    int datarate, channel;
    char datarateType;
    bool success = false;
    success = std::sscanf(m_mav_uri.c_str(), "radio://%d/%d/%d%c/%" SCNx64,
            &m_dev_id, &channel, &datarate,
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

        if (m_dev_id >= MAX_RADIOS) {
            throw std::runtime_error("This version does not support that many radios. Adjust MAX_RADIOS and recompile!");
        }

        {
            m_radio = new Crazyradio(m_dev_id);
            m_radio->setAckEnable(true);
            m_radio->setArc(3);
            m_radio->setChannel(m_channel);
            m_radio->setAddress(m_address); // mav address
            m_radio->setAddr2(0xFFE7E7E7E7); // broadcast address
            m_radio->setMode(m_radio->Mode_PRX);
            m_radio->setDatarate(m_datarate);
        }
    }

    // initialize mav info
    m_current_mission = NONE;
    m_msgs_camera_pose.header.frame_id = m_frame_id;
    m_msgs_mocap_pose.header.frame_id = m_frame_id;
    m_vision_pose_updated = false;
    m_haveTrajectory = false;
    m_link = 0xFF;
    m_last_command_time = ros::Time::now();
}

void Client::run() {
    uint32_t length;
    uint8_t data[32];
    ros::Rate rate_max = 80;

    int count_max = 500;
    int trial_count = 0;
    int ping_count = 0;
    ros::Time start_time = ros::Time::now();
    m_last_pub_time = ros::Time::now();    

    ROS_INFO("[MAVSWARM_CLIENT] Start mavswarm_client");
    while(ros::ok()){
        // Receive packet from crazyradio
        if (!m_radio->receivePacket(data, length)) {
            if(!m_use_vio){
                m_vision_pose_updated = false;
            }
        }
        else{
            handleData(data);
            ping_count++;
        }
        trial_count++;

        // link quality and rate
        if(trial_count == count_max){
            if(!m_use_vio) {
                m_link_quality = (double) ping_count / (double) trial_count;
                m_link_rate = (double) ping_count / (ros::Time::now() - start_time).toSec();

                ROS_INFO_STREAM(
                        "[MAVSWARM_CLIENT] link_quality: " << m_link_quality << ", link_rate: " << m_link_rate << "Hz");
            }
            trial_count = 0;
            ping_count = 0;
            start_time = ros::Time::now();
        }

        updateSetpoints();
        publishMsgs(rate_max);
        ros::spinOnce();
    }
}

void Client::handleData(uint8_t* data){
    // mocap pose
    if(crtp(data[0]) == crtp(6, 1) && data[1] == 9){
        receiveMocapPose(data);
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
        data[0] = data[0] & 0x0c;
        ack_size = 2;
    }
    // write memory
    else if(crtp(data[0]) == crtp(4, 2)) {
//        ROS_INFO_STREAM("data link" << (data[0] & 0x0c));
//        ROS_INFO_STREAM("m_link" << int(m_link));

        if(!isDuplicatedMessage(data)){
            writeMemory(data);
            m_link = (data[0] & 0x0c);
        }

        data[5] = 0;
        ack_size = 6;
    }
    // High level setpoint - take off
    else if(crtp(data[0]) == crtp(8, 0) && data[1] == 1){
        takeoff(data);

        data[3] = 0;
        ack_size = 4;
    }
    // High level setpoint - land
    else if(crtp(data[0]) == crtp(8, 0) && data[1] == 2){
        land(data);

        data[3] = 0;
        ack_size = 4;
    }
    // High level setpoint - stop
    else if(crtp(data[0]) == crtp(8, 0) && data[1] == 3){
        data[3] = 0;
        ack_size = 4;
    }
    // High level setpoint - goto
    else if(crtp(data[0]) == crtp(8, 0) && data[1] == 4){
        goTo(data);

        data[3] = 0;
        ack_size = 4;
    }
    // High level setpoint - start trajectory
    else if(crtp(data[0]) == crtp(8, 0) && data[1] == 5){
        startTrajectory(data);

        data[3] = 0;
        ack_size = 4;
    }
    // High level setpoint - define trajectory
    else if(crtp(data[0]) == crtp(8, 0) && data[1] == 6){
        if(!isDuplicatedMessage(data)){
            uploadTrajectory(data);
            m_link = (data[0] & 0x0c);
        }

        data[3] = 0;
        ack_size = 4;
    }
    else{
        ROS_ERROR("[MAVSWARM_CLIENT] unsupported crtp message");
        return;
    }

    m_radio->sendPacketNoAck(data, ack_size);
}

void Client::receiveMocapPose(const uint8_t* data) {
    auto extPose_crtp = (crtpExternalPosePacked*)data;

    if(!m_use_vio) {
        m_vision_pose_updated = true;
    }

    for(int i = 0; i < 2; i++) {
        if (m_mav_id == extPose_crtp->poses[i].id) {
            m_msgs_mocap_pose.header.stamp = ros::Time::now();

            m_msgs_mocap_pose.pose.position.x = (float)extPose_crtp->poses[i].x/1000;
            m_msgs_mocap_pose.pose.position.y = (float)extPose_crtp->poses[i].y/1000;
            m_msgs_mocap_pose.pose.position.z = (float)extPose_crtp->poses[i].z/1000;

            float q[4];
            quatextract(extPose_crtp->poses[i].quat, q);
            m_msgs_mocap_pose.pose.orientation.x = q[0];
            m_msgs_mocap_pose.pose.orientation.y = q[1];
            m_msgs_mocap_pose.pose.orientation.z = q[2];
            m_msgs_mocap_pose.pose.orientation.w = q[3];
        }
    }
}

void Client::writeMemory(const uint8_t* data){
    auto cmd_writeMem_crtp = (crtpMemoryWriteRequest*)data;
    m_traj_rawdata.resize(m_traj_rawdata.size() + 1);
    std::move(std::begin(cmd_writeMem_crtp->data), std::end(cmd_writeMem_crtp->data), m_traj_rawdata.back().begin());
}

void Client::uploadTrajectory(const uint8_t* data){
    if(m_traj_rawdata.empty()){
        ROS_INFO("[MAVSWARM_CLIENT] memory is empty, duplicated upload message?");
        return;
    }

    auto def_traj_crtp = (crtpCommanderHighLevelDefineTrajectoryRequest*)data;
    ROS_INFO_STREAM("traj id: " << (int)def_traj_crtp->trajectoryId);
    if(def_traj_crtp->description.trajectoryType != TRAJECTORY_TYPE_POLY4D){
        ROS_ERROR("[MAVSWARM_CLIENT] trajectory type is not poly4d");
        return;
    }
    uint8_t n_pieces = def_traj_crtp->description.trajectoryIdentifier.mem.n_pieces;

    //TODO: traj check
    auto piece = reinterpret_cast<Crazyflie::poly4d*>(m_traj_rawdata.data());
    m_traj_coef.resize(n_pieces);
    for(int i = 0; i < n_pieces; i++){
        m_traj_coef[i] = piece[i];
        ROS_INFO_STREAM("T"<< std::to_string(i) << ": " << m_traj_coef[i].duration);
    }

    ROS_INFO_STREAM("[MAVSWARM_CLIENT] upload trajectory complete, total transferred packet: "  << m_traj_rawdata.size());
    m_traj_rawdata.clear();
    m_haveTrajectory = true;
}

void Client::startTrajectory(const uint8_t *data) {
    if(m_current_mission != HOVER){
        ROS_ERROR("[MAVSWARM_CLIENT] start trajectory failed, mav is not ready");
        return;
    }
    if(!m_haveTrajectory){
        ROS_ERROR("[MAVSWARM_CLIENT] start trajectory failed, the trajectory is not updated yet");
        return;
    }

    auto cmd_startTraj_crtp = (crtpCommanderHighLevelStartTrajectoryRequest*)data;
    if(cmd_startTraj_crtp->relative){
        ROS_ERROR("[MAVSWARM_CLIENT] start trajectory failed, relative trajectory is not supported yet");
        return;
    }
    if(cmd_startTraj_crtp->reversed){
        ROS_ERROR("[MAVSWARM_CLIENT] start trajectory failed, reverse trajectory is not supported yet");
        return;
    }
    if(!m_vision_pose_updated){
        ROS_ERROR("[MAVSWARM_CLIENT] start trajectory failed, vision_pose is not updated");
    }

    m_last_command_time = ros::Time::now();
    m_current_mission = TRAJECTORY;
}

void Client::updateSetpoints(){
    double current_time = (ros::Time::now() - m_last_command_time).toSec();

    switch(m_current_mission){
        case NONE:
        case HOVER:
            break;
        case TAKEOFF:
        case GOTO: {
            double remaining_time = (m_desired_pose.header.stamp - ros::Time::now()).toSec();
            if (remaining_time > 0) {
                // if remaining time exist, then linear interpolation
                double alpha = current_time / (current_time + remaining_time);
                m_msgs_setpoint.pose.position.x = (1-alpha) * m_start_pose.pose.position.x + alpha * m_desired_pose.pose.position.x;
                m_msgs_setpoint.pose.position.y = (1-alpha) * m_start_pose.pose.position.y + alpha * m_desired_pose.pose.position.y;
                m_msgs_setpoint.pose.position.z = (1-alpha) * m_start_pose.pose.position.z + alpha * m_desired_pose.pose.position.z;

                double yaw = (1-alpha) * extractYaw(m_start_pose) + alpha * extractYaw(m_desired_pose);
                m_msgs_setpoint.pose.orientation = yaw2quat(yaw);
            } else {
                ROS_INFO("[MAVSWARM_CLIENT] mission complete");
                m_msgs_setpoint.pose = m_desired_pose.pose;
                m_current_mission = HOVER;
            }
            break;
        }
        case TRAJECTORY: {
            double t, seg_start_time = 0;
            int m_curr;
            // find segment start time;
            for (int m = 0; m < m_traj_coef.size(); m++) {
                if (current_time > seg_start_time + m_traj_coef[m].duration) {
                    seg_start_time += m_traj_coef[m].duration;
                } else {
                    m_curr = m;
                    break;
                }
            }

            // check trajectory is over
            if (m_curr >= m_traj_coef.size()) {
                ROS_INFO("[MAVSWARM_CLIENT] mission complete");

                //reinitialize setpoint_position
                m_msgs_setpoint.pose.position = m_msgs_setpoint_raw.position;
                m_msgs_setpoint.pose.orientation = yaw2quat(m_msgs_setpoint_raw.yaw);

                //initialize mission state
                m_current_mission = HOVER;

                return;
            } else {
                t = current_time - seg_start_time;
            }

            double x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0, yaw = 0;
            for (int i = 0; i < 8; i++) {
                x += m_traj_coef[m_curr].p[0][i] * pow(t, i);
                y += m_traj_coef[m_curr].p[1][i] * pow(t, i);
                z += m_traj_coef[m_curr].p[2][i] * pow(t, i);
                yaw += m_traj_coef[m_curr].p[3][i] * pow(t, i);

                vx += i * m_traj_coef[m_curr].p[0][i] * pow(t, i - 1);
                vy += i * m_traj_coef[m_curr].p[1][i] * pow(t, i - 1);
                vz += i * m_traj_coef[m_curr].p[2][i] * pow(t, i - 1);
            }

            const uint16_t type_mask = (1 << 11) | (7 << 6); // with velocity
//        const uint16_t type_mask = (1 << 11) | (7 << 6) | (7 << 3); //without velocity
            m_msgs_setpoint_raw.type_mask = type_mask;
            m_msgs_setpoint_raw.position.x = x;
            m_msgs_setpoint_raw.position.y = y;
            m_msgs_setpoint_raw.position.z = z;
            m_msgs_setpoint_raw.yaw = yaw;
            m_msgs_setpoint_raw.velocity.x = vx;
            m_msgs_setpoint_raw.velocity.y = vy;
            m_msgs_setpoint_raw.velocity.z = vz;
            break;
        }
        case LAND: {
            break;
        }
    }
}

void Client::takeoff(const uint8_t* data) {
    auto cmd_takeoff_crtp = (crtpCommanderHighLevelTakeoffRequest*)data;

    // prevent duplicated mission
    if (ros::Time::now() - m_last_command_time < ros::Duration(0.05)) {
        return;
    }
    if(m_current_mission != NONE){
        ROS_ERROR("[MAVSWARM_CLIENT] takeoff failed, mav is in air");
        return;
    }
    if(!m_vision_pose_updated){
        ROS_WARN("[MAVSWARM_CLIENT] takeoff failed, no vision pose");
        return;
    }
    
    // initialize setpoint
    initializeSetpoint();

    // change to offboard mode
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

    // arming
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

    // initialize desired pose and start pose
    m_start_pose = m_msgs_vision_pose;
    m_desired_pose = m_msgs_vision_pose;
    m_desired_pose.pose.position.z = cmd_takeoff_crtp->height;
    m_desired_pose.header.stamp = ros::Time::now() + ros::Duration(cmd_takeoff_crtp->duration);

    // mission -> Takeoff
    m_current_mission = TAKEOFF;
    m_last_command_time = ros::Time::now();
}

void Client::goTo(const uint8_t* data) {
    // prevent duplicated mission
    if (ros::Time::now() - m_last_command_time < ros::Duration(0.05)) {
        return;
    }
    if(m_current_mission != HOVER){
        ROS_ERROR("[MAVSWARM_CLIENT] goto failed, mav is not ready");
        return;
    }

    auto cmd_goto_crtp = (crtpCommanderHighLevelGoToRequest*)data;
    double x,y,z,yaw;
    if(!cmd_goto_crtp->relative) {
        x = cmd_goto_crtp->x;
        y = cmd_goto_crtp->y;
        z = cmd_goto_crtp->z;
        yaw = cmd_goto_crtp->yaw;
    }
    else{
        x = m_msgs_setpoint.pose.position.x + cmd_goto_crtp->x;
        y = m_msgs_setpoint.pose.position.y + cmd_goto_crtp->y;
        z = m_msgs_setpoint.pose.position.z + cmd_goto_crtp->z;
        yaw = extractYaw(m_msgs_setpoint);
        yaw += cmd_goto_crtp->yaw;
    }

    m_start_pose = m_msgs_setpoint;
    m_desired_pose.pose.position.x = x;
    m_desired_pose.pose.position.y = y;
    m_desired_pose.pose.position.z = z;
    m_desired_pose.pose.orientation = yaw2quat(yaw);
    m_desired_pose.header.stamp = ros::Time::now() + ros::Duration(cmd_goto_crtp->duration);

    ROS_INFO_STREAM("[MAVSWARM_CLIENT] go to setpoint, x: "   << x
                                                      << ", y: "   << y
                                                      << ", z: "   << z
                                                      << ", yaw: " << yaw);
    m_current_mission = GOTO;
    m_last_command_time = ros::Time::now();
}

void Client::land(const uint8_t* data){
    auto cmd_land_crtp = (crtpCommanderHighLevelLandRequest*)data;

    if(!m_vision_pose_updated){
        ROS_WARN("[MAVSWARM_CLIENT] No vision pose, do not land");
        return;
    }

    m_msgs_setpoint = m_msgs_vision_pose;
    m_msgs_setpoint.pose.position.z = 0.00;
    ROS_INFO_STREAM("[MAVSWARM_CLIENT] Landing setpoint, x: " << m_msgs_setpoint.pose.position.x
                                                         << ", y: " << m_msgs_setpoint.pose.position.y
                                                         << ", z: " << m_msgs_setpoint.pose.position.z);

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";
    if( m_current_state.mode != "AUTO.LAND"){
        if( m_set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent){
            ROS_INFO("[MAVSWARM_CLIENT] Landing enabled");
        }
        else{
            ROS_ERROR("[MAVSWARM_CLIENT] Landing failed");
            return;
        }
    }

    m_current_mission = LAND;
    m_last_command_time = ros::Time::now();
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

    m_current_mission = NONE;
    m_last_command_time = ros::Time::now();
}

void Client::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg){
    m_current_state = *msg;
}

void Client::publishMsgs(ros::Rate rate_max) {
    if (ros::Time::now() - m_last_pub_time < rate_max.expectedCycleTime()) {
        return;
    }

    // publish external pose from motion capture system
    if(m_mocap_exist) {
        m_pub_mocap_pose.publish(m_msgs_mocap_pose);
    }

    // publish camera pose
    if(m_camera_exist) {
        tf::StampedTransform transform;
        bool camera_pose_updated = false;
        try {
            tf_listener.lookupTransform("/camera_odom_frame", "/camera_pose_frame", ros::Time(0), transform);
            m_msgs_camera_pose.header.stamp = ros::Time::now();
            m_msgs_camera_pose.header.frame_id = m_frame_id;
            m_msgs_camera_pose.pose.position.x = transform.getOrigin().x();
            m_msgs_camera_pose.pose.position.y = transform.getOrigin().y();
            m_msgs_camera_pose.pose.position.z = transform.getOrigin().z();
            m_msgs_camera_pose.pose.orientation.x = transform.getRotation().x();
            m_msgs_camera_pose.pose.orientation.y = transform.getRotation().y();
            m_msgs_camera_pose.pose.orientation.z = transform.getRotation().z();
            m_msgs_camera_pose.pose.orientation.w = transform.getRotation().w();

            m_pub_camera_odom.publish(m_msgs_camera_pose);
            camera_pose_updated = true;
        }
        catch (tf::TransformException ex) {
            camera_pose_updated = false;
            ROS_ERROR("%s", ex.what());
        }
        if(m_use_vio) {
            m_vision_pose_updated = camera_pose_updated;
        }
    }

    // publish vision pose for px4
    if(m_camera_exist && m_use_vio){
        m_msgs_vision_pose = m_msgs_camera_pose;
    }
    else if(m_mocap_exist && !m_use_vio){
        m_msgs_vision_pose = m_msgs_mocap_pose;
    }
    else{
        ROS_ERROR("[MAVSWARM_CLIENT] vision_pose is not published, check launch file param");
    }

    if(m_vision_pose_updated) {
        m_pub_vision_pose.publish(m_msgs_vision_pose);
    }
    else{
        ROS_WARN("[MAVSWARM_CLIENT] vision pose is not updated, do not publish vision pose");
    }

    // publish setpoint
    if(m_current_mission == TRAJECTORY){
        m_msgs_setpoint_raw.header.stamp = ros::Time::now();
        m_pub_setpoint_raw.publish(m_msgs_setpoint_raw);
    }
    else{
        m_msgs_setpoint.header.stamp = ros::Time::now();
        m_pub_setpoint.publish(m_msgs_setpoint);
    }

    m_last_pub_time = ros::Time::now();
}

bool Client::isDuplicatedMessage(const uint8_t *data) {
    bool ret = false;
    if(m_link != 0xFF){
        ret = (m_link == (data[0] & 0x0c));
    }
    return ret;
}

void Client::initializeSetpoint() {
    m_msgs_setpoint = m_msgs_vision_pose;
    ROS_INFO_STREAM("[MAVSWARM_CLIENT] Initilize setpoint, x: " << m_msgs_setpoint.pose.position.x
                                                                << ", y: " << m_msgs_setpoint.pose.position.y
                                                                << ", z: " << m_msgs_setpoint.pose.position.z);
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

double Client::extractYaw(geometry_msgs::PoseStamped poseStamped){
    double q0 = poseStamped.pose.orientation.w;
    double q3 = poseStamped.pose.orientation.z;
    return atan2(2 * q0 * q3, 1 - 2 * q3 * q3);
}

geometry_msgs::Quaternion Client::yaw2quat(double yaw){
    geometry_msgs::Quaternion quat;
    quat.w = cos(yaw / 2);
    quat.x = 0;
    quat.y = 0;
    quat.z = sin(yaw / 2);

    return quat;
}