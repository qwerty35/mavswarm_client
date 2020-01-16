#pragma once

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
    Client(const std::string& link_uri);

    void listen();
    void publishVisionPose();
private:
    Crazyradio* m_radio;
    ITransport* m_transport;
    int m_devId;

    uint8_t m_channel;
    uint64_t m_address;
    Crazyradio::Datarate m_datarate;

    void handleData(const uint8_t* data, uint32_t length, ITransport::Ack& ack);
};

