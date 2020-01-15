#include <mutex>

#include "client.h"
#include "crtp.h"
#include "crtpBootloader.h"
#include "crtpNRF51.h"

#include "Crazyradio.h"
#include "CrazyflieUSB.h"

#include <fstream>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <cmath>
#include <inttypes.h>
#include <client.h>


const static int MAX_RADIOS = 1;
const static bool LOG_COMMUNICATION = 0;

//Crazyradio* g_crazyradios[MAX_RADIOS];
//std::mutex g_radioMutex[MAX_RADIOS];
//
//CrazyflieUSB* g_crazyflieUSB[MAX_USB];
//std::mutex g_crazyflieusbMutex[MAX_USB];

Client::Client(
        const std::string& link_uri)
        : m_radio(nullptr)
        , m_transport(nullptr)
        , m_devId(0)
        , m_channel(0)
        , m_address(0)
        , m_datarate(Crazyradio::Datarate_250KPS)
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
            m_radio = new Crazyradio(m_devId);
            m_radio->setAckEnable(true);
            m_radio->setArc(0);
            m_radio->setChannel(m_channel);
            m_radio->setAddress(m_address);
            m_radio->setMode(m_radio->Mode_PRX);
        }
    }
}

void Client::waitMessage() {
    uint8_t* data;
    uint32_t length;
    while(m_radio->receivePacket(data, length) == false) {

    }

    int debug = 0;
}

