//
// Created by dav on 1/28/24.
//

#ifndef BLIMPSWARM_BASECOMMUNICATOR_H
#define BLIMPSWARM_BASECOMMUNICATOR_H

#include <cstdint>
#include "LowLevelComm.h"
#include "util/data_types.h"
#include "util/ParamManager.h"
#include <Preferences.h>

class BaseCommunicator {
public:
    explicit BaseCommunicator(LowLevelComm* comm); // Constructor declaration

    void setMainBaseStation();

    // During boot, off, and periodically just ping the stations.
    void addPingStation(const uint8_t mac_addr[6]);
    void pingStations();

    // Send measurements to base station
    bool sendMeasurements(ReceivedData* measurements);

    bool readNewMessages();
    // Commands from Base station


    ControlInput receiveMsgCmd();
    bool isNewMsgCmd();

    // Send all params to the base station
//    bool sendParams(); //TODO

private:
    LowLevelComm* comm; // Object that manager the low level communication
    uint8_t main_station_mac[6];  // Main station mac

    static const int MAX_PING_STATIONS = 5; // Maximum number of ping stations
    uint8_t pStations[MAX_PING_STATIONS][6]; // Array to store MAC addresses of ping stations
    int numPingStations = 0; // Number of ping stations currently added
    Preferences preferences;

    unsigned long previousMillis = 0; // Stores the last time a message was sent
    const long interval = 200; // Interval at which to run the sender (milliseconds) //TODO make SSD

    // Messages
    ControlInput* msgCmd;
    bool newMsgCmd = false;
    ReceivedData* msgMeasure;
    bool newMeasure = false;
    //updateParam(const uint8_t* data, unsigned int length); //TODO
};


#endif //BLIMPSWARM_BASECOMMUNICATOR_H
