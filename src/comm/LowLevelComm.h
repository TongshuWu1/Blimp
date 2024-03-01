//
// Created by dav on 1/22/24.
//

#ifndef BLIMPSWARM_LOWLEVELCOMM_H
#define BLIMPSWARM_LOWLEVELCOMM_H
#include <Arduino.h>

static const int MAX_DATA_SIZE = 100;


class LowLevelComm {
public:
    // Network Initialization
    virtual void init() = 0;

    // Pure virtual function to send data
    virtual void sendData(const uint8_t mac_addr[6], const uint8_t* data, unsigned int length) = 0;


    virtual void receiveData(uint8_t receivedData[MAX_DATA_SIZE], int& length) = 0;
    virtual bool newData() = 0;
    virtual void addPeer(const uint8_t *peerAddr) = 0;


//    // Pure virtual function to receive data
//    virtual ControlInput receiveLongData() = 0;
//    // True if new data arrived
//    virtual bool newLongData() = 0;
//
//
//    virtual ReceivedData receiveShortData() = 0;
//    virtual bool newShortData() = 0;




};




#endif //BLIMPSWARM_LOWLEVELCOMM_H
