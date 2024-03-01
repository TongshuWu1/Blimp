
#ifndef ESPNOWCOMM_H
#define ESPNOWCOMM_H


#include "LowLevelComm.h"

#include "WiFi.h"
#include <esp_now.h> // This is the arduino library for ESP-NOW
#include "util/ParamManager.h"



volatile bool new_data_received = false;
volatile uint8_t new_data[MAX_DATA_SIZE];
volatile int new_data_len;


volatile bool verbose = false;  //FIXME: this should be a parameter
volatile unsigned long esp_time_now;


//ControlInput ESPNOW_Input;
//ReceivedData ESPNOW_ReceivedData;

int delayMS = 1000;

ParamManager manager;


// Callback when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len){

    char macStr[18];
    if (verbose) {
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    }



//    if (verbose) {
//        Serial.print("Packet from: ");
//        Serial.println(macStr);
//        Serial.print("Control params: ");
//        for (int i = 0; i < NUM_CONTROL_PARAMS; i++)
//        {
//            Serial.print(incomingData->params[i]);
//            if (i < NUM_CONTROL_PARAMS - 1)
//            {
//                Serial.print(", ");
//            }
//        }
//        Serial.println("\tListening from P2P");
//    }


    // Time of the last received package
    esp_time_now = millis();
    
    // Flag for received data
    new_data_received = true;

    // Assuming new_data is a volatile buffer and its size is known
    for (size_t i = 0; i < data_len; ++i) {
        new_data[i] = data[i]; // Copy each byte
    }

    new_data_len = data_len;
    if (data[0] == 0x68){
        Serial.print("Rcv: ");
        Serial.print(data_len);
        Serial.print(",");
        Serial.print(data[0]);
        Serial.print(",");
        Serial.println(data[1]);
        manager.parseAndSetPreference(data, data_len);
    } 
    else if (data[0] == 0x70){
        Serial.print("Gnd: ");
        Serial.print(data_len);
        Serial.print(",");
        Serial.print(data[0]);
        Serial.print(",");
        Serial.println(data[1]);
        // const uint8_t gnd_mac[6];
        // for (size_t i = 0; i < 6; ++i) {
        //     gnd_mac[i] = data[i+1]; // Copy each byte
        // }
        manager.setGroundMac(&data[1]);
    } 

    // Print received info
//    if (verbose) {
//        Serial.print("Packet from: ");
//        Serial.println(macStr);
//        Serial.print("Control params: ");

//        for (int i = 0; i < data_len; i++)
//        {
//            Serial.print(incomingData->params[i]);
//            if (i < NUM_CONTROL_PARAMS - 1)
//            {
//                Serial.print(", ");
//            }
//        }
//        Serial.println("\tListening from P2P");
//    }


//
//    // Identify the package type Control_input, or data by the package size
//    if (data_len == sizeof(ControlInput)){
//        ControlInput *incomingData = (ControlInput *)data;
//
//        if (incomingData->channel == -1){ // Check if the data is P2P
//            // Print received info
//            if (verbose) {
//                Serial.print("Packet from: ");
//                Serial.println(macStr);
//                Serial.print("Control params: ");
//                for (int i = 0; i < NUM_CONTROL_PARAMS; i++)
//                {
//                    Serial.print(incomingData->params[i]);
//                    if (i < NUM_CONTROL_PARAMS - 1)
//                    {
//                        Serial.print(", ");
//                    }
//                }
//                Serial.println("\tListening from P2P");
//            }
//
//            // Store received data in ESPNOW_Input
//            esp_time_now = millis();
//            esp_ready = false;  // FIXME unnecessary?
//            memcpy(&ESPNOW_Input, incomingData, sizeof(ESPNOW_Input));
//            esp_ready = true;
//
//        }else if (incomingData->channel == BROADCAST){ // Check if the data is broadcast
//            // FIXME: this code seems the same as the code in the previous if
//            if (verbose) {
//                Serial.print("Packet from: ");
//                Serial.println(macStr);
//                Serial.print("Control params: ");
//                for (int i = 0; i < NUM_CONTROL_PARAMS; i++)
//                {
//                    Serial.print(incomingData->params[i]);
//                    if (i < NUM_CONTROL_PARAMS - 1)
//                    {
//                        Serial.print(", ");
//                    }
//                }
//                Serial.print("\tListening on channel: ");
//                Serial.println(incomingData->channel);
//            }
//
//            // Store received data in ESPNOW_Input
//            esp_time_now = millis();
//            esp_ready = false;
//            memcpy(&ESPNOW_Input, incomingData, sizeof(ESPNOW_Input));
//            esp_ready = true;
//        }
//        else
//        {
//            Serial.println("Data received on an unexpected channel. Ignoring.");
//        }
//    } else if (data_len == sizeof(ReceivedData)) {
//        // Store received data in ESPNOW_ReceivedData
//        ReceivedData *incomingSensorData = (ReceivedData *)data;
//        memcpy(&ESPNOW_ReceivedData, incomingSensorData, sizeof(ESPNOW_ReceivedData));
//        esp_sensor_ready = true;
//    }
//    else {
//        Serial.println("Received data of unexpected size. Ignoring.");
//    }
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (verbose)
    {
        Serial.print(" Status: ");
        Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    }
}






class LLC_ESPNow : public LowLevelComm {
private:
    const int MAC_ADDRESS_SIZE = 6;
    esp_now_peer_info_t peerInfo;
public:
    LLC_ESPNow() {

    }

    void init() override{
        // Set device as a Wi-Fi Station
        WiFi.mode(WIFI_STA);
        Serial.print("ESP Board MAC Address:  ");
        Serial.println(WiFi.macAddress());

       esp_now_deinit(); //FIXME, this line seens useful
        if (esp_now_init() != ESP_OK)
        {
            Serial.println("Error initializing ESP-NOW");
            return;
        }else{
            Serial.println("ESP-NOW initialized");
        }
        //esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

        // Register for a callback function that will be called when data is received
        esp_now_register_recv_cb(OnDataRecv);
        esp_now_register_send_cb(OnDataSent);

        esp_time_now = millis();

    }


    void sendData(const uint8_t mac_addr[6], const uint8_t* data, unsigned int length) override {
        // Send data using ESP-NOW
        esp_err_t result = esp_now_send(mac_addr, (uint8_t *) data, length);
        
        
        // Serial.print("Sending Data Result: ");
        // for (int i = 0; i < 6; ++i) {
        //     if (i > 0) {
        //         Serial.print(":");
        //     }
        //     // Print each byte in hexadecimal format
        //     if (mac_addr[i] < 16) {
        //         Serial.print("0"); // Print a leading zero for values less than 0x10
        //     }
        //     Serial.print(mac_addr[i], HEX);
        // }
        // Serial.print(" ");
        // if (result == ESP_OK) {
        //     Serial.println("Success!");
        // } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
        //     Serial.println("ESP-NOW not initialized.");
        // } else if (result == ESP_ERR_ESPNOW_ARG) {
        //     Serial.println("Invalid argument.");
        // } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
        //     Serial.println("Internal ESP-NOW error.");
        // } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
        //     Serial.println("Out of memory.");
        // } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
        //     Serial.println("Peer not found.");
        // } else if (result == ESP_ERR_ESPNOW_IF) {
        //     Serial.println("Interface error.");
        // } else {
        //     Serial.println("Unknown error.");
        // }
    }

    void receiveData(uint8_t receivedData[MAX_DATA_SIZE], int& length)  override {
        // uint8_t new_data_temp[length] = new_data;
        // Copy the received new_data into the parameter receivedData
        length = new_data_len;
        int safeLength = min(length, MAX_DATA_SIZE);
        for(int i = 0; i < safeLength; ++i) { //TODO use memcpy instead?
            receivedData[i] = new_data[i];
        }
        new_data_received = false;

    }

    bool newData() override{
        return new_data_received;
    }



    void addPeer(const uint8_t *peerAddr) {

        esp_now_peer_info_t peerInfo;
        memset(&peerInfo, 0, sizeof(peerInfo)); // Initialize peerInfo structure to zero
        memcpy(peerInfo.peer_addr, peerAddr, 6); 
        peerInfo.channel = 0; // Use auto channel
        peerInfo.encrypt = false; // No encryption
        for (int i = 0; i < 6; ++i) {
            if (i > 0) {
                Serial.print(":");
            }
            // Print each byte in hexadecimal format
            if (peerInfo.peer_addr[i] < 16) {
                Serial.print("0"); // Print a leading zero for values less than 0x10
            }
            Serial.print(peerInfo.peer_addr[i], HEX);
        }
        Serial.println();
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("Failed to add peer");
        } else {
            Serial.print("Added peer successfully.");
        }
    }

    void removePeer(const uint8_t *peerAddr) {
        if (esp_now_del_peer(peerAddr) != ESP_OK) {
            Serial.println("Failed to remove peer");
        }
    }


//    ReceivedData receiveShortData()  override {
//        esp_sensor_ready = false;
//        return ESPNOW_ReceivedData;
//    }
//
//
//    bool newShortData() override{
//        return esp_sensor_ready;
//    }

};






#endif // ESPNOWCOMM_H