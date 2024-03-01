//
// Created by dav on 2/1/24.
//

#ifndef BLIMPSWARM_PARAMMANAGER_H
#define BLIMPSWARM_PARAMMANAGER_H


#include <Preferences.h>
#include <Arduino.h>
#include <float.h>
#include "util/data_types.h"

// Other robot types would be included here

class ParamManager {
    private:
        Preferences preferences;

        String readString(const uint8_t *data, int &index, uint8_t length);
    public:
        void parseAndSetPreference(const uint8_t* data, int data_len);
        void setGroundMac(const uint8_t mac_addr[6]);
};



#endif //BLIMPSWARM_PARAMMANAGER_H
