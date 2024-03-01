//
// Created by dav on 2/10/24.
//

#ifndef BLIMPSWARM_BATTERYSENSOR_H
#define BLIMPSWARM_BATTERYSENSOR_H

#include "Sensor.h"

class BatterySensor: public Sensor {
public:
    BatterySensor(int minVal, int maxVal, int pinVal);
    float sense() override;
};


#endif //BLIMPSWARM_BATTERYSENSOR_H
