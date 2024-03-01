//
// Created by dav on 2/9/24.
//

#ifndef BLIMPSWARM_LED_H
#define BLIMPSWARM_LED_H

#include "Actuator.h"

class LED : public Actuator{
public:
    LED(int minVal, int maxVal, int offsetVal, int pinVal);

    void act(float value);
};


#endif //BLIMPSWARM_LED_H
