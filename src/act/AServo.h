//
// Created by dav on 2/9/24.
//

#ifndef BLIMPSWARM_ASERVO_H
#define BLIMPSWARM_ASERVO_H

#include "Actuator.h"
#include <ESP32Servo.h>

class AServo : public Actuator{
public:
    AServo(int minVal, int maxVal, int offsetVal, int pinVal);

    void act(float value);

private:
    Servo servo;
    int servo_min = 550;   //FIXME set it somewhere else
    int servo_max = 2450;  //FIXME set it somewhere else
};


#endif //BLIMPSWARM_ASERVO_H
