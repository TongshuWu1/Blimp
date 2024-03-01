//
// Created by dav on 1/20/24.
//

#ifndef BLIMPSWARM_RAWBICOPTER_H
#define BLIMPSWARM_RAWBICOPTER_H


#include "Robot.h"
#include "act/BLMotor.h"
#include "act/AServo.h"
#include "act/LED.h"
#include <Preferences.h>

class RawBicopter : public Robot {
public:
    RawBicopter();
    int sense(float sensors[MAX_SENSORS]) override;
    bool actuate(const float actuators[], int size) override;
    void calibrate() override;
    //void testActuators(float actuationCmd[4]) override;

private:
    void getPreferences() override;

    BLMotor* motor1;
    BLMotor* motor2;
    AServo* servo1;
    AServo* servo2;
    LED* led;
};


#endif //BLIMPSWARM_RAWBICOPTER_H
