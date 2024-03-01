//
// Created by dav on 2/9/24.
//

#ifndef BLIMPSWARM_BLMOTOR_H
#define BLIMPSWARM_BLMOTOR_H

#include "Actuator.h"
#include <ESP32Servo.h>

class BLMotor : public Actuator {
public:
    BLMotor(int minVal, int maxVal, int offsetVal, int pinVal, int periodHertz);

    void calibrate();
    void arm();

    void act(float value);

private:
    Servo thrust;
    int min_thrust = 0;  // Force in grams  //TODO convert this into variables that can be loaded from the flash memory
    int max_thrust = 35;  // Force in grams  //TODO convert this into variables that can be loaded from the flash memory

    // Constants to convert PWM to Force. Equation: Force = a * PWM + b
    const float pwm_a = 0.042337045;  //TODO convert this into variables that can be loaded from the flash memory
    const float pwm_b = -46.58244237; //TODO convert this into variables that can be loaded from the flash memory
};


#endif //BLIMPSWARM_BLMOTOR_H
