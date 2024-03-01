//
// Created by dav on 2/9/24.
//

#include "AServo.h"



AServo::AServo(int minVal, int maxVal, int offsetVal, int pinVal): Actuator(minVal, maxVal, offsetVal, pinVal) {
    // Additional initialization specific to SpecificActuator

    pinMode(pinVal, OUTPUT);



    servo.attach(pinVal, servo_min, servo_max);

//    this->thrust.setPeriodHertz(periodHertz);

    servo.setPeriodHertz(50); // Standard 50hz servo
}


void AServo::act(float value){
    //TODO the input should be force. We should use the calibration parameters
    //TODO the thrust is related to angular velocity
    int angle = constrain(value, 0, 180) ; // cant handle values between PI and 2PI
    servo.write((int) angle);
//    int val = constrain(value, 0, 1);
//    this->thrust.writeMicroseconds((int)((val) * (this->max - this->min) + this->min));
}