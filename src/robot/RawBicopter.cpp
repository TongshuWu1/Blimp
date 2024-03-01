//
// Created by dav on 1/20/24.
//


#include "RawBicopter.h"
#include <Arduino.h>


#define SERVO1 D0
#define SERVO2 D1
#define THRUST1 D9
#define THRUST2 D10
#define BATT A2


RawBicopter::RawBicopter(){

    servo1 = new AServo(0, 1, 0, SERVO1);
    servo2 = new AServo(0, 1, 0, SERVO2);
    motor1 = new BLMotor(0, 1, 0, THRUST1, 55);
    motor2 = new BLMotor(0, 1, 0, THRUST2, 58);
    // On board LED light
    led = new LED(0, 1, 0, LED_BUILTIN);


    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    Preferences preferences;
    
    preferences.begin("params", false); //true means read-only
    if (preferences.getBool("calibrate", false)){
        //calibrate brushless motors
        calibrate();
        preferences.putBool("calibrate", false);
    }
    else {
        // Arm brushless motors
        motor1->arm();
        motor2->arm();
    }
    preferences.end(); //true means read-only
}

int RawBicopter::sense(float sensors[MAX_SENSORS]) {
    // Implementation for sensing - fill the sensors array
    // Return the number of sensors used
    return 0; // Placeholder return value
}

bool RawBicopter::actuate(const float actuators[], int size) {
    servo1->act(actuators[2]);
    servo2->act(actuators[3]);
    motor1->act(actuators[0]);
    motor2->act(actuators[1]);
    led->act(actuators[4]);

    return true;
}

void RawBicopter::getPreferences() {
    
    // Implementation for reading values from non-volatile storage (NVS)
    // must manually enter keys and default values for every variable.
    Preferences preferences; //initialize the preferences 
    preferences.begin("params", true); //true means read-only

    //value = preferences.getInt("value", default_value); //(value is an int) (default_value is manually set)
    

    preferences.end();
}

void RawBicopter::calibrate(){
//    motor1->calibrate();
//    motor2->calibrate();


    delay(1000);
    Serial.println("Calibrating ESCs....");
    // ESC arming sequence for BLHeli S
    motor1->act(1);
    motor2->act(1);
    delay(8000);

    // Back to minimum value
    motor1->act(0);
    motor2->act(0);
    delay(8000);
    //motor1->act(-1); //FIXME is this necessary? If so, we can have a if (value== -1) then writeMicroseconds(0) in motor
    delay(1000);
    Serial.println("Calibration completed");
}
//void RawBicopter::testActuators(float actuationCmd[4]) {
//    int servo_delta = 1;
//    int motor_delta = 10;
//
//    if (actuationCmd[0] < 180) {
//        actuationCmd[0] += servo_delta;
//    } else if (actuationCmd[1] < 180) {
//        actuationCmd[1] += servo_delta;
//    } else if (actuationCmd[2] < 2000) {
//        actuationCmd[2] += motor_delta;
//    } else if (actuationCmd[3] < 2000) {
//        actuationCmd[3] += motor_delta;
//
//}