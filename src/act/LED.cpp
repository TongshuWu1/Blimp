//
// Created by dav on 2/9/24.
//

#include "LED.h"
#include <Arduino.h>

LED::LED(int minVal, int maxVal, int offsetVal, int pinVal): Actuator(minVal, maxVal, offsetVal, pinVal) {
    pinMode(pinVal, OUTPUT);
}

void LED::act(float value){
    int led = (value < -0.5)? LOW: HIGH;
    digitalWrite(LED_BUILTIN, led);
}