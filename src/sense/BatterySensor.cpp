//
// Created by dav on 2/10/24.
//

#include "BatterySensor.h"
#include <Arduino.h>


BatterySensor::BatterySensor(int minVal, int maxVal, int pinVal)
        : Sensor(minVal, maxVal, pinVal) {

    pinMode(pinVal, INPUT);
}

float BatterySensor::sense() {

    
}