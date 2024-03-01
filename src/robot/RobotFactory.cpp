//
// Created by dav on 1/20/24.
//


#include "RobotFactory.h"

// Include robots
#include "RawBicopter.h"


Robot* RobotFactory::createRobot(const String& type) {
    if (type== "RawBicopter") {
        return new RawBicopter();
    }
        // Add cases for other robot types here
    else {
        return nullptr;
    }
}