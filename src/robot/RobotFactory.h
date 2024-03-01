//
// Created by dav on 1/20/24.
//

#ifndef BLIMPSWARM_ROBOTFACTORY_H
#define BLIMPSWARM_ROBOTFACTORY_H


#include "Arduino.h"
#include "Robot.h"
// Other robot types would be included here

class RobotFactory {
public:
    static Robot* createRobot(const String& type);
};



#endif //BLIMPSWARM_ROBOTFACTORY_H
