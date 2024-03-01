//
// Created by dav on 2/10/24.
//

#ifndef BLIMPSWARM_SENSOR_H
#define BLIMPSWARM_SENSOR_H


class Sensor {
public:
    Sensor(int minVal, int maxVal, int pinVal)
            : min(minVal), max(maxVal), pin(pinVal){
    }

    virtual float sense();
private:
    int min;
    int max;
    int pin;
};


#endif //BLIMPSWARM_SENSOR_H
