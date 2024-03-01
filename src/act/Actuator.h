//
// Created by dav on 2/9/24.
//

#ifndef BLIMPSWARM_ACTUATOR_H
#define BLIMPSWARM_ACTUATOR_H


class Actuator {
public:
    Actuator(int minVal, int maxVal, int offsetVal, int pinVal)
            : min(minVal), max(maxVal), offset(offsetVal), pin(pinVal){
    }

    // Setter method for offset
    void setOffset(int newOffset) {
        offset = newOffset;
    }

    virtual void act(float value) = 0;

protected:
    int min;
    int max;
    int offset;
    int pin;
};


#endif //BLIMPSWARM_ACTUATOR_H
