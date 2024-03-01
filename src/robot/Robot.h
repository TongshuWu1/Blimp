//
// Created by dav on 1/20/24.
//

#ifndef BLIMPSWARM_ROBOT_H
#define BLIMPSWARM_ROBOT_H




class Robot {
private:
    // Pure virtual function for variable settings
    // reads from Preferences library to initialize variables
    virtual void getPreferences();

public:
    // Assume a fixed maximum size for the arrays
    static const int MAX_SENSORS = 10;

    // Pure virtual function for sensing
    // Returns the number of elements filled in the array
    virtual int sense(float sensors[MAX_SENSORS]) = 0;

    // Pure virtual function for actuate
    // Takes an array of actuator commands and its size
    virtual bool actuate(const float actuators[], int size) = 0;

    virtual void calibrate();


    virtual ~Robot() {}
};



#endif
