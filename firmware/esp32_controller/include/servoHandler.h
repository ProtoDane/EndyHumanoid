#ifndef SHANDLER_H
#define SHANDLER_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#include "config.h"

struct legAngles {
    double lth1;
    double lth2;
    double lth3;
    double lth4;
    double rth1;
    double rth2;
    double rth3;
    double rth4;
    bool success;
};

struct armAngles {
    double la1;
    double la2;
    double la3;
    double ra1;
    double ra2;
    double ra3;
    bool success;
};

class servoHandler {

    public:
        servoHandler(Adafruit_PWMServoDriver& pwm);

        void setServoCluster(legAngles *l, armAngles *a);
        void setServoDelay(legAngles *l, armAngles *a, int delayMs);
    
    private:
        void setServo(int i, int us);
        int mapPulse(float angle, int min, int mid, int max);



};

#endif