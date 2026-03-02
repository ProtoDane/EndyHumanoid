#include "servoHandler.h"

servoHandler::servoHandler(Adafruit_PWMServoDriver *pwm) {
    _pwm = pwm;

}

// Public Functions

void servoHandler::setServoCluster(legAngles *l, float torsoAngle) {
    float angles[9] = {
        torsoAngle,
        l->lth1, l->lth2, l->lth3, l->lth4,
        l->rth1, l->rth2, l->rth3, l->rth4,
    };
    setServoCluster(angles, LOWER_BODY);
}

void servoHandler::setServoCluster(legAngles *l, armAngles *a, float torsoAngle) {
    float angles[16] = {
        torsoAngle, a->la1, a->la2, a->la3, 
        l->lth1, l->lth2, l->lth3, l->lth4,
        l->rth1, l->rth2, l->rth3, l->rth4,
        a->ra1, a->ra2, a->ra3,
    };
    setServoCluster(angles, ALL_SERVOS);
}

void servoHandler::setServoCluster(const float *angles, int pinMask) {

    int pIndex = 0;
    for (int i = 0; i < 16; i++) {
        if (pinMask & (1 << i)) {

            int mappedPulse = mapPulse(angles[pIndex], servoCluster[i].min, servoCluster[i].mid, servoCluster[i].max);
            setServo(i, mappedPulse);
            pIndex++;
        }
    }
}

// Private Functions
void servoHandler::setServo(int i, int us) {
    _pwm->writeMicroseconds(i, us);
}

int servoHandler::mapPulse(float angle, int min, int mid, int max) {

    float maxAngle = 90.0;

    if (angle > 0.0) {
        return (int) (angle * (max - mid) / maxAngle + mid);
    } else if (angle < 0.0) {
        return (int) ((angle + maxAngle) * (mid - min) / maxAngle + min);
    } else {
        return mid;
    }
}