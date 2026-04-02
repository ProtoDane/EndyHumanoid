#ifndef ACTIONS_H
#define ACTIONS_H

#include <Bluepad32.h>

#include "servoHandler.h"
#include "kinematics_solver.h"
#include "systemParams.h"

class actionHandler {
    public:
        actionHandler();

        void begin(servoHandler *servo);

        void moveWalkFwd(ControllerPtr gamepad);
        void moveStrafe(ControllerPtr gamepad, bool dir);
        void moveSpin(ControllerPtr gamepad, bool dir);

        void actionIdle();
        void actionCrouch(ControllerPtr gamepad);

    private:
        servoHandler *_servo;
};

#endif