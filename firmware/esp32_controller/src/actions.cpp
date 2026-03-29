#include "actions.h"

actionHandler::actionHandler() {

}

void actionHandler::begin(servoHandler *servo) {
    _servo = servo;
}

// Walk forward sequence
void actionHandler::moveWalkFwd(ControllerPtr gamepad) {
  
    // Trajectory parameters
    double z0 = 60.0, nZ = 35.0 * 1, pZ = 5.0;  // z0: initial standing height | nZ: vertical up distance | pZ: vertical down distance
    double y0 = 3.0, dY = 15.0; // 3 10              // y0: initial sideways offset | dY: sideways foot amplitude
    double x0 = -0.0,  dX = 10.0;             // x0: initial front/back foot distance | dX: step amplitude
    double dT = 15.0, dA = 10.0;              // dT: torso angular amplitude | dA: shoulder joint amplitude

    int M = 28; // Total step cycle points 32
    int N = 8; // Leg lifting points (subset of M)

    // Initial sequence steps
    for (int i = 0; i < 6; i++) {
        
        legAngles l;
        armAngles a = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true};
        ik_leg(&l, 
        x0 + dX * i / 6, 
        y0 + dY * sin(i * PI / 6), 
        z0 - nZ * sin(i * PI / 6),  
        x0 - dX * i / 6, 
        y0 - dY * sin(i * PI / 6), 
        z0 + pZ * sin(i * PI / 6)
        );    

        // Perform sanity check to make sure IK calculation was successful
        if (l.success) {
            _servo->setServoCluster(&l, &a, dT * sin(i * PI / 12));
        } else {
            return;
        }

        BP32.update();
        delay(15);  // 25

        // Escape condition if the stick is no longer held
        if (gamepad->axisY() >= -AXIS_THRESHOLD && gamepad->axisRY() >= -AXIS_THRESHOLD) {
            return;
        }
    }

    // Continuous sequence
    int m = 0;
    int n = 0;
    while (gamepad->axisY() < -AXIS_THRESHOLD || gamepad->axisRY() < -AXIS_THRESHOLD) {
        

        legAngles l;
        armAngles a = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true};

        // Serial.printf("m|%d n|%d\n", m, n);

        ik_leg(&l, 
        (n % N < N/2) ? (x0 + dX * (1 - (n%(N/2))/(N/4))):(x0 + dX * ((n%(N/2))/(N/4) - 1)), 
        y0 - dY * sin(PI * m / (M/2)), 
        (n % N < N/2) ? (z0 + pZ * sin(PI * n / (N/2))):(z0 + nZ * sin(PI * n / (N/2)) * 1), 
        
        (n % N < N/2) ? (x0 + dX * ((n%(N/2))/(N/4) - 1)):(x0 + dX * (1 - (n%(N/2))/(N/4))), 
        y0 + dY * sin(PI * m / (M/2)), 
        (n % N < N/2) ? (z0 - nZ * sin(PI * n / (N/2))):(z0 - pZ * sin(PI * n / (N/2)) * 1)
        );    

        // ik_leg(&l, 
        // (i%12 < 6) ? (x0 + dX * (1 - (i%6)/3)):(x0 + dX * ((i%6)/3 - 1)), 
        // y0 - dY * sin(PI * i / 18), 
        // (i%12 < 6) ? (z0 + pZ * sin(PI * (i%6) / 6) * 1):(z0 - nZ * sin(PI * (i%6) / 6) * 1), 
        // (i%12 < 6) ? (x0 + dX * ((i%6)/3 - 1)):(x0 + dX * (1 - (i%6)/3)), 
        // y0 + dY * sin(PI * i / 18), 
        // (i%12 < 6) ? (z0 - nZ * sin(PI * (i%6) / 6) * 1):(z0 + pZ * sin(PI * (i%6) / 6) * 1)
        // );    

        // Perform sanity check to make sure IK calculation was successful
        if (l.success) {
            _servo->setServoCluster(&l, &a, dT * cos(m * PI / (M/2)));
        } else {
            return;
        }

        m++;

        if (abs((m % M) - M/4) < N/4 || abs((m % M) - 3*M/4) < N/4 || (m%M) == M/4 + N/4 || (m%M) == 3*M/4 + N/4) {
            n++;
        }
        
        BP32.update();
        delay(15); // 20 5
    }
}

void actionHandler::actionIdle() {
    legAngles idleLegs = {15, -30, 30, 15, -15, 30, -30, -15, true};
    _servo->setServoCluster(&idleLegs, 0);
}

void actionHandler::actionCrouch(ControllerPtr gamepad) {
    legAngles crouchLegs = {30, -75, 75, 30, -30, 75, -75, -30, true};
    _servo->setServoCluster(&crouchLegs, 0);

    while(gamepad->a()) {BP32.update(); delay(50);}
}