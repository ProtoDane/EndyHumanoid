#include "actions.h"

actionHandler::actionHandler() {

}

void actionHandler::begin(servoHandler *servo) {
    _servo = servo;
}

// Walk forward sequence
void actionHandler::moveWalkFwd(ControllerPtr gamepad) {
  
    // Trajectory parameters
    double z0 = 60.0, nZ = 35.0 * 1, pZ = 1.0;  // z0: initial standing height | nZ: vertical up distance | pZ: vertical down distance
    double y0 = 3.0, dY = 15.0; // 3 10              // y0: initial sideways offset | dY: sideways foot amplitude
    double x0 = -0.0,  dX = 10.0;             // x0: initial front/back foot distance | dX: step amplitude
    double dT = 15.0, dA = 10.0;              // dT: torso angular amplitude | dA: shoulder joint amplitude

    int M = 28; // Total step cycle points 32
    int N = 8; // Leg lifting points (subset of M)

    // Continuous sequence
    int m = 0;
    int n = 0;
    bool initialTakeoff = true;
    while (gamepad->axisY() < -AXIS_THRESHOLD || gamepad->axisRY() < -AXIS_THRESHOLD) {
        

        legAngles l;
        armAngles a = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true};

        Serial.printf("m|%d n|%d\n", m, n);
        if (initialTakeoff) {
            ik_leg(&l, 
            
            // (n % N < N/2) ? (x0 + dX * (1 - (n%(N/2))/(N/4))):(x0 + dX * ((n%(N/2))/(N/4) - 1)), 
            fmin(x0 + dX * n / (N/4), x0 + dX),
            // x0 + dX * (n%(N/2))/(N/4),
            y0 + dY * sin(PI * m / (M/2)), 
            // z0 - nZ * sin(PI * n / (N/2)),
            (n % N < N/2) ? (z0 - nZ * sin(PI * n / (N/2))):(z0 - pZ * sin(PI * n / (N/2))), 
            
            //(n % N < N/2) ? (x0 + dX * ((n%(N/2))/(N/4) - 1)):(x0 + dX * (1 - (n%(N/2))/(N/4))), 
            fmax(x0 - dX * n / (N/4), x0 - dX),
            // x0 - dX * (n%(N/2))/(N/4),
            y0 - dY * sin(PI * m / (M/2)),
            // z0 + pZ * sin(PI * n / (N/2)) 
            (n % N < N/2) ? (z0 + pZ * sin(PI * n / (N/2))):(z0 + nZ * sin(PI * n / (N/2)))
            );
        } else {
            ik_leg(&l, 
            (n % N < N/2) ? (x0 + dX * (1 - (n%(N/2))/(N/4))):(x0 + dX * ((n%(N/2))/(N/4) - 1)), 
            y0 - dY * sin(PI * m / (M/2)), 
            (n % N < N/2) ? (z0 + pZ * sin(PI * n / (N/2))):(z0 + nZ * sin(PI * n / (N/2))), 
            
            (n % N < N/2) ? (x0 + dX * ((n%(N/2))/(N/4) - 1)):(x0 + dX * (1 - (n%(N/2))/(N/4))), 
            y0 + dY * sin(PI * m / (M/2)), 
            (n % N < N/2) ? (z0 - nZ * sin(PI * n / (N/2))):(z0 - pZ * sin(PI * n / (N/2)))
            );    
        } 

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
        
        if (initialTakeoff && m == M/2) {
            initialTakeoff = false;
            m = 0;
            n = 0;
        }

        BP32.update();
        delay(15); // 20 5
    }
}

void actionHandler::moveStrafe(ControllerPtr gamepad, bool dir) {

    double x0 = 10.0;
    double r0 = 50.0;
    double dr = 30.0;
    double th0 = 5.0;
    double dth1 = 10.0; // 10
    double dth2 = 20.0;
    
    int i = 0;
    while ( (dir & gamepad->axisRX() < -AXIS_THRESHOLD) || (!dir & gamepad->axisRX() > AXIS_THRESHOLD) ) {

        legAngles l;
        armAngles a = {90.0, 60.0, 20.0, -90.0, -60.0, -20.0};

        if (dir) { // strafe left
            ik_polar(&l,
                r0,
                x0,
                th0 + dth1 * sin((i%12) * PI / 12),
                r0 + dr * sin((i%12) * PI / 12),
                x0,
                th0 + dth2 * sin((i%12) * PI / 12)
            );

            
        } else { // strafe right
            ik_polar(&l,
                r0  + dr * sin((i%12) * PI / 12),
                x0,
                th0 + dth2 * sin((i%12) * PI / 12),
                r0,
                x0,
                th0 + dth1 * sin((i%12) * PI / 12)
            );
        }

        if (l.success) {
            _servo->setServoCluster(&l, &a, (dir ? 15.0 : NULL), (dir ? NULL : -15.0), 0.0);
            // _servo->setServoCluster(&l, &a, 0.0);
        } else {
            return;
        }

        i++;
        BP32.update();
        delay(10);
    }
}

void actionHandler::moveSpin(ControllerPtr gamepad, bool dir) {
    double x0 = 10.0;
    double z0 = 50.0;
    double dZ = 30.0;
    double y0 = 3.0;
    double dT = 20.0;

    int i = 0;
    while ( (dir & gamepad->axisX() < -AXIS_THRESHOLD) || (!dir & gamepad->axisX() > AXIS_THRESHOLD) ) {
        
        legAngles l;
        armAngles a ={90.0, 60.0, 20.0, -90.0, -60.0, -20.0};
        
        ik_leg(&l,
            x0, y0, z0 - dZ * sin(PI * (i%6) /  6),
            x0, y0, z0 - dZ * sin(PI * (i%6) /  6)
        );

        if (l.success) {
            _servo->setServoCluster(&l, &a, dT * sin((i) * (dir ? -1 : 1) *  PI / 3));
        } else {
            return;
        }

        i++;
        BP32.update();
        delay(40);
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