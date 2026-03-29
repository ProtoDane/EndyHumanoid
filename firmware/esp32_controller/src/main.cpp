#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Bluepad32.h>

#include "servoHandler.h"
#include "actions.h"
#include "systemParams.h"

TwoWire i2c = TwoWire(0);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, i2c);

servoHandler servo = servoHandler(&pwm);

ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

actionHandler action = actionHandler();

// ==========================================================================
// Bluepad32 Handler Functions
// ==========================================================================

// Handler for controller connected event
void onConnectedController(ControllerPtr ctl) {
  
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        
        if (myControllers[i] == nullptr) {
        
        myControllers[i] = ctl;
        foundEmptySlot = true;

        // Optional, once the gamepad is connected, request further info about the
        // gamepad.
        ControllerProperties properties = ctl->getProperties();
        char buf[80];
        sprintf(buf,
            "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
            "flags: 0x%02x",
            properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
            properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
            properties.vendor_id, properties.product_id, properties.flags);
        Serial.println(buf);

        break;
        }
    }
  
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

// Handler for controller disconnected event
void onDisconnectedController(ControllerPtr ctl) {
    bool foundGamepad = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        
        if (myControllers[i] == ctl) {
        // Serial.print("CALLBACK: Controller is disconnected from index=");
        // Serial.println(i);
        myControllers[i] = nullptr;
        foundGamepad = true;
        break;
        }
    }

    if (!foundGamepad) {
        // Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void gamepadProcessor(ControllerPtr gamepad) {
    
    int lx = gamepad->axisX();
    int ly = gamepad->axisY();
    int rx = gamepad->axisRX();
    int ry = gamepad->axisRY();
    
    if (gamepad->a()) {
        action.actionCrouch(gamepad);
    } else if (ly < -AXIS_THRESHOLD || ry < -AXIS_THRESHOLD) {
        action.moveWalkFwd(gamepad);
    } else {
        action.actionIdle();
    }
}

// ==========================================================================
// Arduino Super Loop
// ==========================================================================

void setup() {
    
    delay(2000);
    
    Serial.begin(115200);
    i2c.begin(21, 22);

    if (!pwm.begin()) {
        while (1) {
        Serial.println("Error initializing PCA9685");
        delay(1000);
        }
    }
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50);

    action.begin(&servo);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();

    delay(10);
}

void loop() {

    // legAngles idlePose = {15, -30, 30, 15, -15, 30, -30, -15, true};
    // servo.setServoCluster(&idlePose, 0.0);
    // delay(1000);

    BP32.update();

    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        ControllerPtr gamepad = myControllers[i];
        if (gamepad != nullptr) {
            gamepadProcessor(gamepad);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}