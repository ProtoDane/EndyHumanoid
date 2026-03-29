#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

TwoWire i2c = TwoWire(0);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, i2c);

const int NUM_SERVOS = 16;
const int SERVO_MIN = 500;
const int SERVO_MAX = 2500;

enum State {calibInit, calibMid, calibFirst, calibLast, calibEnd};
enum State currentState = calibInit;
int servoIndex = 0;

int calibFirstVal;
int calibLastVal;
int calibNeutralVal;

int setCalibratedServo(int i, int value, int neutral, int high, int low) {

  int pulse;
  
  if(value > 0) {

    pulse = map(value, 0, 90, neutral, high);
    
  } else if(value < 0){

    pulse = map(value, -90, 0, low, neutral);
    
  } else {
    
    pulse = neutral;
  
  }

  pwm.writeMicroseconds(i, pulse);
  return pulse;
}

void actuateCalibratedServo(int i, int neutral, int high, int low) {

  char receivedChar = '0';
  int servoPulse = neutral;
  int servoAngle = 0;
  Serial.println("Setting servo to neutral position (" + String(neutral) + "us)...");
  pwm.writeMicroseconds(i, servoPulse);

  Serial.println("Controls:");
  Serial.println("Q/A: +/- 10 deg");
  Serial.println("W/S: +/-  5 deg");
  Serial.println("E/D: +/-  1 deg");
  Serial.println("R: Reset to neutral");
  Serial.println("SPACE: Quit");

  while(receivedChar != ' ') {

    while(!Serial.available()){;}
    receivedChar = Serial.read();

    switch(receivedChar) {
      case 'q':
        servoAngle += 10;
        break;
        
      case 'a':
        servoAngle += -10;
        break;
        
      case 'w':
        servoAngle += 5;
        break;
        
      case 's':
        servoAngle += -5;
        break;
        
      case 'e':
        servoAngle += 1;
        break;
        
      case 'd':
        servoAngle += -1;
        break;
        
      case 'r':
        Serial.println("Resetting to neutral position (" + String(neutral) + "us)...");
        pwm.writeMicroseconds(i, neutral);
        servoPulse = neutral;
        break;
        
      case ' ':
        break;
        
      default:
        Serial.println("Unrecognized character!"); 
    }

    if (receivedChar != 'r' && receivedChar != ' '){

      servoAngle = constrain(servoAngle, -90, 90);
      servoPulse = setCalibratedServo(i, servoAngle, neutral, high, low);
      Serial.print("Current angle: " + String(servoAngle) + "deg ");
      Serial.println("| Corresponding pulse: " + String(servoPulse) + "us");
    }
  }
  
}

int calibrateServo(int i) {
  
  char receivedChar  = '0';
  int servoPulse = 1500;
  Serial.println("Setting servo to neutral position (1500us)...");
  pwm.writeMicroseconds(i, servoPulse);

  while(receivedChar != ' ') {

    while(!Serial.available()){;}
    
    receivedChar = Serial.read();
    
    switch(receivedChar){
      case 'q':
        servoPulse += 100;
        break;
        
      case 'a':
        servoPulse += -100;
        break;
        
      case 'w':
        servoPulse += 10;
        break;
        
      case 's':
        servoPulse += -10;
        break;
        
      case 'e':
        servoPulse += 1;
        break;
        
      case 'd':
        servoPulse += -1;
        break;
        
      case 'r':
        Serial.println("Resetting to neutral position (1500us)");
        pwm.writeMicroseconds(i, 1500);
        servoPulse = 1500;
        break;
        
      case ' ':
        break;
        
      default:
        Serial.println("Unrecognized character!");        
    }

    if (receivedChar != 'r' || receivedChar != ' '){

      servoPulse = constrain(servoPulse, SERVO_MIN, SERVO_MAX);
      pwm.writeMicroseconds(i, servoPulse);
      Serial.println("Current servo pulse (us): " + String(servoPulse));
    }
  }

  return servoPulse;
}


void setup() {
    Serial.begin(115200);
    while (!Serial) {;} // Hold until serial port is connected

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

    

    // Move all servos to neutral position (1500us pulse width)
    for (int i = 0; i < NUM_SERVOS; i++) {
        pwm.writeMicroseconds(i, 1500);
    }
}

void loop() {
    switch(currentState){
        case calibInit: {
            // Display controls and hold until the user presses any key
            Serial.println("Controls:");
            Serial.println("Q/A: +/- 100 us");
            Serial.println("W/S: +/- 10 us");
            Serial.println("E/D: +/- 1 us");
            Serial.println("R: Reset to neutral");
            Serial.println("Z/X: Cycle through servos");
            Serial.println("SPACE: Confirm calibration value");
            Serial.println("\nPress any key to continue...");
            
            while(!Serial.available()) {;}
            char input = Serial.read();

            while (input == 'z' || input == 'x' || input == ' ') {
                if (input == 'z') {
                    servoIndex = max(0, servoIndex - 1);
                } else if (input == 'x') {
                    servoIndex = (servoIndex + 1) % NUM_SERVOS;
                } else {
                    break;
                }
                Serial.println("Selected Servo: " + String(servoIndex));
                Serial.println("Press Z to select previous servo, X to select next servo, or SPACE to confirm selection.");
                while(!Serial.available()) {;}
                input = Serial.read();
            }

            currentState = calibMid;
            break;
        }

        case calibMid: {

            // Prompt user to calibrate the neutral position (around 1500 usec for DS3235 servos)
            Serial.println("\nCalibrate the servo to the neutral position");
            int calibNeutralVal = calibrateServo(servoIndex);
            Serial.println("Neutral position recorded at " + String(calibNeutralVal));
            currentState = calibFirst;
            break;
        }

        case calibFirst: {

            // Prompt user to calibrate the max value at +90 (or +135 for 270 degree servos), which is counter-clockwise from neutral
            Serial.println("\nCalibrate the servo to +90 degrees from the neutral position (counter-clockwise)");
            int calibFirstVal = calibrateServo(servoIndex);
            Serial.println("+90 degrees recorded at " + String(calibFirstVal));
            currentState = calibLast;
            break;
        }

        case calibLast: {

            // Prompt user to calibrate the min value at -90 (-135 for 270 degree servos), which is clockwise from neutral
            Serial.println("\nCalibrate the servo to -90 degrees from the neutral position (clockwise)");
            calibLastVal = calibrateServo(servoIndex);
            Serial.println("-90 degrees recorded at " + String(calibLastVal));
            currentState = calibEnd;                  
            break;
        }

        case calibEnd: {

            Serial.println("\nStarting user control of calibrated servo...");
            actuateCalibratedServo(servoIndex, calibNeutralVal, calibFirstVal, calibLastVal);
            Serial.println("\nPrinting results, be sure to save this somewhere...");
            Serial.println("Calibrated neutral value      (us): " + String(calibNeutralVal));
            Serial.println("+90 degrees calibration value (us): " + String(calibFirstVal));
            Serial.println("-90 degrees calibration value (us): " + String(calibLastVal));
            Serial.println("Resetting the terminal...\n");
            currentState = calibInit;
            break;
        }
    }
}