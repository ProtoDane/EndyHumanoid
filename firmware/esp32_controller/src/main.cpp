#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "servoHandler.h"

TwoWire i2c = TwoWire(0);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, i2c);

servoHandler servo = servoHandler(pwm);

// put function declarations here:
// int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
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

  delay(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  // for (int i = 102; i <= 512; i++) {
  //   pwm.setPWM(0, 0, i);
  //   delay(3);
  // }

  // for (int i = 512; i >= 102; i--) {
  //   pwm.setPWM(0, 0, i);
  //   delay(3);
  // }
  pwm.writeMicroseconds(0, 1500); // Move to 0 degrees
  pwm.writeMicroseconds(1, 1500);
  pwm.writeMicroseconds(2, 1500);
  pwm.writeMicroseconds(3, 1500);
  pwm.writeMicroseconds(4, 1500);
  pwm.writeMicroseconds(5, 1500);
  pwm.writeMicroseconds(6, 1500);
  pwm.writeMicroseconds(7, 1500);
  pwm.writeMicroseconds(8, 1500);
  pwm.writeMicroseconds(9, 1500);
  pwm.writeMicroseconds(10, 1500);
  pwm.writeMicroseconds(11, 1500);
  pwm.writeMicroseconds(12, 1500);
  pwm.writeMicroseconds(13, 1500);
  pwm.writeMicroseconds(14, 1500);
  pwm.writeMicroseconds(15, 1500);
  delay(1000);
  // pwm.writeMicroseconds(0, 2500); // Move to 180 degrees
  delay(1000);
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 50;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}