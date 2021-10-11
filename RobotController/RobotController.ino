//inital important notes:
//when doing follow the line code, do not update motor speeds to same value as they currently were. can strain the network
//https://wokwi.com/arduino/new?template=arduino-uno

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//Motor Objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);


void setup() {
  //DC Motor setup:
  AFMS.begin();
  

}

void loop() {
  // put your main code here, to run repeatedly:
  //Test: Set motor speed and direction
  myMotor->setSpeed(150); //can be between 0 and 255
  myMotor->run(FORWARD);
  delay(2000);
  Serial.print("Everything working properly");
}
