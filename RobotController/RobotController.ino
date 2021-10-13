//inital important notes:
//when doing follow the line code, do not update motor speeds to same value as they currently were. can strain the network
//https://wokwi.com/arduino/new?template=arduino-uno
//BACKWARD is forwards and FORWARD is backwards on model


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


bool first=true;

//Motor Objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

void setup() {
  
  Serial.begin(9600);//Start serial and set the correct Baud Rate
  Serial.println(first);


  
  //DC Motor setup:
  AFMS.begin();
}

void loop() {
  uint8_t i;

  if(first){
    first=false;
    myMotor->setSpeed(150);
    myMotor2->setSpeed(150);  
    myMotor->run(BACKWARD);
    myMotor2->run(BACKWARD);
    Serial.println("Set motor speed");
  }



  
  //testing the rotation of the robot
  
  
  
  /*
  Serial.println("tick");

  myMotor->run(BACKWARD);
  myMotor2->run(BACKWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);
    myMotor2->setSpeed(i);   
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);
    myMotor2->setSpeed(i);   
    delay(10);
  }
  
  Serial.println("tock");

  myMotor->run(BACKWARD);
  myMotor2->run(BACKWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);
    myMotor2->setSpeed(i);  
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);
    myMotor2->setSpeed(i);   
    delay(10);
  }

  Serial.println("tech");
  myMotor->run(RELEASE);
  myMotor2->run(RELEASE);
  delay(1000);

  */
}
