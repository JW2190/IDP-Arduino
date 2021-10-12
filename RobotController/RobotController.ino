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
<<<<<<< HEAD

=======
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
>>>>>>> 6735c0c6bce2fa3889dc6c9cd0dc509b9f05ae76

void setup() {
  
  Serial.begin(9600);//Start serial and set the correct Baud Rate
  Serial.println(first);


  
  //DC Motor setup:
  AFMS.begin();
<<<<<<< HEAD
  

}

void loop() {
  // put your main code here, to run repeatedly:
  //Test: Set motor speed and direction
  myMotor->setSpeed(150); //can be between 0 and 255
  myMotor->run(FORWARD);
  delay(2000);
  Serial.print("Everything working properly");
=======
}

void loop() {
  uint8_t i;

  if(first){
    first=false;
    myMotor->setSpeed(255);
    myMotor2->setSpeed(255);  
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
>>>>>>> 6735c0c6bce2fa3889dc6c9cd0dc509b9f05ae76
}
