/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->	http://www.adafruit.com/products/1438
*/

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);


//Sensor Connection
const int left_sensor_pin = A0;
const int right_sensor_pin = A1;

// Sensor Values Global

const int SENSOR_THRESHOLD = 500;
int left_sensor_state;
int right_sensor_state;

// Motor Speeds Global
const int HIGH_MOTOR_SPEED = 255;
const int LOW_MOTOR_SPEED = 0;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

}

void loop() {
  
  // Read State of Left Sensor from analog pin
  left_sensor_state = analogRead(left_sensor_pin);
  // Read State of Right Sensor from analog pin
  right_sensor_state = analogRead(right_sensor_pin);

  Serial.println(left_sensor_state);
  
//  // If RIGHT SENSOR HIGH (WHITE) and LEFT SENSOR LOW (BLACK) turn LEFT
//  if(right_sensor_state > SENSOR_THRESHOLD && left_sensor_state < SENSOR_THRESHOLD)
//    {
//      left_motor->run(FORWARD);
//      left_motor->setSpeed(HIGH_MOTOR_SPEED);
//      right_motor->run(FORWARD);
//      right_motor->setSpeed(LOW_MOTOR_SPEED);
//      Serial.println("turning right");
//
//    }
//
//
//  if(right_sensor_state < SENSOR_THRESHOLD && left_sensor_state > SESNOR_THRESHOLD)
//    {
//      Serial.println("turning left");
//      
//      digitalWrite (motorA1,HIGH);
//      digitalWrite(motorA2,LOW);                       
//      digitalWrite (motorB1,HIGH);
//      digitalWrite(motorB2,LOW);
//    
//      analogWrite (motorAspeed, turn_speed);
//      analogWrite (motorBspeed, vSpeed);
//    
//      delay(turn_delay);
//     }
//
//if(right_sensor_state > 500 && left_sensor_state > 500)
//{
//  Serial.println("going forward");
////  digitalWrite (motorA2,LOW);
////  digitalWrite(motorA1,HIGH);                       
////  digitalWrite (motorB2,HIGH);
////  digitalWrite(motorB1,LOW);
//
////  analogWrite (motorAspeed, vSpeed);
////  analogWrite (motorBspeed, vSpeed);
//
//  delay(turn_delay);
//  
//  }
//
//if(right_sensor_state < 500 && left_sensor_state < 500)
//{ 
//  Serial.println("stop");
//  
//  analogWrite (motorAspeed, 0);
//  analogWrite (motorBspeed, 0);
//  
//  }





  
}
