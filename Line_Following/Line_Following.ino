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
const int sensor_pins[3] = {A0, A1, A2}


// Sensor Values Global
const int SENSOR_THRESHOLD = 15;
int sensor_state[2];

// Motor Speeds Global
const int HIGH_MOTOR_SPEED = 150;
const int LOW_MOTOR_SPEED = 75;

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
  
  for (byte i = 0; i < sizeof(sensor_pins); i++)
  {
    sensor_state[i] = analogRead(sensor_pins[i]);
    Serial.print(sensor_state[i]);
    Serial.print("/t");  
  }
  
  //If RIGHT SENSOR LOW (WHITE) and LEFT SENSOR HIGH (BLACK) turn LEFT
  if(sensor_state[1] < SENSOR_THRESHOLD && sensor_state[0] > SENSOR_THRESHOLD)
    {
      Serial.println("turning right");
      left_motor->run(FORWARD);
      left_motor->setSpeed(HIGH_MOTOR_SPEED);
      right_motor->run(FORWARD);
      right_motor->setSpeed(LOW_MOTOR_SPEED);
      delay(turn_delay);
    }

  //If RIGHT SENSOR HIGH (BLACK) and LEFT SENSOR LOW (WHITE) turn RIGHT
  if(sensor_state[1] > SENSOR_THRESHOLD && sensor_state[0] < SESNOR_THRESHOLD)
    {
      Serial.println("turning left");
      left_motor->run(FORWARD);
      left_motor->setSpeed(LOW_MOTOR_SPEED);
      right_motor->run(FORWARD);
      right_motor->setSpeed(HIGH_MOTOR_SPEED);
      delay(turn_delay);
     }
     
  //If RIGHT SENSOR HIGH (BLACK) and LEFT SENSOR HIGH (BLACK) stay STRAIGHT
  if(sensor_state[1] > SENSOR_THRESHOLD && sensor_state[0] > SENSOR_THRESHOLD)
  {
    Serial.println("going forward");
    left_motor->run(FORWARD);
    left_motor->setSpeed(HIGH_MOTOR_SPEED);
    right_motor->run(FORWARD);
    right_motor->setSpeed(HIGH_MOTOR_SPEED);
    delay(turn_delay);
  }

  //If RIGHT SENSOR LOW (WHITE) and LEFT SENSOR LOW (WHITE) STOP
  if(sensor_state[1] < SENSOR_THRESHOLD && sensor_state[0] < SENSOR_THRESHOLD)
  { 
    Serial.println("stop");
    left_motor->(RELEASE);
    right_motor->(RELEASE);
  }





  
}
