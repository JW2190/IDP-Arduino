/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->	http://www.adafruit.com/products/1438
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);


//Sensor Connection
const int sensor_pins [2] = {A0, A1};


// Sensor Values Global
float BLACK_TO_WHITE_MULTIPLIER = 0.80;
int SENSOR_THRESHOLD_L;
int SENSOR_THRESHOLD_R;
int sensor_state [2];
int sensor_stateR;
int sensor_stateL;

// Motor Speeds Global
const int HIGH_MOTOR_SPEED = 255;
const int LOW_MOTOR_SPEED = 150;

//turn delay
const int turn_delay = 5000;

//run 1 execution code in loop() (needed for slightly later start of motors)
bool first = true;

//Modes
enum mode_frame{LEFT,RIGHT,STRAIGHT,STOP};
mode_frame mode = STRAIGHT;
//DEBUG: Toggle functionality easily:
bool MotorsOn = true;
bool PRINT_SENSOR_STATES = true;
bool PRINT_TURNING_DECISIONS = true;



void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  
  //set motor toggle button
  pinMode(7, INPUT);
  
  AFMS.begin();


  left_motor->run(RELEASE);
  right_motor->run(RELEASE);

  
  //AUTO CALIBRATE SENSORS (Must be started on BLACK surface)
    delay(100);
    Serial.println("--------CALIBRATION-------");
    
    sensor_stateL = analogRead(sensor_pins[0]);
    sensor_stateR = analogRead(sensor_pins[1]);
    
    //Getting average of 10 sensor readings and multiplying by scale factor to guess white threshold
    for(byte i=0;i<10;i++){
      sensor_stateL = (sensor_stateL+analogRead(sensor_pins[0]))/2;
      sensor_stateR = (sensor_stateR+analogRead(sensor_pins[1]))/2;
      Serial.print("Left Sensor Black AVG: ");
      Serial.print(sensor_stateL);
      Serial.print("     Right Sensor Black AVG: "); 
      Serial.println(sensor_stateR);
      delay(100);
    }
    SENSOR_THRESHOLD_L = BLACK_TO_WHITE_MULTIPLIER * sensor_stateL;
    SENSOR_THRESHOLD_R = BLACK_TO_WHITE_MULTIPLIER * sensor_stateR;
    
    Serial.print("Left Sensor Threshold: ");
    Serial.println(SENSOR_THRESHOLD_L);
    Serial.print("Right Sensor Threshold: "); 
    Serial.println(SENSOR_THRESHOLD_R);
    Serial.println("--------------------------");
    
}

//------------------------------------------------------------------------------------------

void loop() {

  //READ SENSOR STATES
  sensor_stateL = analogRead(sensor_pins[0]);
  sensor_stateR = analogRead(sensor_pins[1]);
  if(PRINT_SENSOR_STATES){
    Serial.print("L: ");
    Serial.print(sensor_stateL);
    Serial.print("------");
    Serial.print("R: ");
    Serial.println(sensor_stateR);
  }
  
  if(first){ //might have to have this as running motor stuff in setup might be too early
    Serial.println("1 execution in 'loop' ran");
    first=false;
    
    //ENABLE MOTOR TURNING
    if(MotorsOn){
      left_motor->run(BACKWARD);
      right_motor->run(BACKWARD);
      }
  }

  //DEBUG: toggling motors on and off based on button input
  if(digitalRead(7)==1){
    MotorsOn = !MotorsOn;
  }
  if(MotorsOn){
    left_motor->run(BACKWARD);
    right_motor->run(BACKWARD);
  }
  else{
    left_motor->run(RELEASE);
    right_motor->run(RELEASE);
    }

  calc_mode();
  follow_line();
  delay(turn_delay);
}


//---------------------------------------------------------------------------------------------

void setmotorspeed(int L_speed,int R_speed){
  Serial.println("motor speeed set");
  left_motor->setSpeed(L_speed);
  right_motor->setSpeed(R_speed);
  
}

//--------------------------------------------------------------------------------

//PID function and objects:
float P ,I ,D ,PIDvalue, error, previousError,Kp, Ki, Kd;
void calculatePID(){
  P = error;
  I = I + error;
  D = error-previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;
  }


//--------------------------------------------------------------------------------

void follow_line(){
  switch(mode){
    case LEFT:
    if(PRINT_TURNING_DECISIONS){Serial.println("turning right");}
    setmotorspeed(LOW_MOTOR_SPEED,HIGH_MOTOR_SPEED);
    case RIGHT:
    if(PRINT_TURNING_DECISIONS){Serial.println("turning left");}
    setmotorspeed(HIGH_MOTOR_SPEED,LOW_MOTOR_SPEED);
    case STRAIGHT:
    if(PRINT_TURNING_DECISIONS){Serial.println("going forward");}
    setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED);
    case STOP:
    if(PRINT_TURNING_DECISIONS){Serial.println("stop");}
    setmotorspeed(0,0);
  }
}


//--------------------------------------------------------------------------------

//Contains turn delay
void calc_mode(){
  //If RIGHT SENSOR LOW (WHITE) and LEFT SENSOR LOW (WHITE) STOP
  if(sensor_stateR < SENSOR_THRESHOLD_R && sensor_stateL < SENSOR_THRESHOLD_L){mode = STOP;return;}
  //If RIGHT SENSOR LOW (WHITE) and LEFT SENSOR HIGH (BLACK) turn RIGHT
  else if(sensor_stateR > SENSOR_THRESHOLD_R && sensor_stateL < SENSOR_THRESHOLD_L){mode = RIGHT;return;}
  //If RIGHT SENSOR HIGH (BLACK) and LEFT SENSOR HIGH (BLACK) stay STRAIGHT
  else if(sensor_stateR > SENSOR_THRESHOLD_R && sensor_stateL > SENSOR_THRESHOLD_L){mode = STRAIGHT;return;}
  //If RIGHT SENSOR HIGH (BLACK) and LEFT SENSOR LOW (WHITE) turn LEFT
  else if(sensor_stateR < SENSOR_THRESHOLD_R && sensor_stateL > SENSOR_THRESHOLD_L){mode = LEFT;return;}
  
}
