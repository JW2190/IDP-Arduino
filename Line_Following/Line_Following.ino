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
Adafruit_DCMotor *right_motor = AFMS.getMotor(2);
Adafruit_DCMotor *left_motor = AFMS.getMotor(1);

// Sensor Values Global
const int NUM_OF_SENSORS = 4;
float BLACK_TO_WHITE_MULTIPLIER = 0.8;
int SENSOR_THRESHOLDS[NUM_OF_SENSORS]; //
int SENSOR_THRESHOLD [NUM_OF_SENSORS];
int sensor_state [NUM_OF_SENSORS];
int sensor_state_bin [NUM_OF_SENSORS];
int sensor_stateR;
int sensor_stateL;
int sensor_stateM;

//Sensor Connection
const int sensor_pins [NUM_OF_SENSORS] = {A0, A1, A2, A3}; //{L,,M,R}


// Motor Speeds Global
const int HIGH_MOTOR_SPEED = 255;
const int HIGHER_MID_MOTOR_SPEED = 220;
const int MID_MOTOR_SPEED = 200;
const int LOW_MOTOR_SPEED = 150;

//turn delay
const int turn_delay = 50;

//run 1 execution code in loop() (needed for slightly later start of motors)
bool first = true;

//PID objects
float P ,I ,D ,PIDvalue, previousError,Kp, Ki, Kd;
int error = 0;

//Modes
enum mode_frame{LEFT=0,RIGHT=1,STRAIGHT=2,STOP=3};
mode_frame mode = STRAIGHT;

//DEBUG: Toggle functionality easily:
bool MotorsOn = false;
bool PRINT_SENSOR_STATES = false;
bool PRINT_TURNING_DECISIONS = false;
bool PRINT_BIN_SENSOR_STATES = true;


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
    for(byte x=0;x<NUM_OF_SENSORS;x++){
        sensor_state[x] = analogRead(sensor_pins[x]);;
        }
        
    //Getting average of 10 sensor readings and multiplying by scale factor to guess white threshold
    for(byte i=0;i<10;i++){
      for(byte x=0;x<NUM_OF_SENSORS;x++){
        sensor_state[x] = (sensor_state[x]+analogRead(sensor_pins[x]))/2;
        }
      Serial.print("Far Left Black AVG: ");
      Serial.print(sensor_state[0]);
      Serial.print("     Left Black AVG: "); 
      Serial.println(sensor_state[1]);
      Serial.print("     Right Black AVG: "); 
      Serial.print(sensor_state[2]);
      Serial.print("     Far Right Black AVG: "); 
      Serial.println(sensor_state[3]);
      delay(100);
    }
    for(byte x=0;x<NUM_OF_SENSORS;x++){
      SENSOR_THRESHOLD[x] = BLACK_TO_WHITE_MULTIPLIER * sensor_state[x];
      }
    
    Serial.print("Far Left Sensor Threshold: ");
    Serial.println(SENSOR_THRESHOLD[0]);
    Serial.print("Left Sensor Threshold: "); 
    Serial.println(SENSOR_THRESHOLD[1]);
    Serial.print("Right Sensor Threshold: "); 
    Serial.println(SENSOR_THRESHOLD[2]);
    Serial.print("Far right Sensor Threshold: "); 
    Serial.println(SENSOR_THRESHOLD[3]);
    Serial.println("--------------------------");
    
}

//------------------------------------------------------------------------------------------

void loop() {

  //READ SENSOR STATES
  for(byte x=0;x<NUM_OF_SENSORS;x++){
        sensor_state[x] = analogRead(sensor_pins[x]);;
        }
  if(PRINT_SENSOR_STATES){
    //Serial.print("FL: ");
    Serial.print(sensor_state[0]);
    //Serial.print("------");
    //Serial.print("L: ");
    Serial.print(sensor_state[1]);
    //Serial.print("------");
    //Serial.print("R: ");
    Serial.print(sensor_state[2]);
    //Serial.print("------");
    //Serial.print("FR: ");
    Serial.println(sensor_state[3]);
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

  analoguetobin();
  calc_mode();
  if(mode == STOP){
    setmotorspeed(0,0);
  }
  else{
    follow_line();
    } 
  delay(turn_delay);
}


//---------------------------------------------------------------------------------------------

void setmotorspeed(int L_speed,int R_speed){
  left_motor->setSpeed(L_speed);
  right_motor->setSpeed(R_speed);
  
}

//--------------------------------------------------------------------------------

//PID function
void calculatePID(){
  P = error;
  I = I + error;
  D = error-previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;
  }


//--------------------------------------------------------------------------------

void follow_line(){
  switch(error){
    case 3:
    if(PRINT_TURNING_DECISIONS){Serial.println("sharp left");}
    setmotorspeed(LOW_MOTOR_SPEED,HIGH_MOTOR_SPEED);
    break;
    case 2:
    if(PRINT_TURNING_DECISIONS){Serial.println("mid left");}
    setmotorspeed(MID_MOTOR_SPEED,HIGH_MOTOR_SPEED);
    break;
    case 1:
    if(PRINT_TURNING_DECISIONS){Serial.println("slight left");}
    setmotorspeed(HIGHER_MID_MOTOR_SPEED,HIGH_MOTOR_SPEED);
    break;
    case 0:
    if(PRINT_TURNING_DECISIONS){Serial.println("forward");}
    setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED);
    break;
    case -1:
    if(PRINT_TURNING_DECISIONS){Serial.println("slight right");}
    setmotorspeed(HIGH_MOTOR_SPEED,HIGHER_MID_MOTOR_SPEED);
    break;
    case -2:
    if(PRINT_TURNING_DECISIONS){Serial.println("mid right");}
    setmotorspeed(HIGH_MOTOR_SPEED,MID_MOTOR_SPEED);
    break;
    case -3:
    if(PRINT_TURNING_DECISIONS){Serial.println("sharp right");}
    setmotorspeed(HIGH_MOTOR_SPEED,LOW_MOTOR_SPEED);
    break;
  }
}

//--------------------------------------------------------------------------------
//does not work
void follow_line2(){
  setmotorspeed(255*sensor_state[0]*0.8/SENSOR_THRESHOLD[0],255*sensor_state[2]*0.8/SENSOR_THRESHOLD[2]);
  }

//--------------------------------------------------------------------------------

//1 0 0 0  3
//1 1 0 0  2
//0 1 0 0  1
//0 1 1 0  0
//0 0 1 0  -1
//0 0 1 1  -2
//0 0 0 1  -3

//0 0 0 0 no line
//1 1 1 1 line


//Contains turn delay
void calc_mode(){
  /*
  //If RIGHT SENSOR LOW (WHITE) and LEFT SENSOR LOW (WHITE) STOP
  if(sensor_stateR < SENSOR_THRESHOLD_R && sensor_stateL < SENSOR_THRESHOLD_L){mode = STOP;return;}
  //If RIGHT SENSOR LOW (WHITE) and LEFT SENSOR HIGH (BLACK) turn RIGHT
  else if(sensor_stateR > SENSOR_THRESHOLD_R && sensor_stateL < SENSOR_THRESHOLD_L){mode = RIGHT;return;}
  //If RIGHT SENSOR HIGH (BLACK) and LEFT SENSOR HIGH (BLACK) stay STRAIGHT
  else if(sensor_stateR > SENSOR_THRESHOLD_R && sensor_stateL > SENSOR_THRESHOLD_L){mode = STRAIGHT;return;}
  //If RIGHT SENSOR HIGH (BLACK) and LEFT SENSOR LOW (WHITE) turn LEFT
  else if(sensor_stateR < SENSOR_THRESHOLD_R && sensor_stateL > SENSOR_THRESHOLD_L){mode = LEFT;return;}
  */
  //1 0 0 0  3 sharp left
  if(sensor_state_bin[0]==1 && sensor_state_bin[1]==0 && sensor_state_bin[2]==0 && sensor_state_bin[3]==0){mode = LEFT;error = 3;return;}
  
  //1 1 0 0  2 mid left
  else if(sensor_state_bin[0]==1 && sensor_state_bin[1]==1 && sensor_state_bin[2]==0 && sensor_state_bin[3]==0){mode = LEFT;error = 2;return;}
  
  //0 1 0 0  1 slight left
  else if(sensor_state_bin[0]==0 && sensor_state_bin[1]==1 && sensor_state_bin[2]==0 && sensor_state_bin[3]==0){mode = LEFT;error = 1;return;}
  
  //0 1 1 0  0 straight
  else if(sensor_state_bin[0]==0 && sensor_state_bin[1]==1 && sensor_state_bin[2]==1 && sensor_state_bin[3]==0){mode = STRAIGHT;error = 0;return;}

  //0 0 0 0  0 straight
  else if(sensor_state_bin[0]==0 && sensor_state_bin[1]==0 && sensor_state_bin[2]==0 && sensor_state_bin[3]==0){mode = STRAIGHT;error = 0;return;}
  
  //0 0 1 0  -1 small right
  else if(sensor_state_bin[0]==0 && sensor_state_bin[1]==0 && sensor_state_bin[2]==1 && sensor_state_bin[3]==0){mode = RIGHT;error = -1;return;}
  
  //0 0 1 1  -2 mid right
  else if(sensor_state_bin[0]==0 && sensor_state_bin[1]==0 && sensor_state_bin[2]==1 && sensor_state_bin[3]==1){mode = RIGHT;error = -2;return;}
  
  //0 0 0 1  -3 sharp right
  else if(sensor_state_bin[0]==0 && sensor_state_bin[1]==0 && sensor_state_bin[2]==0 && sensor_state_bin[3]==1){mode = RIGHT;return;}
  
  //0 0 0 0  stop
  else if(sensor_state_bin[0]==0 && sensor_state_bin[1]==0 && sensor_state_bin[2]==0 && sensor_state_bin[3]==0){mode = STOP;error = 1;return;}
}

//--------------------------------------------------------------------------------

//temporary script until alex finishes sensor circuit
void analoguetobin(){
  for(byte x=0;x<NUM_OF_SENSORS;x++){
    if(sensor_state[x] < SENSOR_THRESHOLD[x]){
      sensor_state_bin[x] = 1;
    }
    else{
      sensor_state_bin[x] = 0;
      }
      //Serial.println(sensor_state_bin[x]);
    }
    if(PRINT_BIN_SENSOR_STATES){
      Serial.print(sensor_state_bin[0]);
      Serial.print(sensor_state_bin[1]);
      Serial.print(sensor_state_bin[2]);
      Serial.println(sensor_state_bin[3]);
      }
  }
