#include <Servo.h>
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

//Sensor Connection
const int sensor_pins [NUM_OF_SENSORS] = {8,9,10,11}; //{L,,M,R}

//Amber flashing light
int amber_light_counter = 0;
bool amber_light_on = false;

//Servo
Servo myservo;

// Motor Speeds Global
const int HIGH_MOTOR_SPEED = 255;
const int HIGHER_MID_MOTOR_SPEED = 180;
const int MID_MOTOR_SPEED = 150;
const int LOW_MOTOR_SPEED = 100;

//turn delay
const int turn_delay = 20; //**DO NOT CHANGE**

//Junction Handling
int JUNCTIONS_FOUND = 0;

//PID objects
float P ,I ,D ,PIDvalue, previousError;
float Kd = 30;
float Ki = 1;
float Kp = 30; //max value can be is 18 for average motor speed to be 200
int error = 0;

//Modes
enum mode_frame{LEFT=0,RIGHT=1,STRAIGHT=2,STOP=3};
mode_frame mode = STRAIGHT;

//DEBUG: Toggle functionality easily:
bool MotorsOn = false;
bool PRINT_SENSOR_STATES = false;
bool PRINT_TURNING_DECISIONS = false;


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("---------------SETUP---------------");

  //Set amber flashing light pin
  pinMode(1, OUTPUT);
  
  //set motor toggle button
  pinMode(7, INPUT);

  //Wheel encoders:
  //LEFT:
  pinMode(5,OUTPUT); //For LED
  pinMode(4,INPUT); // Actual Encoder reading
  //RIGHT:
  pinMode(3,OUTPUT); //For LED
  pinMode(2,INPUT); // Actual Encoder reading
  
  // SERVO: attaches the servo on pin 6 to the servo object and reset rotation if needed
  myservo.attach(6);  
  myservo.write(180);
  
  //set line sensor input pins
  for(byte i=0;i<NUM_OF_SENSORS;i++){
    pinMode(sensor_pins[i], INPUT);
  }
  AFMS.begin();

  //ENABLE MOTOR TURNING
    if(MotorsOn){
      left_motor->run(FORWARD);
      right_motor->run(FORWARD);
      }
}
//------------------------------------------------------------------------------------------

void loop() {
  
  //Amber flashing light requirement: keeps light on for 1 turn_delay every 0.5 seconds
  amber_light_counter++;
  if(amber_light_on == true){ //turns light off if still on
    digitalWrite(1,LOW);
    amber_light_on = false;
    }
  if(amber_light_counter >= 500/turn_delay && mode!=STOP){ //turn light on every 0.5 seconds
    amber_light_counter = 0;
    digitalWrite(1,HIGH);
    amber_light_on = true;
    }

  
  //READ SENSOR STATES
  for(byte x=0;x<NUM_OF_SENSORS;x++){
        sensor_state[x] = digitalRead(sensor_pins[x]);
        }
  if(PRINT_SENSOR_STATES){
    Serial.print(sensor_state[0]);
    Serial.print(sensor_state[1]);
    Serial.print(sensor_state[2]);
    Serial.println(sensor_state[3]);
  }
  

  //DEBUG: toggling motors on and off based on button input
  if(digitalRead(7)==1){
    Serial.println("Button Pressed");
    MotorsOn = !MotorsOn;
  }
  if(MotorsOn){
    left_motor->run(FORWARD);
    right_motor->run(FORWARD);
  }
  else{
    left_motor->run(RELEASE);
    right_motor->run(RELEASE);
    }

  calc_mode();

  //handle junction detection
  if(mode == STOP){
    
    Serial.println(JUNCTIONS_FOUND);
    setmotorspeed(0,0);
    switch(JUNCTIONS_FOUND){
      case 3: 
      turn_180();
      break;
      }
      setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED); //Stop junction from being found again
      delay(500); //Stop junction from being found again
    }
  else{
    calculatePID();
    follow_line();
    //setpidmotorspeed();
    } 
  delay(turn_delay);
}


//---------------------------------------------------------------------------------------------

void setmotorspeed(int L_speed,int R_speed){
  left_motor->setSpeed(L_speed);
  right_motor->setSpeed(R_speed);
  
}
//---------------------------------------------------------------------------------------------

void setpidmotorspeed(){
  Serial.print("PIDvalue:");
  Serial.println(PIDvalue);
  //left_motor->setSpeed(200-PIDvalue);
  //right_motor->setSpeed(200+PIDvalue);
  if(PIDvalue == 0){
    left_motor->setSpeed(255);
    right_motor->setSpeed(255);
    }
  else if(PIDvalue>0){ //positive when vehicle should turn LEFT
    left_motor->setSpeed(255-PIDvalue);
    right_motor->setSpeed(255);
    } 
  else if(PIDvalue<0){ //negative when vehicle should turn RIGHT
    left_motor->setSpeed(255);
    right_motor->setSpeed(255+PIDvalue);
    }
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
    //left_motor->run(BACKWARD);
    //right_motor->run(FORWARD);
    //setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED);
    break;
    case 0:
    if(PRINT_TURNING_DECISIONS){Serial.println("forward");}
    setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED);
    //left_motor->run(FORWARD);
    //right_motor->run(FORWARD);
    //setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED);
    break;
    case -1:
    if(PRINT_TURNING_DECISIONS){Serial.println("slight right");}
    setmotorspeed(HIGH_MOTOR_SPEED,HIGHER_MID_MOTOR_SPEED);
    //left_motor->run(FORWARD);
    //right_motor->run(BACKWARD);
    //setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED);
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
  //1 0 0 0  3 sharp left
  if(sensor_state[0]==1 && sensor_state[1]==0 && sensor_state[2]==0 && sensor_state[3]==0){mode = LEFT;error = 3;return;}
  
  //1 1 0 0  2 mid left
  else if(sensor_state[0]==1 && sensor_state[1]==1 && sensor_state[2]==0 && sensor_state[3]==0){mode = LEFT;error = 2;return;}
  
  //0 1 0 0  1 slight left
  else if(sensor_state[0]==0 && sensor_state[1]==1 && sensor_state[2]==0 && sensor_state[3]==0){mode = LEFT;error = 1;return;}
  
  //0 1 1 0  0 straight
  else if(sensor_state[0]==0 && sensor_state[1]==1 && sensor_state[2]==1 && sensor_state[3]==0){mode = STRAIGHT;error = 0;return;}

  //0 0 0 0  0 straight
  else if(sensor_state[0]==0 && sensor_state[1]==0 && sensor_state[2]==0 && sensor_state[3]==0){mode = STRAIGHT;error = 0;return;}
  
  //0 0 1 0  -1 small right
  else if(sensor_state[0]==0 && sensor_state[1]==0 && sensor_state[2]==1 && sensor_state[3]==0){mode = RIGHT;error = -1;return;}
  
  //0 0 1 1  -2 mid right
  else if(sensor_state[0]==0 && sensor_state[1]==0 && sensor_state[2]==1 && sensor_state[3]==1){mode = RIGHT;error = -2;return;}
  
  //0 0 0 1  -3 sharp right
  else if(sensor_state[0]==0 && sensor_state[1]==0 && sensor_state[2]==0 && sensor_state[3]==1){mode = RIGHT; error = -3;return;}
  
  //1 1 1 1  stop
  else if(sensor_state[0]==1 && sensor_state[1]==1 && sensor_state[2]==1 && sensor_state[3]==1){mode = STOP;error = 0;JUNCTIONS_FOUND++;return;}
}

//--------------------------------------------------------------------------------

void turn_right(){
  setmotorspeed(0,0);
  Serial.print("Turning right");
  left_motor->run(FORWARD);
  right_motor->run(BACKWARD);
  setmotorspeed(255,255);
  delay(850); //let the turning go for a bit before we check for the stop point
  setmotorspeed(0,0);
  left_motor->run(FORWARD);
  right_motor->run(FORWARD);
  
  }
//-------------------------------------------------------------------------------

void turn_180(){
  setmotorspeed(0,0);
  Serial.print("Turning 180");
  left_motor->run(FORWARD);
  right_motor->run(BACKWARD);
  setmotorspeed(255,255);
  delay(1700); //let the turning go for a bit before we check for the stop point
  setmotorspeed(0,0);
  left_motor->run(FORWARD);
  right_motor->run(FORWARD);
  }

//-------------------------------------------------------------------------------

void collect_block(){
  //Move forwards until the first junction is reached
  setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED);
  for(byte i; i<10000;i++){
    calc_mode();
    if(mode == STOP){
      break;
      }
    }
  
  }
