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
int sensor_state [NUM_OF_SENSORS];
//Sensor Connection
const int sensor_pins [NUM_OF_SENSORS] = {5,6,7,8}; //{Back Left, Front Left, Front Right, Back right}

//Amber flashing light
int amber_light_counter = 0;
bool amber_light_on = false;

//Servo
Servo leftservo;
Servo rightservo;

// Motor Speeds Global
const int HIGH_MOTOR_SPEED = 255;

//turn delay
const int turn_delay = 20;

//Junction Handling
int JUNCTIONS_FOUND = 0;
bool lock_junctions = false;
int junction_locking_counter = 0;
int JUNCTION_LOCK_TIME = 14000; //ms

//Blocks:
bool metal_block = false;
int deposited = 0;

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
  pinMode(13, INPUT);

  //set line sensor input pins
  for(byte i=0;i<NUM_OF_SENSORS;i++){
    pinMode(sensor_pins[i], INPUT);
    }

  // SERVO: attaches the servos on pin 6 to the servo object and reset rotation if needed
  leftservo.attach(9);  
  rightservo.attach(10);
  open_claws(0,50);
  close_claws(50,0);
  
  AFMS.begin();

  //ENABLE MOTOR TURNING
  if(MotorsOn){
    left_motor->run(FORWARD);
    right_motor->run(FORWARD);
    }
}
//------------------------------------------------------------------------------------------

void loop(){
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

  //junction detection locking:
  if(lock_junctions){
    junction_locking_counter++;
    if(junction_locking_counter >= JUNCTION_LOCK_TIME/turn_delay){
      Serial.println("junction detection unlocked");
      junction_locking_counter=0;
      lock_junctions = false;
      }
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


  //Calculate motor turning direction based on sensor inputs
  calc_mode();
  
  //DEBUG: toggling motors on and off based on button input
  if(digitalRead(13)==1){
    Serial.println("Button Pressed");
    MotorsOn = !MotorsOn;
  }
  if(!MotorsOn){
    Serial.println("motors off");
    left_motor->run(RELEASE);
    right_motor->run(RELEASE);
  }

  //handle junction detection
  if(mode == STOP){
    Serial.print("JUNCTIONS FOUND: ");
    Serial.println(JUNCTIONS_FOUND);
    setmotorspeed(0,0);
    switch(JUNCTIONS_FOUND){
      //case 1 is end of robot start area. 2 is block deposit junction
      case 2:
        lock_junctions = true;
        break;
      case 3: //at block collection area
        backward_for(2000);
        open_claws(0,180);
        forward_for(3800);
        close_claws(180,70);
        forward_for(500);
        close_claws(100,0);
        delay(100);
        detect_metal();
        turn_angle(180,true);
        lock_junctions=true;
        break;
      //4 is junction at block collection area if we go inside area
      case 4: //junction at block deposit area
        forward_for(1500); //this line is to get the robot forward so it rotates aligned with the line
        if(metal_block){
          turn_90_to_deposit(true);
          }
        else{
          turn_90_to_deposit(false);
          }
        setmotorspeed(0,0);
        open_claws(0,180);
        forward_for(500);
        backward_for(2500);
        close_claws(180,0);
        forward_for(2000);
        deposited++;
        if(deposited < 6){
          if(metal_block){
            turn_angle(90,true);
            }
          else{
           turn_angle(90,false);
            }
          JUNCTIONS_FOUND = 2;
          lock_junctions = true;
          }
        else if(deposited==6){
          if(metal_block){
            turn_angle(90,false);
            }
          else{
           turn_angle(90,true);
            }
          JUNCTIONS_FOUND = 4;
          }
        break;
      case 5:
        forward_for(2000);
        delay(10000000); //wait in starting area
        //deposit area start
      }
      setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED); //Stop junction from being found again
      delay(500); //Stop junction from being found again
    }
  delay(turn_delay);
}

//---------------------------------------------------------------------------------------------
//Currently not being used
void setmotorspeed(int L_speed,int R_speed){
  left_motor->setSpeed(L_speed);
  right_motor->setSpeed(R_speed);
  }

//---------------------------------------------------------------------------------------------
void turn_left(){
  left_motor->run(BACKWARD);
  right_motor->run(FORWARD); 
  setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED);
  }
void turn_right(){
  left_motor->run(FORWARD);
  right_motor->run(BACKWARD);
  setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED);
  }
void forward(){
  left_motor->run(FORWARD);
  right_motor->run(FORWARD);
  setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED);
  }
void backward(){
  left_motor->run(BACKWARD);
  right_motor->run(BACKWARD);
  setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED);
  } 
//---------------------------------------------------------------------------------------------
//will move forward/backward for x time then stop
void forward_for(int ms){
  left_motor->run(FORWARD);
  right_motor->run(FORWARD);
  setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED);
  delay(ms);
  setmotorspeed(0,0);
  }

void backward_for(int ms){
  left_motor->run(BACKWARD);
  right_motor->run(BACKWARD);
  setmotorspeed(HIGH_MOTOR_SPEED,HIGH_MOTOR_SPEED);
  delay(ms);
  setmotorspeed(0,0);
  }
//---------------------------------------------------------------------------------------------

//Contains turn delay
void calc_mode(){
  if(MotorsOn){
    //HIGH PRIORITY STATES: Back sensors for detecting junctions and catching robot if going off line
    if(sensor_state[0]==1 && sensor_state[1]==1 && sensor_state[2]==1 && sensor_state[3]==1){
      if(PRINT_TURNING_DECISIONS){Serial.println("Decision: junction found");}
      if(!lock_junctions){
        mode = STOP;
        JUNCTIONS_FOUND++;
        forward();
        return;
        }
      }
    else if(sensor_state[0]==1 && sensor_state[1] ==1 && lock_junctions){ //catch slope lip line sensor error
      forward();
      delay(1000);
      return;
      }
    
    else if(sensor_state[0]==1 && sensor_state[3]==0){
      mode = LEFT;
      if(PRINT_TURNING_DECISIONS){Serial.println("Decision: back left on");}
      turn_left();
      return;
      }
    else if(sensor_state[0]==0 && sensor_state[3]==1){
      mode = RIGHT;
      if(PRINT_TURNING_DECISIONS){Serial.println("Decision: back right on");}
      turn_right();
      return;
      }
    //LOWER PRIORITY STATES
    else if(sensor_state[1]==0 && sensor_state[2]==1){
      mode = RIGHT;
      if(PRINT_TURNING_DECISIONS){Serial.println("Decision: front right on");}
      turn_right();
      return;
      }
    else if(sensor_state[1]==0 && sensor_state[2]==0){
      mode = RIGHT;
      if(PRINT_TURNING_DECISIONS){Serial.println("both off");}
      forward();
      return;
      }
    else if(sensor_state[1]==1 && sensor_state[2]==0){
      mode = LEFT;
      if(PRINT_TURNING_DECISIONS){Serial.println("Decision: front left on");}
      turn_left();
      return;
      }
    else if(sensor_state[1]==1 && sensor_state[2]==1){
      mode = RIGHT;
      if(PRINT_TURNING_DECISIONS){Serial.println("both off");}
      forward();
      return;
      }
  }
}
//-------------------------------------------------------------------------------

void turn_angle(int degree,bool clockwise){
  setmotorspeed(0,0);
  Serial.print("Turning: ");
  if(clockwise){
    Serial.println("clockwise");
    turn_right();
    }
  else{
    Serial.println("anticlockwise");
    turn_left();
  }
  if(degree==180){ //let the turning go for a bit before we check for the stop point
    delay(4000);
    }
  else if(degree==90){
    delay(1500);
    }
  for(int i;i<100000;i++){
    for(byte x=0;x<NUM_OF_SENSORS;x++){
      sensor_state[x] = digitalRead(sensor_pins[x]);
      }
    if(clockwise && sensor_state[0]==0 && sensor_state[1]==1 && sensor_state[2]==0 && sensor_state[3]==0){
      break;
      }
    if(!clockwise && sensor_state[0]==0 && sensor_state[1]==0 && sensor_state[2]==1 && sensor_state[3]==0){
      break;
      }
    }
  setmotorspeed(0,0);
  left_motor->run(FORWARD);
  right_motor->run(FORWARD);
  }
//------------------------------------------------------------------------------
void turn_90_to_deposit(bool clockwise){
  setmotorspeed(0,0);
  Serial.print("Turning 180");
  if(clockwise){
    turn_right();
    }
  else{
    turn_left();
    }
  delay(2500);
  for(int i;i<100000;i++){
    for(byte x=0;x<NUM_OF_SENSORS;x++){
      sensor_state[x] = digitalRead(sensor_pins[x]);
      }
    if(sensor_state[0]==0 && sensor_state[1]==0 && sensor_state[2]==0 && sensor_state[3]==0){
      break;
      }
    }
}
//------------------------------------------------------------------------------
void open_claws(int open_from, int open_to){    
    for (int pos = open_from; pos <= open_to; pos += 1) { 
    // in steps of 1 degree
    leftservo.write(pos);
    rightservo.write(180-pos);// tell servo to go to position in variable 'pos'
    delay(15);     }// waits 15ms for the servo to reach the position
  }
//------------------------------------------------------------------------------
void close_claws(int close_from,int close_to){
  for (int pos = close_from; pos >= close_to; pos -= 1) { // goes from 180 degrees to 0 degrees
    leftservo.write(pos);              // tell servo to go to position in variable 'pos'
    rightservo.write(180-pos);
    delay(15); 
    }
  }
//------------------------------------------------------------------------------
void detect_metal(){
     
  }
