// Metal detector initialization

const byte npulse = 30;
const bool debug = true;

#define pulsepin A0
#define cappin A1
#define rLEDpin 12
#define gLEDpin 11
#define measureLEDpin 2

#define measurepin 3 //used for testing to trigger measurement period

// global variables
const int nmeas=128;  //measurements to take
long int sumsum=0; //running sum of 64 sums 
long int skip=0;   //number of skipped sums
long int diff=0;        //difference between sum and avgsum
long int flash_period=0;//period (in ms) 
long unsigned int prev_flash=0; //time stamp of previous flash
int ref = 0; //reference value of average diff to switch to metal state, lower is more sensitive
bool isMeasure=false;//whether the program is in the measurement period
long int avgsum;

// averaging variables
int nAverage = 10;         //define the number of points to calculate the average
int diffReadings[10];      //set number = nAverage
long sumReading;            //stores the sum of all readings to calculate average, spreads calulation over more cycles
int index = 0;              //stores the index position in the average array for next cycle
int average;                //stores final average calulation sumReading/nAverage



//---------------------------------------------------------------------------------------------------------------
//Other initialization

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
const int sensor_pins [NUM_OF_SENSORS] = {5,6,7,10}; //{Far Left, Middle Left, Middle Right, Far right}

//Amber flashing light
int amber_light_counter = 0;
bool amber_light_on = false;

//Servo
Servo leftservo;
Servo rightservo;
int closed_claw_angle = 23;

// Motor Speeds Global
const int MAX_MOTOR_SPEED = 255;
const int MID_MOTOR_SPEED = 210;
const int LEFT_MOTOR_SPEED = 254; //added biasing to left motor to stop drifting of robot

//timings
const int turn_delay = 20;
//how long the robot should turn for before it starts checking where the line is
const int turn_lock_time_180 = 3000;
const int turn_lock_time_90 = 1300;
const int turn_time_exact_90 = 1500;

//Junction Handling
int JUNCTIONS_FOUND = 0;
bool lock_junctions = false;
int junction_locking_counter = 0;
int JUNCTION_LOCK_TIMES [2] = {9000,9500}; //ms [towards block collection, away from block collection]
int stay_forward_for = 500;

//Blocks:
bool metal_block = false;
int deposited = 0;
const int maxtodeposit = 5;

//Modes
enum mode_frame{LEFT=0,RIGHT=1,STRAIGHT=2,STOP=3,FAR_LEFT=4,FAR_RIGHT=5};
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

  //Metal detector
  pinMode(pulsepin, OUTPUT); 
  digitalWrite(pulsepin, LOW);
  pinMode(cappin, INPUT);  
  pinMode(rLEDpin, OUTPUT);
  digitalWrite(rLEDpin, LOW);
  pinMode(gLEDpin, OUTPUT);
  digitalWrite(gLEDpin, LOW);
  pinMode(measureLEDpin, OUTPUT);
  digitalWrite(measureLEDpin, LOW);
  pinMode(measurepin,INPUT);

  // SERVO: attaches the servos on pin 6 to the servo object and reset rotation if needed
  leftservo.attach(9);  
  rightservo.attach(8);
  open_claws(0,60,15);
  close_claws(60,0,15);
  
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
    if(junction_locking_counter >= JUNCTION_LOCK_TIMES[JUNCTIONS_FOUND-2]/turn_delay){
      Serial.println("junction detection unlocked");
      junction_locking_counter=0;
      lock_junctions = false;
      }
    }

  //READ SENSOR STATES
  read_sensors();
  
  if(PRINT_SENSOR_STATES){
    Serial.print(sensor_state[0]);
    Serial.print(sensor_state[1]);
    Serial.print(sensor_state[2]);
    Serial.println(sensor_state[3]);
    }


  //Calculate motor turning direction based on sensor inputs
  follow_line();
  
  //DEBUG: toggling motors on and off based on button input
  if(digitalRead(13)==1){
    Serial.println("Button Pressed");
    MotorsOn = !MotorsOn;
  }
  if(!MotorsOn){
    left_motor->run(RELEASE);
    right_motor->run(RELEASE);
  }

  //handle junction detection
  if(mode == STOP){
    handle_junctions();
    setmotorspeed(MAX_MOTOR_SPEED,MAX_MOTOR_SPEED); //Stop junction from being found again
    delay(500); //Stop junction from being found again
    }
  delay(turn_delay); //base clock speed set for arduino
}

//---------------------------------------------------------------------------------------------
//Set motor speed
void setmotorspeed(int L_speed,int R_speed){
  left_motor->setSpeed(L_speed);
  right_motor->setSpeed(R_speed);
  }

//---------------------------------------------------------------------------------------------
//Basic turning functions. motor speed is constantly set as a way to reset motor speeds if changed in other functions
void turn_left(){
  left_motor->run(BACKWARD);
  right_motor->run(FORWARD); 
  setmotorspeed(MID_MOTOR_SPEED,MID_MOTOR_SPEED);
  }
void turn_right(){
  left_motor->run(FORWARD);
  right_motor->run(BACKWARD);
  setmotorspeed(MID_MOTOR_SPEED,MID_MOTOR_SPEED);
  }
void forward(){
  left_motor->run(FORWARD);
  right_motor->run(FORWARD);
  setmotorspeed(LEFT_MOTOR_SPEED,MAX_MOTOR_SPEED);
  }
void backward(){
  left_motor->run(BACKWARD);
  right_motor->run(BACKWARD);
  setmotorspeed(MAX_MOTOR_SPEED,MAX_MOTOR_SPEED);
  } 
//---------------------------------------------------------------------------------------------
//will move forward/backward for x time then stop
void forward_for(int ms){
  left_motor->run(FORWARD);
  right_motor->run(FORWARD);
  setmotorspeed(LEFT_MOTOR_SPEED,MAX_MOTOR_SPEED);
  delay(ms);
  setmotorspeed(0,0);
  }

void backward_for(int ms){
  left_motor->run(BACKWARD);
  right_motor->run(BACKWARD);
  setmotorspeed(MAX_MOTOR_SPEED,MAX_MOTOR_SPEED);
  delay(ms);
  setmotorspeed(0,0);
  }
//---------------------------------------------------------------------------------------------

//Contains turn delay
void follow_line(){
  if(MotorsOn){
    //HIGH PRIORITY STATES: Back sensors for detecting junctions and catching robot if going off line

    //junction
    if(sensor_state[0]==1 && sensor_state[1]==1 && sensor_state[2]==1 && sensor_state[3]==1){
      if(PRINT_TURNING_DECISIONS){Serial.println("Decision: junction found");}
      if(!lock_junctions){ //only add to junctions and set mode to stop if junction lock is not on
        mode = STOP;
        JUNCTIONS_FOUND++;
        return;
        }
      forward();
      }

      //catch line sensor ramp errors ----------------------------------- defaulting to moving forward for any glitched sensor states:
      else if(lock_junctions && sensor_state[0]==1 && sensor_state[1] ==1){ //catch slope lip line sensor error
      forward();
      delay(stay_forward_for);
      return;
      }
      else if(lock_junctions && sensor_state[0]==1 && sensor_state[3] ==1){ //catch slope lip line sensor error
      Serial.println("forward for 1");
      forward();
      delay(stay_forward_for);
      return;
      }
      else if(lock_junctions && sensor_state[0]==1 && sensor_state[2] ==1){ //catch slope lip line sensor error
      Serial.println("forward for 1");
      forward();
      delay(stay_forward_for);
      return;
      }
      else if(lock_junctions && sensor_state[2]==1 && sensor_state[3] ==1){ //catch slope lip line sensor error
      Serial.println("forward for 1");
      forward();
      delay(stay_forward_for);
      return;
      }
      else if(lock_junctions && sensor_state[1]==1 && sensor_state[3] ==1){ //catch slope lip line sensor error
      forward();
      delay(stay_forward_for);
      return;
      }
    //---------------------------------------------------

    //
    else if(sensor_state[0]==1 && sensor_state[3]==0){
      if(mode == RIGHT && lock_junctions){ //stop repeating moving left and right
        forward;
        delay(1000);
        return;
        }
      mode = LEFT;
      if(PRINT_TURNING_DECISIONS){Serial.println("Decision: back left on");}
      turn_left();
      return;
      }
      
    else if(sensor_state[0]==0 && sensor_state[3]==1){
      if(mode == LEFT && lock_junctions){ //stop repeating moving left and right
        forward;
        delay(1000);
        return;
        }
      mode = RIGHT;
      if(PRINT_TURNING_DECISIONS){Serial.println("Decision: back right on");}
      turn_right();
      return;
      }
      
    //LOWER PRIORITY STATES: General line following -----------------
    else if(sensor_state[1]==0 && sensor_state[2]==1){
      mode = RIGHT;
      if(PRINT_TURNING_DECISIONS){Serial.println("Decision: front right on");}
      turn_right();
      return;
      }
      
    else if(sensor_state[1]==0 && sensor_state[2]==0){
      //mode = STRAIGHT;
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
      //mode = STRAIGHT;
      if(PRINT_TURNING_DECISIONS){Serial.println("both off");}
      forward();
      return;
      }
  }
}
//-------------------------------------------------------------------------------
void handle_junctions(){
  Serial.print("JUNCTIONS FOUND: ");
    Serial.println(JUNCTIONS_FOUND);
    setmotorspeed(0,0);
    switch(JUNCTIONS_FOUND){
      //case 1 is end of robot start area. case 2 is block deposit junction
      case 2:
        lock_junctions = true;
        forward_for(1000);//move past this junction
        mode = STRAIGHT; //reset mode
        break;
      case 3: //at block collection area
      
        //safely open claws
        backward_for(1500);
        open_claws(10,180,5);
        forward_for(1800);

        if(deposited == 0){ //saving time as location of first block known
          close_claws(180,60,15);
          }
        else{
          //sweep entire block collection area
          follow_line_for(850);
          forward_for(100); //reset any decisions to turn left or right to forwards
        
          close_claws(180,100,15);
          forward_for(400);
          close_claws(100,60,15);
        
          }
        
        backward_for(1200); //draggging block to centre of claws
        delay(700); //awating reduction in vibration before metal detector mean value set
        
        //metal detector calibration
        avgsum = 0; //asking new reference to be set
        metal_detect_loop(); //set new reference
        
        close_claws(60,closed_claw_angle,15); //bring block into electromagnet

        //detect metal
        isMeasure = true;
        while(isMeasure){
          metal_detect_loop();
          }

        //turn and set junction lock
        turn_angle(180,true);
        lock_junctions=true;
        JUNCTIONS_FOUND = 3; //in case the robot finds the same junction again
        mode = STRAIGHT; //reset mode
        break;

      case 4: //junction at block deposit area
        forward_for(500); //get the robot forward so it rotates aligned with the line
        if(metal_block){
          turn_90_to_deposit(true); //rotate to red box
          }
        else{
          turn_90_to_deposit(false); //rotate to blue box
          }
        
        //safely open claws
        backward_for(600);
        setmotorspeed(0,0);
        open_claws(0,120,15);

        //push block into deposit area
        forward_for(1100+200);
        backward_for(2000+200);
        close_claws(120,0,15);
        forward_for(1500);
        
        //turn off metal detetor lights
        digitalWrite(rLEDpin, LOW);
        digitalWrite(gLEDpin, LOW);
        
        deposited++;

        //turn to go back to block collection
        if(deposited < maxtodeposit){
          if(metal_block){
            turn_angle(90,true);
            }
          else{
           turn_angle(90,false);
            }
          JUNCTIONS_FOUND = 2;
          lock_junctions = true;
          }

        //turn to go to start zone
        else{
          if(metal_block){
            turn_angle(90,false);
            }
          else{
           turn_angle(90,true);
            }
          JUNCTIONS_FOUND = 4;
          }
          mode = STRAIGHT; //reset mode
        break;
      case 5: // at starting zone. stop here
        forward_for(1600);
        delay(10000000); //wait in starting area
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
  setmotorspeed(MAX_MOTOR_SPEED,MAX_MOTOR_SPEED); //set it to turn fast
  if(degree==180){ //let it turn for x seconds before we check for the stop point
    delay(turn_lock_time_180);
    }
  else if(degree==90){
    delay(turn_lock_time_90);
    }

  //read sensors and stop at specific cases
  for(int i;i<100000;i++){
    read_sensors();
    if(clockwise && sensor_state[0]==0 && sensor_state[1]==0 && sensor_state[2]==1 && sensor_state[3]==0){
      break;
      }
    if(clockwise && sensor_state[0]==1 && sensor_state[1]==0 && sensor_state[2]==0 && sensor_state[3]==0){
      break;
      }
    if(!clockwise && sensor_state[0]==0 && sensor_state[1]==0 && sensor_state[2]==1 && sensor_state[3]==0){
      break;
      }
    if(!clockwise && sensor_state[0]==0 && sensor_state[1]==0 && sensor_state[2]==0 && sensor_state[3]==1){
      break;
      }
    }
  //reset rotation direction of motors
  forward();
  setmotorspeed(0,0);
  }
//------------------------------------------------------------------------------
//separate function as sensor states are not being looked for. A specific time to rotate is set
void turn_90_to_deposit(bool clockwise){
  setmotorspeed(0,0);
  Serial.print("Turning 180");
  if(clockwise){
    turn_right();
    }
  else{
    turn_left();
    }
  setmotorspeed(MAX_MOTOR_SPEED,MAX_MOTOR_SPEED); //set it to turn fast
  delay(turn_time_exact_90);
  
}
//------------------------------------------------------------------------------
void open_claws(int open_from, int open_to,int timestep){    
    for (int pos = open_from; pos <= open_to; pos += 1) { 
    // in steps of 1 degree
    leftservo.write(pos);
    rightservo.write(190-pos); //was 180-pos but 10 added to make claws meet in exact centre when both partially closed
    delay(timestep); // waits set time for the servo to reach the position
    }
  }
//------------------------------------------------------------------------------
void close_claws(int close_from,int close_to,int timestep){
  for (int pos = close_from; pos >= close_to; pos -= 1) {
    leftservo.write(pos);              
    rightservo.write(190-pos); 
    delay(timestep); // waits set time for the servo to reach the position
    }
  }
//------------------------------------------------------------------------------
void read_sensors(){
  for(byte x=0;x<NUM_OF_SENSORS;x++){
      sensor_state[x] = digitalRead(sensor_pins[x]);
      }
  }
//-----------------------------------------------------------------------------
void follow_line_for(int tme){
  for(int i=0;i<tme/turn_delay;i++){
    read_sensors();
    follow_line();
    delay(turn_delay);
    }
  }
//-----------------------------------------------------------------------------
// Metal detector
// Runs a pulse over the search loop in series with resistor
// Voltage over search loop spikes
// Through a diode this charges a capacitor
// Value of capacitor after series of pulses is read by ADC

// Metal objects near search loop change inductance.
// ADC reading depends on inductance.
// changes wrt long-running mean are indicated by LEDs

// wiring:
// 220Ohm resistor on D2
// 10-loop D=10cm seach loop between ground and resistor
// diode (-) on pin A0 and (+) on loop-resistor connection
// 10nF capacitor between A0 and ground

void metal_detect_loop(){
    int minval=1023;
    int maxval=0;
    
    //perform value measurement
    long unsigned int sum=0;
    for (int imeas=0; imeas<nmeas+2; imeas++){
      //reset the capacitor
      pinMode(cappin,OUTPUT);
      digitalWrite(cappin,LOW);
      delayMicroseconds(20);
      pinMode(cappin,INPUT);
      //apply pulses
      for (int ipulse = 0; ipulse < npulse; ipulse++) {
        digitalWrite(pulsepin,HIGH); //takes 3.5 microseconds
        delayMicroseconds(3);
        digitalWrite(pulsepin,LOW);  //takes 3.5 microseconds
        delayMicroseconds(3);
      }
      //read the charge on the capacitor
      int val = analogRead(cappin); //takes 13x8=104 microseconds
      minval = min(val,minval);
      maxval = max(val,maxval);
      sum+=val;  
    }
  
    //subtract minimum and maximum value to remove spikes
    sum-=minval; sum-=maxval;

    if (avgsum == 0) avgsum=sum+20;//+40; //calibration for when line-by-line printing enabled at script end
    diff=sum-avgsum;

    
    // looks for measurepin (button input) to start measurements
    if (digitalRead(measurepin)==LOW) {
      isMeasure = true;
      digitalWrite(measureLEDpin, HIGH);
    }
      
    // averaging process for measurement
    if (isMeasure) {
      sumReading = sumReading - diffReadings[index] + diff;
      diffReadings[index] = diff;
      index++;
      
      // end measurement period after datapoints for average have been collected
      if (index >= nAverage - 1) {
        isMeasure = false;
        index = 0;
        average = sumReading / nAverage;
        digitalWrite(measureLEDpin, LOW); 
            
        if (debug) {
          Serial.print("Average: ");
          Serial.println(average);
        }
        
        // comparing average measuremnt to reference value 
        if (average < ref) {
          digitalWrite(rLEDpin, HIGH);
          digitalWrite(gLEDpin, LOW);
          metal_block = true;
          Serial.println("in metal true");     
        }
        
        else {
          digitalWrite(rLEDpin, LOW);
          digitalWrite(gLEDpin, HIGH); 
          metal_block = false;
          Serial.println("in metal false");
        }
      }
    }
    if (false){
      Serial.print(sum); 
      Serial.print(" ");
      Serial.print(avgsum); 
      Serial.print(" ");
      Serial.print(diff); 
      Serial.println();
    }
  }
//------------------------------------------------------------------------------
