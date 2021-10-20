#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);

const int sensor_pins [4] = {A0, A1, A2, A3};
int sensors[4] = {0, 0, 0, 0};
int weights[4] = {-2, -1, 1, 2};


int positions, proportional, integral, derivative, last_proportional, error_value;

int Kp = 1;
int Ki = 0;
int Kd = 0;

int right_motor_speed = 0;
int left_motor_speed = 0;
int max_speed = 255;

int set_point = 0;



void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
   AFMS.begin();
   left_motor->run(BACKWARD);
   right_motor->run(BACKWARD);
   calibration(4);

   Serial.print("-------CALIBRATION FINISHED---------");

//   left_motor-> setSpeed(255);
//   right_motor->setSpeed(255);
//
//   delay(100);
//
//   left_motor-> setSpeed(0);
//   right_motor->setSpeed(0);
}

void loop() 
{
  read_sensors();
  pid_calc();
  calc_turn();
  //motor_drive();

  Serial.print("Position:");
  Serial.print("\t");
  Serial.print(positions);
  Serial.print("\t");
  Serial.println();
  Serial.print("Error Value:");
  Serial.print("\t");
  Serial.print(error_value);
}



void read_sensors()
{
  positions = 0;
  for  (int i = 0; i < 4; i++)
  {
    sensors[i] = analogRead(sensor_pins[i]); 
    positions += sensors[i] * weights[i];
  }
}
void pid_calc()
{ 
  delay(100);
  proportional = positions - set_point;
  integral = integral + proportional;
  derivative = proportional - last_proportional;
  last_proportional = proportional;
  error_value = int(proportional * Kp + integral * Ki + derivative * Kd);
}

void calc_turn()
{ //Restricting the error value between +-255.
  if (error_value < -1 * max_speed)
   {
  error_value = -1 * max_speed;
   }
   
  if (error_value > max_speed)
   {
  error_value = max_speed;
   }

   
   // If error_value is less than zero calculate right turn speed values
  if (error_value > 0)
   {
  right_motor_speed = max_speed - error_value;
  left_motor_speed = max_speed;
   }
   // Iferror_value is greater than zero calculate left turn values
  else
   {
  right_motor_speed = max_speed;
  left_motor_speed = max_speed + error_value;
   }
}

void motor_drive()
{
  left_motor->setSpeed(left_motor_speed);
  right_motor->setSpeed(right_motor_speed);
}

void calibration(int number_of_sensors) 
{
    long sensors_blk_sum[number_of_sensors] = {0, 0, 0, 0};
    int sensors_blk_avg[number_of_sensors] = {0, 0, 0, 0};
    int sensors_threshold[number_of_sensors] = {0, 0, 0, 0};
    int sensors_wht_sum[number_of_sensors] = {0, 0, 0, 0}; 
    int sensors_wht_avg[number_of_sensors] = {0, 0, 0, 0};
    const int SAMPLES = 150;
    const float BLACK_TO_WHITE_MULTIPLIER = 0.5;
    int truth_counter = 0;



    //AUTO CALIBRATE SENSORS (Must be started on BLACK surface)
    delay(100);
    Serial.println("--------BLACK CALIBRATION-------");

    for (int i = 0; i < SAMPLES; i++)
    { 
      delay(20);
      
      for (int j = 0; j < number_of_sensors; j++)
      {
        sensors_blk_sum[j] += analogRead(sensor_pins[j]);
      }
      
    }

    for (int k = 0; k < number_of_sensors; k++)
    {
      sensors_blk_avg[k] = sensors_blk_sum[k] / SAMPLES;
      Serial.print("Sensor: ");
      Serial.print(k);
      Serial.print("\t");
      Serial.print("Black AVERAGE Value: ");
      Serial.print(sensors_blk_avg[k]);
      Serial.println();
      
      sensors_threshold[k] = BLACK_TO_WHITE_MULTIPLIER * sensors_blk_avg[k];
      Serial.print("Sensor: ");
      Serial.print(k);
      Serial.print("\t");
      Serial.print("THRESHOLD Value: ");
      Serial.print(sensors_threshold[k]);
      Serial.println();
    }

    
    Serial.println("--------------------------");
    Serial.println("--------MOVING TO WHITE JUNCTION--------");
    
    left_motor->setSpeed(80);
    right_motor->setSpeed(80);

      

    for (int r = 0; r < 100; r++)
    {
      delay(20);
      for (int j = 0; j < number_of_sensors; j++)
      {
        sensors[j] += analogRead(sensor_pins[j]);
      }

      truth_counter = 0;
      for (int y = 0; y < number_of_sensors; y++)
      {
        if (sensors[y] < sensors_threshold[y])
        {
        truth_counter += 1;
        }

        else
        {
          break;
        }
      }

      if (truth_counter == number_of_sensors) 
      {
        left_motor->setSpeed(0);
        right_motor->setSpeed(0);
        Serial.println("-------WHITE CALIBRATION--------");
        break;
      }
    }

    

    for (int q = 0; q < SAMPLES; q++)
    { 
      delay(20);
      
      for (int w = 0; w < number_of_sensors; w++)
      {
        sensors_wht_sum[w] += analogRead(sensor_pins[w]);
      }
    }

    for (int e = 0; e < number_of_sensors; e++)
    {
      sensors_wht_avg[e] = sensors_wht_sum[e] / SAMPLES;
      Serial.print("Sensor: ");
      Serial.print(e);
      Serial.print("\t");
      Serial.print("White AVERAGE Value: ");
      Serial.print(sensors_wht_avg[e]);
      Serial.println();
      
      sensors_threshold[e] = (sensors_wht_avg[e] + sensors_blk_avg[e]) / 2;
      Serial.print("Sensor: ");
      Serial.print(e);
      Serial.print("\t");
      Serial.print("NEW THRESHOLD Value: ");
      Serial.print(sensors_threshold[e]);
      Serial.println();
    }
    
    Serial.println("--------------------------"); 
    
    set_point = 0;
    for (int h = 0; h < number_of_sensors; h++)
    {
      // This depends on sensor arrangment
      if (h == 1 || h == 2)
      {
        set_point += sensors_blk_avg[h] * weights[h];
      }

      else
      {
        set_point += sensors_wht_avg[h] * weights[h];
      }
    }

    Serial.print("SET POINT:");
    Serial.print("\t");
    Serial.print(set_point);
    left_motor->run(RELEASE);
    right_motor->run(RELEASE);
}
