#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);




const int sensor_pins [2] = {A0, A1};
int sensors[2] = {0, 0};


int sensors_blk_sum[2] = {0, 0}; 
int sensors_blk_avg[2] = {0, 0};
int sensors_threshold[2] = {0, 0};
int sensors_wht_sum[2] = {0, 0}; 
int sensors_wht_avg[2] = {0, 0};


const int SAMPLES = 50;
const float BLACK_TO_WHITE_MULTIPLIER = 0.8;

void setup() {
  Serial.begin(9600); 
  AFMS.begin();
  left_motor->run(BACKWARD);
  right_motor->run(BACKWARD);
  calibration();

}

void loop() {
  // put your main code here, to run repeatedly:

}


void calibration() 
{
    //AUTO CALIBRATE SENSORS (Must be started on BLACK surface)
    delay(100);
    Serial.println("--------BLACK CALIBRATION-------");

    for (int i = 0; i < SAMPLES; i++)
    { 
      delay(20);
      
      for (int j = 0; j < 2; j++)
      {
        sensors_blk_sum[j] += analogRead(sensor_pins[j]);
      }
      
    }

    for (int k = 0; k < 2; k++)
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
    
    left_motor->setSpeed(75);
    right_motor->setSpeed(75);

      

    for (int r = 0; r < 1000; r++)
    {
      delay(20);
      sensors[0] = analogRead(sensor_pins[0]);
      sensors[1] = analogRead(sensor_pins[1]);

      Serial.print(sensors[0]);
      Serial.print("\t");
      Serial.print(sensors[1]);
      Serial.print("\t");
      Serial.println();

      if (sensors[0] < sensors_threshold[0] && sensors[1] < sensors_threshold[1])
      {
        left_motor->setSpeed(0);
        right_motor->setSpeed(0);
        Serial.println("--------WHITE CALIBRATION-------");
        break;
      }

      else
      {
        continue;
      }
    
      
    }

    for (int q = 0; q < SAMPLES; q++)
    { 
      delay(20);
      
      for (int w = 0; w < 2; w++)
      {
        sensors_wht_sum[w] += analogRead(sensor_pins[w]);
      }
    }

    for (int e = 0; e < 2; e++)
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
  
}
