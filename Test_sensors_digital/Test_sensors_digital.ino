
const int sensor_pins [4] = {8,9,10,11};
int sensors[4] = {0, 0, 0, 0};

void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
   pinMode(8,INPUT);
   pinMode(9,INPUT);
   pinMode(10,INPUT);
   pinMode(11,INPUT);
   
}

void loop() {
  for  (int i = 0; i < 4; i++)
  {
    sensors[i] = digitalRead(sensor_pins[i]);
    Serial.print(i);
    Serial.print("\t");
    Serial.print("Value: ");
    Serial.print(sensors[i]);
    Serial.print("\t");
  
  }


    Serial.println();
}
