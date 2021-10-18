
const int sensor_pins [4] = {A0, A1, A2, A3};
int sensors[4] = {0, 0, 0, 0};

void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);
}

void loop() {
  for  (int i = 0; i < 4; i++)
  {
    sensors[i] = analogRead(sensor_pins[i]);
    Serial.print(i);
    Serial.print("\t");
    Serial.print("Value: ");
    Serial.print(sensors[i]);
    Serial.print("\t");
  
  }


    Serial.println();
}
