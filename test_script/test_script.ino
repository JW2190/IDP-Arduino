void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Start serial and set the correct Baud Rate
  Serial.print("Test");
  pinMode(4,OUTPUT);
  pinMode(A5,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
}
