int counter =0;
bool covered;

void setup() {
  // put your setup code here, to run once:
  pinMode(3,OUTPUT);
  pinMode(2,INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(counter);
  if(digitalRead(2)==HIGH){
    if(covered==false){
      covered = true;
      counter++;
      }
    digitalWrite(3,HIGH);
    }
    else{
      covered = false;
      digitalWrite(3,LOW);
      
      }
}
