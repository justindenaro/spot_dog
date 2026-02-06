char c = ' ';

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Serial1.begin(9600);
  Serial.println("Serial1 started at 9600");
}

void loop() {
  //Serial comms to HC-06
  while(Serial1.available()){
    c = Serial1.read();
    Serial.write(c);
  }
  while(Serial.available()) {
    c = Serial.read();

    Serial.write(c);
    Serial1.write(c);
  }
}
