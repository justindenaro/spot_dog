String sendstr;
int axis[6] = {};
int newaxis[6] = {};
int request = 0;

void setup() {
  Serial.begin(38400);
  // Serial connection w/ dog
  Serial1.begin(38400);

  //Reads all 6 joystick values
  for(int i = 0; i < 6; i++) {
    axis[i] = getaxes(i);
  }
  senddata();   //Sends all data initally to create gsv data
}

void loop() {
  //Reads all 6 joystick values
  //Only assigns joystick readings to axis vector if
  //different from existing axis vector (i.e. only accepts changes)
  while(Serial1.available() == 0) {}
  request = Serial1.read();
  for(int i = 0; i < 6; i++) {
    axis[i] = getaxes(i);
  }
  if(request == 69){
    senddata();   //Sends all data when requested
  } else {}
}

int getaxes(int pin) {
  int read = 0;
  read = analogRead(pin);
  if(pin == 1) {
    read = map(read,1023,0,0,100);
  } else {
    read = map(read,0,1023,0,100);
  }
  return read;
}

//Sends all data at once
void senddata() {
  //Concatenates start char
  int connect = 11;
  sendstr = "s";
  sendstr = sendstr + connect + "|";

  //Concatenates all axis to end
  for(int i = 0; i < 6; i++) {
    sendstr = sendstr + axis[i] + "|";
  }

  sendstr = sendstr + "e";      //Ending code

  //Concatenate sendstr with more data as needed

  //Sends final string
  Serial.println(sendstr);
  Serial1.print(sendstr);

  // delay(100);   //Allows /second refresh on robot. Can make faster if desired
}