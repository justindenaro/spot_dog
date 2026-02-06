#include <VarSpeedServo.h>

VarSpeedServo FLW; VarSpeedServo FLE; VarSpeedServo FLS; //Front left servo assignments
VarSpeedServo FRW; VarSpeedServo FRE; VarSpeedServo FRS; //Front right servo assignments
VarSpeedServo BRW; VarSpeedServo BRE; VarSpeedServo BRS; //Back right servo assignments
VarSpeedServo BLW; VarSpeedServo BLE; VarSpeedServo BLS; //Back left servo assignments

double l1 = 110.0;     //Length of upper arm
double l2 = 137.0;     //Length of lower arm
double h = l1 + l2;    //Length of arm if fully extended
double l3 = 39.0;    //Half width of robot in mm
double dh;       //Desired height of robot
double dl;       //Desired left/right shimmy
int theta1;   //Angle in crack of elbow
int theta2;   //Angle between vertical line and upper arm
int theta3;   //Angle offset from vertical that the legs are
// int theta4;   //Shoulder shrug angle
int fbm = 0;      //Forward/backward movement
int lrm = 0;      //Left/right movement
int tlrm = 0;      //Turn left/right mevement
int stroke = 0;   //Tracker of walking stroke movement
int cyc = 0;   //Tracks cycle number (0, 1, or 2)
int dir = 1;  //Forward/backwards direction of walk


int ADCin[12];
int startpos[12] = {0, 180, 90, 180, 10, 85, 180, 10, 85, 0, 180, 95}; //Easy start position on belly
int ref[12] = {145, 180, 90, 35, 10, 85, 35, 10, 85, 155, 180, 95};   //Reference position for easy geometry
// int aref[12] = {};  //Acutal standing reference position (only difference is wrist/elbows)
int dw[4] = {};
int de[4] = {};
int ds[4] = {};

int connect = 0; //Connect status
int remote_axis[6] = {};
int ax1; int ax2; int ax3; int ax4; int ax5; int ax6; //Remote axes
String readstr;

void setup() {
  Serial.begin(38400);
  // Serial for BT control w/ remote
  Serial1.begin(38400);

  coldstart();
}

void loop() {
  //Tests connection and doesn't move robot unless connected to remote
  get_remote_data();
  if(connect == 0){
    do {
      get_remote_data();
    } while(connect == 0);
  }
  math(ax1, ax2, ax3, ax4);  //Do math based on remot axis imputs

  if(fbm > 0) {
    walk();   // Does one walk cycle
    Serial.println("Finished one walk cycle");
  } else if (fbm == 0 && cyc == 1) {
    cyc = 2; // Flag for cleanup cycle
    walk();   // Cleaning up last 3 steps
    Serial.println("Finished cleaning up walk cycle");
  } else {
    elbow(de[0], de[1], de[2], de[3], 255, 0);    //Move elbows
    wrist(dw[0], dw[1], dw[2], dw[3], 255, 0);    //Move wrists
    shoulder(ds[0], ds[1], ds[2], ds[3], 255, 0);  //Move shoulders 
  }
}

//Does math for angles, positions
void math(int ax1, int ax2, int ax3, int ax4) {
  double t1;
  // double t1r;
  // double t1l;
  double t2;
  // double t2r;
  // double t2l; 
  double t3;
  // double t4;
  // double dhr;
  // double dhl;

  dh = map(ax1, 0, 100, 3*h/8, 7*h/8);  //mapping ax1 to h values to avoid dividing by zero - THESE FRACTIONS SET HEIGHT (AXIS 1) TRAVEL RANGE
  
  //Mapping axis 2 to +/- 80 mm
  if(ax2 >= 50) {   //If joystick is to the right
    dl = map(ax2, 50, 100, 0, 80); 
    t3 = atan(dl / dh);
    theta3 = t3 * 180 / PI;

    ds[0] = ref[2] - theta3;
    ds[1] = ref[5] - theta3;
    ds[2] = ref[8] + theta3;
    ds[3] = ref[11] + theta3;
  } else if(ax2 < 50) {   //If joystick is to the left
    dl = map(ax2, 49, 0, 0, 80);
    t3 = atan(dl / dh);
    theta3 = t3 * 180 / PI;

    ds[0] = ref[2] + theta3;
    ds[1] = ref[5] + theta3;
    ds[2] = ref[8] - theta3;
    ds[3] = ref[11] - theta3;
  }

  //Final angle, joint determinations
  t1 = acos((pow(l2, 2.0) + pow(l1, 2.0) - pow(dh, 2.0)) / (2.0*l1*l2));
  theta1 = t1 * 180 / PI;
  t2 = acos((pow(l1, 2.0) + pow(dh, 2.0) - pow(l2, 2.0)) / (2.0*l1*dh));
  theta2 = t2 * 180 / PI;

  dw[0] = theta1 - (180 - ref[0]);
  dw[1] = ref[3] + (180 - theta1);
  dw[2] = ref[6] + (180 - theta1);
  dw[3] = theta1 - (180 - ref[9]);

  de[0] = theta2 + 10;  //Offset to match right side
  de[1] = 180 - theta2;
  de[2] = 180 - theta2;
  de[3] = theta2 + 10; //Offset to match right side

  // Mapping axis 4 to forward/backward movement
  if(ax4 <= 40) {   //If joystick is to the forward (this is the backwards channel)
    fbm = map(ax4, 40, 0, 1, 255);   //Maps joystick to servo movement speed
    dir = -1;
  } else if(ax4 >= 60) {   //If joystick is to the backward
    fbm = map(ax4, 60, 100, 1, 255);   //Maps joystick to servo movement speed
    dir = 1;
  }else {
    fbm = 0;
  }  
  return;
}

//Walk function
void walk() {
  double x[2] = {0, -0.4*dh*dir};       //Predetermined array of x positions for walk stroke
  double y[2] = {0, 0.3*dh};     //Predetermined array of y positions for walk stroke
  double s[1] = {0};      //Shoulder position vector (not used now but here if desired)
  int speed = 255;   // Speed requested of legs to move

  double BLX[8] = {x[0], x[0], x[0], x[0], x[1], x[1], x[1], x[0]};
  double FLX[8] = {x[0], x[1], x[1], x[0], x[0], x[0], x[0], x[0]};
  double FRX[8] = {x[0], x[0], x[0], x[0], x[1], x[1], x[1], x[0]};
  double BRX[8] = {x[0], x[1], x[1], x[0], x[0], x[0], x[0], x[0]};

  // Y components
  double BLY[8] = {y[1], y[1], y[0], y[0], y[0], y[0], y[1], y[0]};
  double FLY[8] = {y[0], y[0], y[0], y[1], y[1], y[0], y[0], y[0]};
  double FRY[8] = {y[1], y[1], y[0], y[0], y[0], y[0], y[1], y[0]};
  double BRY[8] = {y[0], y[0], y[0], y[1], y[1], y[0], y[0], y[0]};

  // Shoulder components (not used rn but could be used to refine gait)
  double BLS[8] = {s[0], s[0], s[0], s[0], s[0], s[0], s[0], s[0]};
  double FLS[8] = {s[0], s[0], s[0], s[0], s[0], s[0], s[0], s[0]};
  double FRS[8] = {s[0], s[0], s[0], s[0], s[0], s[0], s[0], s[0]};
  double BRS[8] = {s[0], s[0], s[0], s[0], s[0], s[0], s[0], s[0]};

  if (cyc == 0 or cyc == 1) {
    do {
      cartesian(BLX[stroke], BLY[stroke], BLS[stroke], 4);
      cartesian(FLX[stroke], FLY[stroke], FLS[stroke], 1);
      cartesian(BRX[stroke], BRY[stroke], BRS[stroke], 3);
      cartesian(FRX[stroke], FRY[stroke], FRS[stroke], 2);

      //Change position as necessary after each stroke
      //shoulder(ds[0], ds[1], ds[2], ds[3], 0, 0);
      //delay(100);

      // No forced wait for elbow since it's first to move
      elbow(de[0], de[1], de[2], de[3], speed, 0);    //Move elbows
      //delay(200);

      // Forced wait for wrist to ensure movements complete
      wrist(dw[0], dw[1], dw[2], dw[3], speed, 0);    //Move wrist
      //delay(200);
      stroke++;
      // Delaying between each step
      if(stroke == 2 || stroke == 5) {
        delay (100);
      }
    } while (stroke < 6);
    cyc = 1;  //Alerting that a cycle has been done
    stroke = 0;
  } else if (cyc == 2) {
    stroke = 6;
    do {
      cartesian(BLX[stroke], BLY[stroke], BLS[stroke], 4);
      cartesian(FLX[stroke], FLY[stroke], FLS[stroke], 1);
      cartesian(BRX[stroke], BRY[stroke], BRS[stroke], 3);
      cartesian(FRX[stroke], FRY[stroke], FRS[stroke], 2);

      //Change position as necessary after each stroke
      //shoulder(ds[0], ds[1], ds[2], ds[3], 0, 0);
      //delay(100);

      // No forced wait for elbow since it's first to move
      elbow(de[0], de[1], de[2], de[3], s, 0);    //Move elbows
      //delay(200);

      // Forced wait for wrist to ensure movements complete
      wrist(dw[0], dw[1], dw[2], dw[3], s, 1);    //Move wrist
      //delay(200);
      stroke++;
    } while (stroke < 9);
    cyc = 0;
    stroke = 0;
  }

  return;
}

//Function to determine dw[], de[] and ds[] from x, y, leg values
void cartesian(double x, double y, double s, int leg) {
  double dhnew;
  double t1;
  double t2;
  double tt;
  int thetat;
  int thetaf;

  // Triangle that x, y coordinates make
  if (y >= 0) {
    y = dh - y;
  } else {
    y = dh + y;
  }
  tt = atan(abs(x) / y);
  thetat = tt * 180 / PI;
  dhnew = sqrt(pow(x, 2.0) + pow(y, 2.0));

  Serial.print("x: ");
  Serial.println(x);
  Serial.print("y: ");
  Serial.println(y);
  Serial.print("dhnew: ");
  Serial.println(dhnew);

  t1 = acos((pow(l2, 2.0) + pow(l1, 2.0) - pow(dhnew, 2.0)) / (2.0*l1*l2));
  theta1 = t1 * 180 / PI;
  Serial.print("theta1: ");
  Serial.println(theta1);
  t2 = acos((pow(l1, 2.0) + pow(dhnew, 2.0) - pow(l2, 2.0)) / (2.0*l1*dhnew));
  theta2 = t2 * 180 / PI;
  Serial.print("theta2: ");
  Serial.println(theta2);

  if(x >= 0) {
    thetaf = theta2 - thetat;
  }else if(x < 0) {
    thetaf = theta2 + thetat;
  }

  //Assigning dw[], de[] , ds[] based on leg
  // Front Left Leg
  if(leg == 1) {
    dw[0] = theta1 - (180 - ref[0]);
    de[0] = thetaf + 10;  //Offset to match right side
    ds[0] = ref[2] + s;
  // Front Right Leg
  }else if(leg == 2) {
    dw[1] = ref[3] + (180 - theta1);
    de[1] = 180 - thetaf;
    ds[1] = ref[5] + s;
  // Back Right Leg
  }else if(leg == 3) {
    dw[2] = ref[6] + (180 - theta1);
    de[2] = 180 - thetaf;
    ds[2] = ref[8] - s;
  // Back Left Leg
  }else if(leg == 4) {
    dw[3] = theta1 - (180 - ref[9]);
    de[3] = thetaf + 10; //Offset to match right side
    ds[3] = ref[11] - s;
  }
  return;
}


//Always initial read, attachment and assignment
void coldstart() {
  //Reads all servos initally (deg)
  int i = 0;
  for(int temp = 2; temp < 14; temp++) {
    ADCin[i] = read_pins(temp);
    i++;
  }
  
  //Write to initial read ADC value
  FLW.write(ADCin[0]); FLE.write(ADCin[1]); FLS.write(ADCin[2]);
  FRW.write(ADCin[3]); FRE.write(ADCin[4]); FRS.write(ADCin[5]);
  BRW.write(ADCin[6]); BRE.write(ADCin[7]); BRS.write(ADCin[8]);
  BLW.write(ADCin[9]); BLE.write(ADCin[10]); BLS.write(ADCin[11]);

  //Attach servo objects to pins
  FLW.attach(2); FLE.attach(3); FLS.attach(4);
  FRW.attach(5); FRE.attach(6); FRS.attach(7);
  BRW.attach(8); BRE.attach(9); BRS.attach(10);
  BLW.attach(11); BLE.attach(12); BLS.attach(13);

  //Slow move from current to zero[] position
  shoulder(startpos[2], startpos[5], startpos[8], startpos[11], 15, 0);
  elbow(startpos[1], startpos[4],  startpos[7], startpos[10], 15, 0);
  wrist(startpos[0], startpos[3], startpos[6], startpos[9], 15, 0);
  return;
}

//Moves wrists to desired location
void wrist(int d1, int d2, int d3, int d4, int speed, int wait) {
  int des[4] = {d1, d2, d3, d4};
  // Serial.println("Started wrist");
  //Moves wrists at desired speed waits for finish
  
  FLW.write(des[0], speed);
  FRW.write(des[1], speed);
  BRW.write(des[2], speed);
  BLW.write(des[3], speed);
  if(wait == 1) {
    FLW.wait();
    FRW.wait();
    BRW.wait();
    BLW.wait();
  }
  // Serial.println("finished wrist");
  return;
}

//Moves elbows to desired location
void elbow(int d1, int d2, int d3, int d4, int speed, int wait) {
  int des[4] = {d1, d2, d3, d4};
  // Serial.println("Started elbow");
  //Moves elbows at desired speed waits for finish
  FLE.write(des[0], speed);
  FRE.write(des[1], speed);
  BRE.write(des[2], speed);
  BLE.write(des[3], speed);
  if(wait == 1) {
    FLE.wait();
    FRE.wait();
    BRE.wait();
    BLE.wait();
  }

  // Serial.println("Finished elbow");
  return;
}

//Moves shoulders to desired location
void shoulder(int d1, int d2, int d3, int d4, int speed, int wait) {
  int des[4] = {d1, d2, d3, d4};
  
  //Moves shoulders at desired speed waits for finish
  FLS.write(des[0], speed);
  FRS.write(des[1], speed);
  BRS.write(des[2], speed);
  BLS.write(des[3], speed);
  if(wait == 1) {
    FLS.wait();
    FRS.wait();
    BRS.wait();
    BLS.wait();
  }

  return;
}

//Getting remote data
void get_remote_data() {
  String temp;
  Serial1.write(69);
  delay(10);

  while(Serial1.available() == 0) {}
  readstr = Serial1.readStringUntil('e');  //All of string at once
  readstr.trim();
  // Serial.println(readstr);
  
  //Finding all of our marker characters
  int startsignal = readstr.indexOf('s');
  int first = readstr.indexOf('|');
  int second = readstr.indexOf('|', first+1);
  int third = readstr.indexOf('|', second+1);
  int fourth = readstr.indexOf('|', third+1);
  int fifth = readstr.indexOf('|', fourth+1);
  int sixth = readstr.indexOf('|', fifth+1);
  int seventh = readstr.indexOf('|', sixth+1);
  int markerarray[7] = {first, second, third, fourth, fifth, sixth, seventh};

  //Parses connection value
  temp = readstr.substring((startsignal+1), first);
  connect = temp.toInt();

  //Parses data from remote axes
  for(int i = 0; i < 6; i++) {
    temp = readstr.substring((markerarray[i]+1), markerarray[i+1]);  //Actual first value
    remote_axis[i] = temp.toInt();
  }

  //Continue something else if more data is sent from remote

  //Rearranging axis array to correct variable because I'm lazy and didn't re-pin the remote
  ax1 = remote_axis[5];
  ax2 = remote_axis[4];
  ax3 = remote_axis[3];
  ax4 = remote_axis[1];
  ax5 = remote_axis[0];
  ax6 = remote_axis[2];
  // Serial.println(ax4);
  return;
}

//Reads and returns one corrected ADC value in degrees
int read_pins(int pin) {
  int read = 0;
  read = analogRead(pin);
  read = map(read, 525, 155, 0, 180);
  read = constrain(read, 0, 180);
  return read;
}
