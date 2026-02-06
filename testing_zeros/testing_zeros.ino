#include <VarSpeedServo.h>

VarSpeedServo FLE; VarSpeedServo FRE; VarSpeedServo BRE; VarSpeedServo BLE;
VarSpeedServo FLS; VarSpeedServo FRS; VarSpeedServo BRS; VarSpeedServo BLS;



void setup() {e:
  FLE.attach(3); FRE.attach(6); BRE.attach(9); BLE.attach(12);
  FLS.attach(4); FRS.attach(7); BRS.attach(10); BLS.attach(13);
  
  FLE.write(180, 30);
  FRE.write(10, 30);
  BRE.write(10, 30);
  BLE.write(180, 30);

  FLS.write(90, 30);
  FRS.write(85, 30);
  BRS.write(85, 30);
  BLS.write(95, 30);

}

void loop() {
  // put your main code here, to run repeatedly:

}
