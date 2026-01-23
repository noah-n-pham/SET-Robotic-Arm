#include <Servo.h>
Servo servo1;
Servo servo2;
Servo servo3;

// straight up 180 deg is 1000 (s1), 1500 (s2), 2100 (s3)
// touching the ground is 1000 (s1), 2000 (s2), 1500 (s3)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Program started");
  //servo1 is the rotation
  servo1.attach(2);
  //servo2 corresponds to the base attachment
  servo2.attach(3);
  //servo3 corresponds to the joint connecting the top arm
  servo3.attach(4);
}



void loop() {
  // put your main code here, to run repeatedly:

  // go to start position (RUN THIS FIRST)
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(2100);
  delay(50);

  // read position
  int pos1 = servo1.readMicroseconds();
  int pos2 = servo2.readMicroseconds();
  int pos3 = servo3.readMicroseconds();

  // until position = target, move to target (use a while loop)
}

