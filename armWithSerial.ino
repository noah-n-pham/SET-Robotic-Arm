#include <Servo.h> // necessary for using servo


Servo servo1;
Servo servo2;
Servo servo3;




void setup() {
  // put your setup code here, to run once:
  // begin serial communication (9600 is apparently the standard value)
  Serial.begin(9600);


  Serial.println("Program start: success");
  //servo1 is the rotation
  servo1.attach(2);
  //servo2 corresponds to the base attachment
  servo2.attach(3);
  //servo3 corresponds to the joint connecting the top arm
  servo3.attach(4);


  Serial.println("Servos centered: success");




}


void loop() {


  // put your main code here, to run repeatedly:
  // go to start position (RUN THIS FIRST)
  servo1.writeMicroseconds(1000);
  servo2.writeMicroseconds(1300);
  servo3.writeMicroseconds(1100);
  delay(1000);
  // if Serial is talking to us (there is something there), read in the position of the thing
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');


    Serial.println("Angles received: success");
    int theta1 = 0;
    int theta2 = 0;
    int theta3 = 0;
    int spaces = 0;
    int commaIndex = data.indexOf(',');
    int commaIndex2 = data.indexOf(',', commaIndex + 1);


    String first = "";
    String second = "";
    String third = "";
    if ((commaIndex > 0) && (commaIndex2 > 0)){
      first = data.substring(0, commaIndex);
      second = data.substring(commaIndex + 1, commaIndex2);
      third = data.substring(commaIndex2 + 1);
    }
   
    Serial.println("Angles parsed: success");
    theta1 = first.toInt();
    theta2 = second.toInt();
    theta3 = third.toInt();


    theta1 = constrain(theta1, 0, 180);
    theta2 = constrain(theta2, 0, 180);
    theta3 = constrain(theta3, 0, 180);


    theta1 = (theta1 * 7.40740741) + 500;
    theta2 = (theta2 * 7.40740741) + 1500;
    theta3 = (theta3 * 7.40740741) + 2100;


    Serial.println("Angles reframed: success");
    int pos1 = servo1.readMicroseconds();
    int pos2 = servo2.readMicroseconds();
    int pos3 = servo3.readMicroseconds();
    servo1.writeMicroseconds(theta1);
    delay(1000);
    servo2.writeMicroseconds(theta2);
    delay(1000);
    servo3.writeMicroseconds(theta3);
    delay(1000);
    Serial.println("Arm moved: success");
  }
}
