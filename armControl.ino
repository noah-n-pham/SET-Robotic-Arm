#include <Servo.h> // necessary for using servo


// Looked it up and it looks like this will take 2 servos so 
Servo armXServo; // pin 9
Servo armYServo; // pin 10

// servo pins
int xPin = 9;
int yPin = 10;

// initial positioning (centered) 
int targetX = 90;
int targetY = 90;

void setup() {
  // put your setup code here, to run once:
  // begin serial communication (9600 is apparently the standard value)
  Serial.begin(9600);

  // attach/enable servos
  armXServo.attach(xPin);
  armYServo.attach(yPin);
  
  // center servos
  armXServo.write(targetX);
  armYServo.write(targetY);
}

void loop() {
  // put your main code here, to run repeatedly:
  // if Serial is talking to us (there is something there), read in the position of the thing
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');

    // data will come as  "Theta1 Theta2"
    // parse to find spaces, substring to properly store 
    int theta1 = 0;
    int theta2 = 0;
    int spaces = 0;
    int commaIndex = data.indexOf(',');

    if (commaIndex > 0){
      // read the first part of data until the comma appears (theta1)
      String first = data.substring(0, commaIndex);

      // then read to the end after the comma (theta2)
      String second = data.substring(commaIndex + 1);

      // convert strings to integers
      int targetX = first.toInt();
      int targetY = second.toInt();

      // constrain the targets so they don't do stupid things
      // 0-180 looks like the max range of motion for a servo
      targetX = constrain(targetX, 0, 180); 
      targetY = constrain(targetY, 0, 180);

      // go there!
      armXServo.write(targetX);
      armYServo.write(targetY);
    }
  }

  // I think I'm supposed to put a delay here so the servos don't die
  delay(15);
}

