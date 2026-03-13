#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

bool gripperAttached = false;
Servo gripperServo;

// RDS51150SG servo: 500-2500μs = 270°, center at 1500μs
const int US_MIN = 500;
const int US_MAX = 2500;

void setup() {
  Serial.begin(9600);
  Serial.println("Program start: success");

  servo1.attach(2, US_MIN, US_MAX);
  servo2.attach(3, US_MIN, US_MAX);
  servo3.attach(4, US_MIN, US_MAX);

  // Uncomment the next line ONLY if you have a gripper servo wired to pin 5
  // gripperServo.attach(5); gripperAttached = true;

  // 270° servos: 0°→500μs, 135°(center)→1500μs, 270°→2500μs
  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);
  delay(2000);

  Serial.println("Servos centered: success");
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    if (data == "GRIP") {
      Serial.println("Grip command received");
      if (gripperAttached) {
        gripperServo.writeMicroseconds(2000);
        delay(500);
      }
      Serial.println("Gripper activated: success");
      return;
    }

    Serial.println("Angles received: success");

    int commaIndex = data.indexOf(',');
    int commaIndex2 = data.indexOf(',', commaIndex + 1);

    if (commaIndex > 0 && commaIndex2 > 0) {
      String first = data.substring(0, commaIndex);
      String second = data.substring(commaIndex + 1, commaIndex2);
      String third = data.substring(commaIndex2 + 1);

      Serial.println("Angles parsed: success");

      int theta1 = first.toInt();
      int theta2 = second.toInt();
      int theta3 = third.toInt();

      theta1 = constrain(theta1, 0, 270);
      theta2 = constrain(theta2, 0, 270);
      theta3 = constrain(theta3, 0, 270);

      int us1 = map(theta1, 0, 270, US_MIN, US_MAX);
      int us2 = map(theta2, 0, 270, US_MIN, US_MAX);
      int us3 = map(theta3, 0, 270, US_MIN, US_MAX);

      Serial.println("Angles reframed: success");

      servo1.writeMicroseconds(us1);
      servo2.writeMicroseconds(us2);
      servo3.writeMicroseconds(us3);
      delay(15);

      Serial.println("Arm moved: success");
    }
  }
}
