#include <Servo.h>

// Servo set up
Servo servoOne;
Servo servoTwo;
Servo servoThree;

void setup() {
  Serial.begin(9600);

  // Servo attaching to pins
  servoOne.attach(3);
  servoTwo.attach(5);
  servoThree.attach(6);
}

void loop() {

  /* 
  1) Checks if there is incoming data in serial from python servo controller
  2) If there is assign each number (0-180) received to which servo it will control
  */

  if (Serial.available() >= 3) {
    int servoOneInput = Serial.read();
    int servoTwoInput = Serial.read();
    int servoThreeInput = Serial.read();

    // Prints what each servo got as input
    Serial.print(" Servo1: ");
    Serial.print(servoOneInput);
    Serial.print(" Servo2: ");
    Serial.print(servoTwoInput);
    Serial.print(" Servo3: ");
    Serial.println(servoThreeInput);

    //Writes the angles inputed to each servo
    servoOne.write(servoOneInput);
    servoTwo.write(servoTwoInput);
    servoThree.write(servoThreeInput);
  }
}
