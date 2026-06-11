#include <Servo.h>

// Servo set up
Servo servoOne;
Servo servoTwo;
Servo servoThree;
Servo servoFour;

void setup() {
  Serial.begin(9600);

  // Servo attaching to pins with explicit pulse range
  servoOne.attach(3, 500, 2500); // base
  servoTwo.attach(5, 500, 2500); //bottom
  servoThree.attach(6, 500, 2500); //top
  servoFour.attach(9, 500, 2500); //gripper
}

void loop() {

  /* 
  1) Checks if there is incoming data in serial from python servo controller
  2) If there is assign each number (0-180) received to which servo it will control
  */

  if (Serial.available() >= 8) {

    byte servoOneBig = Serial.read();
    byte servoOneLittle = Serial.read();

    byte servoTwoBig = Serial.read();
    byte servoTwoLittle = Serial.read();

    byte servoThreeBig = Serial.read();
    byte servoThreeLittle = Serial.read();

    byte servoFourBig = Serial.read();
    byte servoFourLittle = Serial.read();

    // Print raw bytes separately (avoid second-argument overload)
    Serial.print("Raw bytes -> S1: ");
    Serial.print((int)servoOneBig);
    Serial.print(" ");
    Serial.print((int)servoOneLittle);
    Serial.print("  S2: ");
    Serial.print((int)servoTwoBig);
    Serial.print(" ");
    Serial.print((int)servoTwoLittle);
    Serial.print("  S3: ");
    Serial.println((int)servoThreeBig);
    Serial.print(" ");
    Serial.println((int)servoThreeLittle);
    Serial.print("  S4: ");
    Serial.println((int)servoFourBig);
    Serial.print(" ");
    Serial.println((int)servoFourLittle);

    int servoOneInput = twoByteCentiDegreeTooFloat(servoOneBig, servoOneLittle);
    int servoTwoInput = twoByteCentiDegreeTooFloat(servoTwoBig, servoTwoLittle);
    int servoThreeInput = twoByteCentiDegreeTooFloat(servoThreeBig, servoThreeLittle);
    int servoFourInput = twoByteCentiDegreeTooFloat(servoFourBig, servoFourLittle);

    // Prints what each servo got as input
    Serial.print("Decoded deg -> S1: ");
    Serial.print(servoOneInput);
    Serial.print("  S2: ");
    Serial.print(servoTwoInput);
    Serial.print("  S3: ");
    Serial.println(servoThreeInput);
    Serial.print("  S4: ");
    Serial.println(servoFourInput);

    //Writes the angles inputed to each servo
    floatServoDriver(servoOne, servoOneInput);
    floatServoDriver(servoTwo, servoTwoInput);
    floatServoDriver(servoThree, servoThreeInput);
    floatServoDriver(servoFour, servoFourInput);
  }
}
