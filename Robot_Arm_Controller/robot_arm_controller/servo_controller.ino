#include <Servo.h>

// This function recives two bytes which make up one angle in centidegrees (byte1 and byte2) and returns the combined float
float twoByteCentiDegreeTooFloat(byte byte1, byte byte2){
  // promote to int before shifting so the high byte isn't lost
  int high = ((int)byte1) << 8;
  int centiDegreeByte = high | (int)byte2;
  float centiDegreeFloat = (float)centiDegreeByte;

  // Converts centidegrees to degrees
  float degreeFloat = centiDegreeFloat / 100.0;

  return degreeFloat;
}


void floatServoDriver(Servo &servo, float degree){
   // Map degrees (0-180) to microseconds (500-2500µs)
   int microseconds = map(degree, 0, 180, 500, 2500);
  
   servo.writeMicroseconds(microseconds);
   Serial.println(microseconds);
   delay(100);
}
