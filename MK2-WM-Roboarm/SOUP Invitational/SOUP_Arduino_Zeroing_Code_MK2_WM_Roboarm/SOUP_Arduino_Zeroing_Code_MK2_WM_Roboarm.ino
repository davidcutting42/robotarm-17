/*  
 *  A simple utility program that will read the values of three AS5048B's connected over I2C and print their values to the Serial monitor for the purpouse of debugging/setup. 
 *  Written by David Cutting Feb. 3, 2017
 */
 
#include <Wire.h> // Library that contains framework for communication with AS5048Bs (encoders)
#include <ams_as5048b.h> // Library that communicates with encoders (uses Wire)

// Define constants for the encoders
#define U_DEG 3

// Construct encoder objects
//AMS_AS5048B encodera(0x01);
//AMS_AS5048B encoderb(0x02);
AMS_AS5048B encoderd(0x48);

void setup() {
  // Start encoder communication
  //encodera.begin();
  //encoderb.begin();
  encoderd.begin();

  Serial.begin(9600);
}

void loop() {
  //Serial.print("Elnncoder A: ");
  //Serial.println(encodera.angleR(U_DEG));
  //Serial.print("Encoder B: ");
  //Serial.println(encoderb.angleR(U_DEG));
  Serial.print("Encoder D: ");
  Serial.println(encoderd.angleR(U_DEG));
  delay(500);
}
