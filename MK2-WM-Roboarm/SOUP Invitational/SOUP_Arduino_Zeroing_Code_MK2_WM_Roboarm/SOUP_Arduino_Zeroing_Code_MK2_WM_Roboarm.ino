/*  
 *  A simple utility program that will read the values of three AS5048B's connected over I2C and print their values to the Serial monitor for the purpouse of debugging/setup. 
 *  Written by David Cutting Feb. 3, 2017
 */
 
#include <Wire.h> // Library that contains framework for communication with AS5048Bs (encoders)
#include <ams_as5048b.h> // Library that communicates with encoders (uses Wire)

// Define constants for the encoders
#define U_RAW 1
#define U_DEG 3

// Construct encoder objects
AMS_AS5048B encodera(0x44);
//AMS_AS5048B encoderb(0x48);
//AMS_AS5048B encoderd(0x4C);

void setup() {
  // Start encoder communication
  encodera.begin();
  //encoderb.begin();
  //encoderd.begin();

  Serial.begin(9600);
}

void loop() {
  Serial.print("Encoder A: ");
  Serial.println(encodera.angleR(U_DEG));
  //Serial.print("Encoder B: ");
  //Serial.println(encoderb.angleR(U_RAW));
  //Serial.print("Encoder D: ");
  //Serial.println(encoderd.angleR(U_RAW));
  delay(500);
}
