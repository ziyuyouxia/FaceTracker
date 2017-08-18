#include <Wire.h>
#include "PanTilt.h"


PanTilt arm;
const int I2C_ADDR = 0x04;

//volatile, as updated by interrupt while being used elsewhere
volatile float xPosition = 0.5;
volatile float yPosition = 0.5;
volatile int faceDetected = 0; // could do bool, but it's read in from Wire as an int
volatile bool newPositionData = false;

unsigned long oldScanTime = micros();

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADDR);
  Wire.onReceive(dataReceived);
  arm.init(); //set pins and timer etc.
}

void loop() {
  if (((micros() - oldScanTime) > 2000000) & (faceDetected == 0))
  {
    /*wait 2s to see if face is detected, then move to new position.
     * don't just use delay, because we want to respond to face detection
     * as soon as possible
     */
    
    arm.scan(); //move arm around until we see a face
    printArmPosition();
    oldScanTime = micros();
  }
  else if (faceDetected && newPositionData) //we have a face! Only want to update the arm position if we've got new info from the camera
  {
    //copy volatiles so they won't change during control loop
    float xPos = xPosition;
    float yPos = yPosition;
    
    Serial.print("x: ");
    Serial.print(xPos - 0.5);
    Serial.print(" | y: ");
    Serial.println(yPos - 0.5);

    arm.centre(xPos, yPos); //a control law to move the arm so that the detected face is in the centre of the camera
    newPositionData = false; // this will be set to true once new data comes through over I2C
//    printArmPosition();

    oldScanTime = micros(); //reset scan timer
    Serial.println("---");
  }
}

void dataReceived(int numBytes)
{
  /*two options: the first byte is the "address", which says what the remaining bytes are for.
    Address 0: a single byte saying if a face has been detected
    Address 1: 2 words giving the face centroid x and y co-ordinates in float 0-1 relative terms (scaled by 65535 to give an int)
  */
  int address = Wire.read();
  if ((numBytes == 2) && (address == 0)) {
    //we have a face when we didn't before: initialise some control stuff:
    arm.resetCtrl();
    faceDetected = Wire.read();
  }
  else if ((numBytes == 5) && (address == 1)) {
    xPosition = (float)((Wire.read() * 255) + Wire.read()) / 65535.0;
    yPosition = (float)((Wire.read() * 255) + Wire.read()) / 65535.0;
    newPositionData = true;
  }
  else {
    Serial.print("Invalid bytes: ");
    Serial.print(numBytes);
    while (Wire.available()) {
      Wire.read();
    }
  }
}

void printArmPosition() {
    float panAngle, tiltAngle;
    arm.getAngles(panAngle, tiltAngle);
    Serial.print("p: ");
    Serial.print(panAngle);
    Serial.print(" | t: ");
    Serial.println(tiltAngle);
}

