#include <Arduino.h>
#include <AccelStepper.h>

// definitions for rail stepper

#define stepPin 9
#define directionPin 8
#define buttonCWpin 11
#define buttonCCWpin  12
#define enablePin  10

#define motorSpeed 1200 // max 1800
#define motorAccel 10000 // 15000 for speed = 1800

boolean buttonCWpressed = false;
boolean buttonCCWpressed = false;
AccelStepper railStepper(1,stepPin,directionPin);
