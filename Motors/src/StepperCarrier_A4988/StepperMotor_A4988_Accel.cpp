#include <Arduino.h>
#include <AccelStepper.h>

const byte directionPin = 2;
const byte stepPin = 3;
AccelStepper stimStepper(1,stepPin,directionPin);

const byte sleepPin = 4;
//const byte resetPin = 5;
const byte enablePin = 5;
const byte ledPin = 13;

void setup()
{
  stimStepper.setMaxSpeed(2000);
  stimStepper.setAcceleration(500);

//  DDRD = DDRD | B11111100;  // sets pins 2 to 7 as outputs
//                    // without changing the value of pins 0 & 1 (serial com)
//same as sequence below:
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(sleepPin, OUTPUT);
//  pinMode(resetPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

//  PORTD = PORTD | B00110000;
//  same as below:
  digitalWrite(sleepPin, HIGH);
//  digitalWrite(resetPin, HIGH);
  digitalWrite(enablePin, LOW);
  delay(1); //just in case it was in sleep mode
}

void loop(){
//  delay(500);
  stimStepper.moveTo(100);
  while (stimStepper.currentPosition() != 50) // Full speed up to 300
    stimStepper.run();
    stimStepper.stop(); // Stop as fast as possible: sets new target
    stimStepper.runToPosition();
//  // Now stopped after quickstop
    digitalWrite(ledPin, !digitalRead(ledPin));

    digitalWrite(enablePin, HIGH);
    delay(50);
    digitalWrite(enablePin, LOW);

  // Now go backwards
  stimStepper.moveTo(-100);
  while (stimStepper.currentPosition() != -50) // Full speed basck to 0
    stimStepper.run();
    stimStepper.stop(); // Stop as fast as possible: sets new target
    stimStepper.runToPosition();
//  // Now stopped after quickstop
    digitalWrite(ledPin, !digitalRead(ledPin));

    digitalWrite(enablePin, HIGH);
    delay(50);
    digitalWrite(enablePin, LOW);

  digitalWrite(sleepPin, LOW);
}
