#include <RailStepper_def.h>

// define functions
void readButtons();
void actOnButtons();
void testrailstepper();

void setup() {
  //  Serial.begin(9600);

  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(buttonCWpin, INPUT);
  pinMode(buttonCCWpin, INPUT);

  railStepper.setMaxSpeed(motorSpeed);
  railStepper.setSpeed(motorSpeed);
  railStepper.setAcceleration(motorAccel);

  // test rail
  testrailstepper();
  testrailstepper();
  testrailstepper();

  // disable output
   digitalWrite(enablePin, LOW);
}

void loop() {
 readButtons();
  if (buttonCCWpressed || buttonCWpressed == true) {
    digitalWrite(enablePin, HIGH);
    actOnButtons();
  } else {
    //  Serial.println("idle");
    digitalWrite(enablePin, LOW);
  }
}

void readButtons() {
 buttonCCWpressed = false;
 buttonCWpressed = false;

 if (digitalRead(buttonCWpin) == HIGH) {
 buttonCWpressed = true;
 // Serial.println("CW rotation");
 }
 if (digitalRead(buttonCCWpin) == HIGH) {
 buttonCCWpressed = true;
 // Serial.println("CCW rotation");
 }
}

void actOnButtons() {
 if (buttonCWpressed == true) {
 digitalWrite(directionPin, LOW);
 railStepper.setSpeed(motorSpeed);
 railStepper.runSpeed();
 }
 if (buttonCCWpressed == true) {
 digitalWrite(directionPin, HIGH);
 railStepper.setSpeed(-motorSpeed);
 railStepper.runSpeed();
 }
}

void testrailstepper() {
   delay(200);
    // enable output
  digitalWrite(enablePin, LOW);

  // test foward / backward motion
 unsigned long currentTime = millis();
 unsigned long previousTime = currentTime;
 digitalWrite(directionPin, LOW);
 while (currentTime - previousTime < 200) {
   digitalWrite(enablePin, HIGH);
   railStepper.setSpeed(motorSpeed);
   railStepper.runSpeed();
   currentTime = millis();
 }
 // delay(200);
 digitalWrite(enablePin, LOW);
 delay(200);
 currentTime = millis();
 previousTime = currentTime;
 digitalWrite(directionPin, HIGH);
 while (currentTime - previousTime < 200) {
 digitalWrite(enablePin, HIGH);
 railStepper.setSpeed(-motorSpeed);
 railStepper.runSpeed();
 currentTime = millis();
 }
 // delay(200);

 // disable output
 digitalWrite(enablePin, LOW);
}
