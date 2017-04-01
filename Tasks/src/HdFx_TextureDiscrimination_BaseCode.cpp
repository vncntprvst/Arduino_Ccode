// present then retract stimulus panel
// rotates stimulus between trial with random permutations
// monitor lick sensor
// deliver reward
// deliver air puff

#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h> // for I2C
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>

//define functions;
  void readbuttons();
  void actonbuttons();
  void moverail(int speed, int duration);
  void rotatestim();
  void getdatafromPC();
  boolean trialready();
  void trialwrapup(int duration);
  void rewardflush();
  void sendTTL(int TTLpin, int instruct);
  int maxdiffanalog();
  void reward(Adafruit_DCMotor* solenoid,int dur);
  void sendtoPC(int trialNum, int tType, int trialResult, int rewCount);
  void soundout(int instruct);
  void rotatepanel();

// Stepper 1: rail (controlled through ST-M5045)
  #define stepPin_SM1 9
  #define directionPin_SM1 8
  #define enablePin_SM1  10
  #define buttonFWpin 11
  #define buttonBWpin  12
  #define motorSpeed 1200 // max 1800, base 1200 (ST-M5045 set at 400 step per rev, 2 microsteps)
  #define motorAccel 10000 // 15000 for speed = 1800, 10000 for 12000
  unsigned long startLinearMoveTime = 0;
  AccelStepper railStepper(1,stepPin_SM1,directionPin_SM1);

// Stepper 2: stim (controlled through Polulu )
  #define stepPin_SM2 3
  #define directionPin_SM2 2
  #define sleepPin_SM2 4
  #define enablePin_SM2  5
  //#define resetPin 5
  AccelStepper stimStepper(1,stepPin_SM2,directionPin_SM2);

// Create the motor shield object with the default I2C address
  Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// reward solenoid on port M1
  Adafruit_DCMotor *rewardSolenoid = AFMS.getMotor(1);
// Air puff solenoid on port M2
  Adafruit_DCMotor *puffSolenoid = AFMS.getMotor(2);

// shield control variables
  boolean joyBehavior = true;
  boolean buttonCWpressed = false;
  boolean buttonCCWpressed = false;

// lick sensor variables
  #define piezoPin A0
  #define lickTTLPin 13
  #define sampleWindowSize 600
  #define vibThd 20
  // unsigned short int baseline = 1;
  unsigned short int piezoVal;
  short int touchDetect = 0;
  unsigned short int maxDiff;
  int lickCount = 0;

// session and trial variables
  #define trialTTLPin 12
  unsigned long trialMillis = 0;
  byte TrialSelectMode=0; // 0 -> random selection,
  //1 -> preset from computer, 2 -> block trial preset (see rotatepanel).
  int BlockSize=15; // this is the number of trials in each block
  int BlockPos=1; // where we are in the block of trials
  byte trialType = 0;
  byte SessionStatus[2] = {0, 0}; // 1/ Run [ON/OFF (1/0)] 2/ Reset (1)
  byte trialOutcome=0;
  int trialCount=0;
  int rewardCount=0;
  int Lrewtrig=0;
  boolean railMovedBack=false;
  boolean stimShuffle=false;

// texture panel definitions
  int curr_pos=0; // inital panel position, curr_pos < 100 means texture side -> go Left (trial type 1)
  int next_pos;
  byte rot_seq; // number of panel rotations

// audio definitions
  #define soundTriggerPin 11 // audio output

// other defs
  // #define ledPin 13
  unsigned long currentMillis = 0;
  boolean buttonFWpressed = false;
  boolean buttonBWpressed = false;
  boolean railControl = false;
  boolean initialization = false;
  boolean stimRotation = false;
  boolean railMotion = false;

void setup() {
  Serial.begin(57600);
  pinMode(directionPin_SM1, OUTPUT);
  pinMode(stepPin_SM1, OUTPUT);
  pinMode(enablePin_SM1, OUTPUT);
  pinMode(buttonFWpin, INPUT);
  pinMode(buttonBWpin, INPUT);
  //
  railStepper.setMaxSpeed(motorSpeed);
  railStepper.setSpeed(motorSpeed);
  railStepper.setAcceleration(motorAccel);
  digitalWrite(enablePin_SM1, HIGH); // enable output
  //
  //  DDRD = DDRD | B11111100;  // sets pins 2 to 7 as outputs
  //                    // without changing the value of pins 0 & 1 (serial com)
  //same as sequence below:
  pinMode(directionPin_SM2, OUTPUT);
  pinMode(stepPin_SM2, OUTPUT);
  pinMode(sleepPin_SM2, OUTPUT);
  //  pinMode(resetPin, OUTPUT);
  pinMode(enablePin_SM2, OUTPUT);
  //
  stimStepper.setMaxSpeed(2000);
  stimStepper.setAcceleration(500);

  //  PORTD = PORTD | B00010000;
  //  same as below:
  digitalWrite(sleepPin_SM2, HIGH);
  //  digitalWrite(resetPin, HIGH);
  digitalWrite(enablePin_SM2, LOW);
  delay(1); //just in case stepper 2 was in sleep mode

  // turn off current in solenoid coils
   rewardSolenoid->run(RELEASE);
   puffSolenoid->run(RELEASE);
}

void loop() {
  // getdatafromPC();
  currentMillis = millis(); //keep track of time
  if(SessionStatus[1] == 1){ // reset counters
      //    arrayinit();
      trialCount=0;
      rewardCount=0;
      Lrewtrig=0;
      // release step motors
    }

  //SessionStatus[0]=1;
  while (SessionStatus[0] == 0){ // session is OFF! Standby
   readbuttons();
   if (buttonCCWpressed || buttonCWpressed == true) {
     actonbuttons();
   } else {
     //  Serial.println("idle");
     digitalWrite(enablePin_SM1, LOW);
     getdatafromPC();
   }
  }

  // rewardSolenoid->run(RELEASE);    // make sure to close solenoids
  // puffSolenoid->run(RELEASE);

  if (SessionStatus[0] == 1){ // session is ON!
    // check panel has been retracted and shuffled
    if (trialready() ==  true) {
      // start trial!
      trialMillis = millis();
      sendTTL(trialTTLPin,1); // send TTL : begin trial
      trialCount=trialCount+1; //increment trial number
      // sendToPC(trialCount,int(trialType),rewardCount);
      // start moving panel forward
      digitalWrite(enablePin_SM1, HIGH);
      digitalWrite(directionPin_SM1, LOW);
      moverail(motorSpeed,1000);
      // listen for licks
      maxDiff = maxdiffanalog(); //peakAnalog();
      if (maxDiff>vibThd){
        sendTTL(lickTTLPin,0);
        lickCount += 1;
       }
       // check trial rules
       if (trialType == 1) { // go trial
         if (lickCount > 0) {// has licked
           rewardCount=rewardCount+1;
           reward(rewardSolenoid,40);
           trialOutcome = 1; // "hit"
         } else { // no lick
           trialOutcome = 2; // "miss" ... pwon pwon pwon
         }
       } else { // no-go trial
         if (lickCount > 0) { // has licked
           trialOutcome = 3; // "false alarm!"" :p
         } else {
           rewardCount=rewardCount+1;
           reward(rewardSolenoid,40);
           trialOutcome = 4; // "correct rejection" yeah !!
         }
       }
      sendtoPC(trialCount,int(trialType),int(trialOutcome),rewardCount); // result of current trial
      trialwrapup(2000);

      sendTTL(trialTTLPin,0); // end of trial

    }

  }

}

void readbuttons() {
 buttonBWpressed = false;
 buttonFWpressed = false;

 if (digitalRead(joyBehavior) == HIGH) {
   railControl = true;
   initialization = false;
 } else {
   railControl = false;
   initialization = true;
 }

 if (digitalRead(buttonFWpin) == HIGH) {
 buttonFWpressed = true;
 // Serial.println("Forward");
 }
 if (digitalRead(buttonBWpin) == HIGH) {
 buttonBWpressed = true;
 // Serial.println("Backward");
 }
}

void actonbuttons() {
 if (railControl == true) {
   digitalWrite(enablePin_SM1, HIGH);
   if (buttonFWpressed == true) {
     digitalWrite(directionPin_SM1, LOW);
     moverail(motorSpeed,100); // adjusting initial location
   } else if (buttonBWpressed == true) {
      digitalWrite(directionPin_SM1, HIGH);
      moverail(-motorSpeed,100); // adjusting initial location
    }
 }
 if (initialization == true) {
   // set up first trial
    if (buttonFWpressed == true) {
      trialwrapup(2000);
    }
   // or flush reward
     while (digitalRead(buttonBWpin) == HIGH){
       readbuttons();
       rewardflush();
     }
   }
}

void moverail(float speed, int duration) {
 // railMotion = true;
 startLinearMoveTime=millis();
 while ((startLinearMoveTime+duration)>millis()){
   railStepper.setSpeed(speed);
   railStepper.runSpeed();
 }
 // railMotion = false;
}

void rotatestim() {
  // if (curr_pos==0){ //initialize random starting panel position
  //   //        Serial.println("initialize random starting panel position");
  //     panelrotate();
  //     //        Serial.println(trialType);
  // }


  stimRotation = true ;
  stimStepper.moveTo(100);
  while (stimStepper.currentPosition() != 50) // Full speed up to 300
    stimStepper.run();
    stimStepper.stop(); // Stop as fast as possible: sets new target
    stimStepper.runToPosition();
    // Now stopped after quickstop
    //delay(500);

    // Now go backwards
  stimStepper.moveTo(-100);
  while (stimStepper.currentPosition() != -50) // Full speed basck to 0
    stimStepper.run();
    stimStepper.stop(); // Stop as fast as possible: sets new target
    stimStepper.runToPosition();
    // Now stopped after quickstop

    //  digitalWrite(sleepPin_SM2, LOW);
  stimRotation = false ;
}

boolean trialready() {
  // check if panel is retracted
  railMovedBack = true;
  // check if texture has been randomized
  stimShuffle = true;
  return (railMovedBack && stimShuffle);
}

int cumSumAnalog() {
  // see also runningMedian Class for Arduino: http://playground.arduino.cc/Main/RunningMedian
  //  float piezoArray[2] = {analogRead(piezoPin)};
  //  int piezoVal;
  //  int minMax;
  int cumVal = analogRead(piezoPin); // analogRead(piezoPin);
  for (int inc=0; inc < 9; inc++ ){
      //      piezoVal=analogRead(piezoPin);
        cumVal= cumVal + analogRead(piezoPin);
        //      if (piezoArray[0] > piezoVal) {
        //        piezoArray[0]=piezoVal;
        //        }
        //      peakVal = max(peakVal, piezoVal);
  }
    //  minMax=piezoArray[1]-piezoArray[0];
    //  Serial.print("peak to peak value is ");
    //  Serial.println(peakVal);
  return cumVal; //peakVal;
}

void sendTTL(int TTLpin, int instruct){
  digitalWrite(TTLpin, HIGH);
  delay(25);
  digitalWrite(TTLpin, LOW);
  switch (instruct) {
    case 1:
    // trial initiation: 2 TTL
    delay(25);
    digitalWrite(TTLpin, HIGH);
    delay(25);
    digitalWrite(TTLpin, LOW);
    break;
    case 2:
  // that's it
    break;
  }
}

int maxdiffanalog(){
  unsigned short int maxVal=0;
  unsigned short int minVal=1024;

  for(int i = 0; i < sampleWindowSize ; ++i){
    piezoVal = analogRead(piezoPin);
    minVal = min(minVal, piezoVal);
    maxVal = max(maxVal, piezoVal);
  }
  return abs(maxVal-minVal);
}

void reward(Adafruit_DCMotor* solenoid,int dur){
  solenoid->setSpeed(255);
  solenoid->run(FORWARD);
//    for (int dec=200; dec>170; dec-=decrease) { //max 255
//    solenoid->setSpeed(dec);
//    delay(15);
//    Serial.println(dec);
//   }
 delay(dur);
 solenoid->run(RELEASE); // cut power to motor
}

void sendtoPC(int trialNum, int tType, int trialResult, int rewCount) {
	// if (curMillis - prevReplyToPCmillis >= replyToPCinterval) {
	// 	prevReplyToPCmillis += replyToPCinterval;
	// 	int valForPC = curMillis >> 9; // approx half seconds
	// 	Serial.print('<');
	// 	Serial.print(valForPC);
	// 	Serial.print('>');
	// }

  Serial.print(trialNum);
  Serial.print(",");
  Serial.print(tType);
  Serial.print(",");
  Serial.print(trialResult);
  Serial.print(",");
  Serial.print(rewCount);
  Serial.println(",");
}

void trialwrapup(int timeout){ // needs to happen after sendToPC() command
  if (TrialSelectMode==2){ // time to move trial position one up (ie, setting up next trial type)
    BlockPos+=1;
    if (BlockPos<=BlockSize){
      trialType=1;
    } else if ((BlockPos>BlockSize) && (BlockPos<=2*BlockSize)){
      trialType=2;
    }
    if (BlockPos==2*BlockSize){
      BlockPos=0;
    }
//    Serial.print("End of trial. BlockPos moved to ");
//    Serial.println(BlockPos);
//    Serial.print("trialType = ");
//    Serial.println(trialType);
  }
  digitalWrite(enablePin_SM1, HIGH);
  digitalWrite(directionPin_SM1, LOW);
  moverail(motorSpeed,1000);
  soundout(1); // white noise mask
  rotatepanel(); // set for next trial

  // refractory period
  delay(timeout); // timeout to leave time for 2s white noise + extra timeout if wrong trial
}

void soundout(int instruct){
//HIGH triggers trinkets listening
//Serial.println("TTL out");
  switch (instruct) {
    // both cases should total 10ms
    case 1:
    // white noise
      digitalWrite(soundTriggerPin, HIGH); // trigger
// NB:
// if control PlayTone enabled in WhiteNoise_USbeep
// need to add a total of 10ms (or control beep's duration)
// to wait for if statment
      delay(2);
      digitalWrite(soundTriggerPin, LOW); // White noise
      delay(8);
      break;
    case 2:
      // US
      digitalWrite(soundTriggerPin, HIGH); // trigger
      delay(2);
      digitalWrite(soundTriggerPin, LOW);
      delay(2);
      digitalWrite(soundTriggerPin, HIGH); // +1
      delay(2);
      digitalWrite(soundTriggerPin, LOW);
      delay(4);
    break;
   }
}

void rotatepanel(){
  // Rotate panel for next trial
  if (TrialSelectMode==0){ // random rotation
    next_pos = random(1,201); // random number between 1 and 200 (included)
    if (next_pos<101){
      trialType=1;
      // Serial.print("panelrotate says trialType is ");
      // Serial.println(trialType);
    } else {
      trialType=2;
      // Serial.print("panelrotate says trialType is ");
      // Serial.println(trialType);
    }
//   else if (TrialSelectMode==1)  //instructions from computer
  } else if (TrialSelectMode==2) {
      if (trialType==1){// current trial within first block type
         next_pos = random(1,101);
    } else if (trialType==2){
         next_pos = random(101,201);
    }
  }

//  Serial.print("panelrotate next pos ");
//  Serial.println(next_pos);
  rot_seq = random(200); // random number between 0 and 1
//  Serial.print("panelrotate rot seq ");
//  Serial.println(rot_seq);

  if (((curr_pos < 101) && (next_pos < 101)) || ((curr_pos > 100) && (next_pos > 100))){
    if (rot_seq < 100 ) {
//      Serial.println("Rotate CW 1/4");
      // TexturePanelStepper->step(50, FORWARD, DOUBLE);
      delay(100);
//      Serial.println("Rotate CCW 1/4");
    //  TexturePanelStepper->step(50, BACKWARD, DOUBLE);
    } else {
//      Serial.println("Rotate CCW 1/4");
      // TexturePanelStepper->step(50, BACKWARD, DOUBLE);
      delay(100);
//      Serial.println("Rotate CW 1/4");
      // TexturePanelStepper->step(50, FORWARD, DOUBLE);
    }
  } else { //present different texture
    if (rot_seq < 100 ) {
//      Serial.println("Rotate CCW 1/4");
      // TexturePanelStepper->step(50, BACKWARD, DOUBLE);
      delay(100);
//      Serial.println("Rotate CCW 1/4");
      // TexturePanelStepper->step(50, BACKWARD, DOUBLE);
    } else {
//      Serial.println("Rotate CW 1/4");
      // TexturePanelStepper->step(50, FORWARD, DOUBLE);
      delay(100);
//      Serial.println("Rotate CW 1/4");
      // TexturePanelStepper->step(50, FORWARD, DOUBLE);
    }
  }
//    TexturePanelStepper->release();
  curr_pos = next_pos;
  // Serial.print("curr_pos is now ");
  // Serial.println(curr_pos);
}
