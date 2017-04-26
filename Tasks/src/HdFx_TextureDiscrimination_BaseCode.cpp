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
  void moverail(int speed, uint8_t direction, int duration);
  void getdatafromPC();
  boolean trialready();
  void trialwrapup(int duration);
  void rewardflush();
  void sendTTL(int TTLpin, int instruct);
  int maxdiffanalog();
  void reward(Adafruit_DCMotor* solenoid,int dur);
  void sendtoPC(int trialNum, int tType, int trialResult, int rewCount);
  void soundout(int instruct);
  boolean isRailRetracted();
  void trialselection();
  void rotatepanel(long relativePos, int rotDirection,  int pauseDuration);

// Stepper 1: rail (controlled through ST-M5045)
  #define stepPin_SM1 9
  #define directionPin_SM1 8
  #define enablePin_SM1  10
  #define buttonFWpin 11
  #define buttonBWpin  12
  #define railMotorSpeed 1200
  #define railMotorAccel 10000
  unsigned long startLinearMoveTime = 0;
  AccelStepper railStepper(1,stepPin_SM1,directionPin_SM1);

// Stepper 2: stim (controlled through Polulu )
  #define stepPin_SM2 3
  #define directionPin_SM2 2
  #define sleepPin_SM2 4
  #define enablePin_SM2  5
  //#define resetPin 5
  #define stepsPerRev_SM2 200 // Stepper motor - NEMA-17 - 200 steps/rev, https://www.adafruit.com/product/324
  #define panelStepDifferential 40 // 5 stims=> stepsPerRev_SM2/5
  #define stimMotorSpeed 2000 // max 1800, base 1200 (ST-M5045 set at 400 step per rev, 2 microsteps)
  #define stimMotorAccel 500 // 15000 for speed = 1800, 10000 for 12000
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
  #define lickTTLPin 14
  #define sampleWindowSize 600
  #define vibThd 20
// byte baseline = 1;
  unsigned int piezoVal; // max 1024
  unsigned int maxDiff;
  unsigned int lickCount = 0;

// session and trial variables
  #define trialTTLPin 12
  unsigned long trialMillis = 0;
  byte TrialSelectMode=0; // 0 -> randomized trials
                          // 1 -> trial-by-tria instrucitons from computer
                          // 2 -> block trial preset (see trialselection).
  byte BlockSize=15; // this is the number of trials in each block
  byte BlockPos=1; // where we are in the block of trials
  byte trialType = 0;
  byte SessionStatus[2] = {0, 0}; // 1/ Run [ON/OFF (1/0)] 2/ Reset (1)
  byte trialOutcome=0;
  unsigned int trialCount=0;
  unsigned int TrialInit=0;
  unsigned int gracePeriod=1000;
  unsigned int responseWindow=1000;
  unsigned int rewardCount=0;
  boolean railMovedBack=false;
  boolean stimShuffle=false;

// texture panel definitions
  #define railPosIRpin A1
  int currentTrialType=0; // inital panel position
  int nextTrialType;
  byte rot_seq; // number of panel rotations
  int rotationAngle;

// audio definitions
  #define soundTriggerPin 16 // audio output

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
  randomSeed(analogRead(0));
  //shield controls
  pinMode(buttonFWpin, INPUT);
  pinMode(buttonBWpin, INPUT);
  // rail SM setup
  pinMode(directionPin_SM1, OUTPUT);
  pinMode(stepPin_SM1, OUTPUT);
  pinMode(enablePin_SM1, OUTPUT);
  railStepper.setMaxSpeed(railMotorSpeed);
  railStepper.setSpeed(railMotorSpeed);
  railStepper.setAcceleration(railMotorAccel);
  digitalWrite(enablePin_SM1, LOW); // disable output
  // stimulus panel - SM setup
  pinMode(directionPin_SM2, OUTPUT);
  pinMode(stepPin_SM2, OUTPUT);
  pinMode(sleepPin_SM2, OUTPUT);
  pinMode(enablePin_SM2, OUTPUT);
  stimStepper.setMaxSpeed(stimMotorSpeed);
  stimStepper.setAcceleration(stimMotorAccel);

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
      lickCount=0;
      rewardCount=0;
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
    if (trialready() ==  true) { // check that panel has been retracted and shuffled
      // start trial!
      if (TrialInit==0){ // initialize on first pass
        trialMillis = millis();
        sendTTL(trialTTLPin,1); // send TTL : begin trial
        trialCount=trialCount+1; //increment trial number
        // sendToPC(trialCount,int(trialType),rewardCount);
        // start moving panel forward
        moverail(railMotorSpeed,LOW,1000); // move panel to present stimulus
        soundout(2); //
        TrialInit=1;
      } else {
        // listen for licks
        maxDiff = maxdiffanalog(); //peakAnalog();
        if (maxDiff>vibThd){
          if ((abs(currentMillis - trialMillis) <= gracePeriod)
            ||  (abs(currentMillis - trialMillis) > (gracePeriod + responseWindow))) {
          // Grace period:
          // 0.5–1.5 s from onset of panel movement where
          // licking does not signal the response outcome.
          sendTTL(lickTTLPin,1);
        }
        if ((abs(currentMillis - trialMillis) > gracePeriod)
          &&  (abs(currentMillis - trialMillis) <= (gracePeriod + responseWindow))) {
          sendTTL(lickTTLPin,0);
          // Response window: 1s.
          lickCount += 1;
         }
       }
       if ((abs(currentMillis - trialMillis) > (gracePeriod + responseWindow))) {
         // end of response window: check trial rules
         if (lickCount > 0) {// has licked
           if (trialType == 1) { // go trial
             rewardCount=rewardCount+1;
             reward(rewardSolenoid,40);
             trialOutcome = 1; // "hit"
           } else { // no-go trial
              trialOutcome = 3; // "false alarm!"" :p
           }
         } else { // no lick
           if (trialType ==1) { // go trial
             rewardCount=rewardCount+1;
             reward(rewardSolenoid,40);
             trialOutcome = 2; // "correct rejection" yeah !!
           } else { // no-go trial
             trialOutcome = 4; // "miss" ... pwon pwon pwon
           }
         }
        sendtoPC(trialCount,int(trialType),int(trialOutcome),rewardCount); // result of current trial
        trialwrapup(2000);
        sendTTL(trialTTLPin,0); // end of trial
        }
      }
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
   // adjusting initial AP location
   if (buttonFWpressed == true) {
    //  digitalWrite(directionPin_SM1, LOW);
     moverail(railMotorSpeed,LOW,100);
   } else if (buttonBWpressed == true) {
      // digitalWrite(directionPin_SM1, HIGH);
      moverail(-railMotorSpeed,HIGH,100);
    }
 }
 if (initialization == true) {
   // Define current panel angle as 0
   stimStepper.setCurrentPosition(0);
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

void moverail(float speed, uint8_t direction, int duration) {
 // railMotion = true;
 digitalWrite(directionPin_SM1, direction);
 digitalWrite(enablePin_SM1, HIGH);
 startLinearMoveTime=millis();
 while ((startLinearMoveTime+duration)>millis()){
   railStepper.setSpeed(speed);
   railStepper.runSpeed();
 }
 digitalWrite(enablePin_SM1, LOW);
 // railMotion = false;
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
    case 0:
    // that's it
    case 1:
    // 2 TTL pulse
    delay(25);
    digitalWrite(TTLpin, HIGH);
    delay(25);
    digitalWrite(TTLpin, LOW);
    break;
    case 2:
  // one more
    delay(25);
    digitalWrite(TTLpin, HIGH);
    delay(25);
    digitalWrite(TTLpin, LOW);
    delay(25);
    digitalWrite(TTLpin, HIGH);
    delay(25);
    digitalWrite(TTLpin, LOW);
    break;
  }
}

int maxdiffanalog(){
  unsigned int maxVal=0;
  unsigned int minVal=1024;

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

void soundout(int instruct){
  //HIGH triggers trinkets listening
  //Serial.println("TTL out");
  switch (instruct) {
    // both cases should total 10ms
    case 1:
    // white noise
      digitalWrite(soundTriggerPin, HIGH); // trigger
      // Nota Bene:
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

void trialwrapup(int timeout){ // needs to happen after sendToPC() command
  if (TrialSelectMode==2){ // block trials
    // time to move trial position one up (ie, setting up next trial type)
    BlockPos++;
    if (BlockPos>BlockSize){
      trialType++;
    }
    if (trialType>5){
      trialType=1;
    }
  }
  soundout(1); // white noise mask
  moverail(railMotorSpeed, HIGH, 1000); // HIGH = retract panel
  while (isRailRetracted()==false) {
    delay(1);
  }
  trialselection(); // set for next trial

  // refractory period
  delay(timeout); // timeout to leave time for 2s white noise + extra timeout if wrong trial
}

boolean isRailRetracted() {
  if (digitalRead(railPosIRpin) == HIGH) {
    return false;
  } else {
    return true;
  }
}

void trialselection(){
  // Rotate panel for next trial
  if (TrialSelectMode==0){ // random rotation
    nextTrialType = random(1,6);
  } else if (TrialSelectMode==1) {
     //instructions from computer
  } else if (TrialSelectMode==2) {
    // nextTrialType stays the same until it reaches end of block
  }

  //  Serial.print("panelrotate next pos ");
  //  Serial.println(nextTrialType);
  rot_seq = random(0,2); // random number between 0 and 1
  //  Serial.print("panelrotate rot seq ");
  //  Serial.println(rot_seq);

// move to new stim
 // each trial type corresponds to an angle
 // 1 /  2  / 3  /  4  /  5
 // 0 / 40 / 80 / 120 / 160 (for 5 stim panels and 200 steps/rev

    if (rot_seq == 0) {
      rotationAngle=stepsPerRev_SM2-(panelStepDifferential * (currentTrialType-nextTrialType));
      if (rotationAngle<stepsPerRev_SM2){
        rotationAngle=rotationAngle+stepsPerRev_SM2;
      }
      // move over one turn to target followed by one full turn
        rotatepanel(rotationAngle + stepsPerRev_SM2, 1, 50);
    } else { // same but counterclockwise
      rotationAngle=(panelStepDifferential * (nextTrialType-currentTrialType))-stepsPerRev_SM2;
      if (rotationAngle>-stepsPerRev_SM2){
        rotationAngle=rotationAngle-stepsPerRev_SM2;
      }
      rotatepanel(rotationAngle - stepsPerRev_SM2 , 1, 0);
    }

  //   digitalWrite(sleepPin_SM2, LOW);
  currentTrialType = nextTrialType;
  // Serial.print("currentTrialType is now ");
  // Serial.println(currentTrialType);
}

void rotatepanel(long relativePos, int rotDirection,  int pauseDuration){
  stimStepper.move(relativePos * rotDirection);
  stimStepper.runToPosition();
  delay(pauseDuration);
}
