// deliver reward when piezo is deflected

#include <HdFx_definitions.h>

void setup() {
  Serial.begin(115200);
  // Serial.println("Track 0");
  AFMS.begin();  // create with the default frequency 1.6KHz
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
  // Serial.println("Track 1");
  currentMillis = millis(); //keep track of time
  if(sessionStatus[1] == 1){ // reset counters
      //    arrayinit();
      trialCount=0;
      lickCount=0;
      rewardCount=0;
      // release step motors
    }
  //sessionStatus[0]=1;
  while (sessionStatus[0] == 0){ // session is OFF! Standby
    // Serial.println("Standby");
   if (readbuttons()) {
     actonbuttons();
   } else {
     //  Serial.println("idle");
     digitalWrite(enablePin_SM1, LOW);
     getdatafromPC();
   }
  }

  // rewardSolenoid->run(RELEASE);    // make sure to close solenoids
  // puffSolenoid->run(RELEASE);

  if (sessionStatus[0] == 1){ // session is ON!
    if (trialInit==0){
      Serial.println("initialize on first loop");
      trialMillis = millis();
      sendTTL(trialTTLPin,1); // send TTL : begin trial
      trialCount=trialCount+1; //increment trial number
      // sendToPC(trialCount,int(trialType),rewardCount);
      trialInit=1;
    } else {
      // Serial.println("waiting for licks");
      if (touchPiezo()){
           rewardCount=rewardCount+1;
           reward(rewardSolenoid,rewardDuration); // 40 calibrated to 1ul
           sendtoPC(trialCount,int(trialType),int(trialOutcome),rewardCount); // result of current trial
           trialwrapup(interTrialInterval/4);
           sendTTL(trialTTLPin,0); // end of trial
      }
    }
  }
}

bool readbuttons() {
  // Serial.println("Track 2");
 // buttonBWpressed = false;
 // buttonFWpressed = false;
 if ((digitalRead(joyBehaviorPin) == HIGH) || (digitalRead(redButton) == HIGH)) {
   if (digitalRead(joyBehaviorPin) == HIGH) {
    //  Serial.println("joyBehavior push button ON"); // Green button
     railControl = true;
     initialization = false;
     if  (digitalRead(redButton) == HIGH) { // both buttons pressed
       Serial.println("Manual override. Session is ON!"); // Red button also pressed
       sessionStatus[0]=1;
     }
   }
   if  (digitalRead(redButton) == HIGH) {
     railControl = false;
     initialization = true;
   }
   return (true);
   } else {
     if  (digitalRead(redButton) == LOW) {
       flushtubes = false;
     }
     return (false);
  }
}

void actonbuttons() {
  // Serial.println("Track 3");
 if (railControl == true) {
   digitalWrite(enablePin_SM1, HIGH);
   // adjusting initial AP location
   if (digitalRead(buttonFWpin) == HIGH) { //(buttonFWpressed == true) {
    //  digitalWrite(directionPin_SM1, LOW);
     moverail(-railMotorSpeed,LOW,100);
   } else if (digitalRead(buttonBWpin) == HIGH) { //(buttonBWpressed == true) {
      // digitalWrite(directionPin_SM1, HIGH);
      moverail(railMotorSpeed,HIGH,100);
    }
    digitalWrite(enablePin_SM1, LOW);
 }
 if (initialization == true) {
   // Define current panel angle as 0
   stimStepper.setCurrentPosition(0);
   // set up first trial
    if (digitalRead(buttonFWpin) == HIGH) { //(buttonFWpressed == true) {
      trialwrapup(2000);
    } else if (digitalRead(buttonBWpin) == HIGH) {
   // or flush reward
   flushtubes = true;
       while (flushtubes == true){
         Serial.println("flush tubes");
         readbuttons();
         flushtubing();
       }
     }
   }
}

void moverail(int speed, unsigned char direction, int duration) {
  if (isRailRetracted()==true) {
    // stop and move forward a bit
    digitalWrite(directionPin_SM1, LOW);
    digitalWrite(enablePin_SM1, HIGH);
    while (isRailRetracted()==true){
      railStepper.setSpeed(-railMotorSpeed);
      railStepper.runSpeed();
    }
  } else {
   // railMotion = true;
   digitalWrite(directionPin_SM1, direction);
   digitalWrite(enablePin_SM1, HIGH);
   startLinearMoveTime=millis();
   while ((startLinearMoveTime+duration)>millis()){
     railStepper.setSpeed(speed);
     railStepper.runSpeed();
   }
 }
 // digitalWrite(enablePin_SM1, LOW);
 // railMotion = false;
}

bool trialready() {
  if (trialInit==0) { // only run checks if trial is not ongoing
  // check if panel is retracted
      // railMovedBack = isRailRetracted();
    // return (railMovedBack && stimShuffle);
    return (stimShuffle);
  } else {
    return (true);
  }
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

bool touchPiezo(){
  lowpassFilter.input( analogRead( piezoPin ) );
  centeredVal=abs(lowpassFilter.output()-sampleTouch.mean());
  sampleTouch.push(centeredVal);
  // Serial.print(centeredVal);
  SNR = max((sampleTouch.mean()/(max(sampleTouch.stddev(),0.01))),1);
  // Serial.print("\t");
  // Serial.println(5*SNR);
  cumulativeTouch<<=1;
  if (centeredVal>(SNRthreshold*SNR)){
    // detect.push(1);
    cumulativeTouch = cumulativeTouch | 1;
    //    Serial.println("above SNR");
  } else {
    // detect.push(0);
    cumulativeTouch = cumulativeTouch | 0;
  }
  //  Serial.println(cumulativeTouch & mask);
  if ((cumulativeTouch & mask) == mask && ((millis()-touchTime)>lickInterval)){//((detect.mean()*4)>=2) {
    // Serial.print("Touch detected with mean value ");
    // Serial.println(sampleTouch.mean());
    Serial.print(centeredVal);
    Serial.print("\t");
    Serial.println(5*SNR);
    // sampleTouch.clear();
    touchTime = millis(); //keep track of time
    return(true);
  } else {
    return(false);
  }
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

void trialwrapup(unsigned int timeout){ // needs to happen after sendToPC() command
  trialInit=0;
  // refractory period
  countDown = millis();
  while (millis()-countDown<timeout){// timeout to leave time for 2s white noise + extra timeout if wrong trial
    touchPiezo();
  };
}

bool isRailRetracted() {
  if (digitalRead(railPosIRpin) == HIGH) {
    // Serial.println("rail retracted");
    return true;
  } else {
    return false;
  }
}

void trialselection(){
  // Rotate panel for next trial
  if (trialselectMode==0){ // random rotation
    nextTrialType = random(1,6);
  } else if (trialselectMode==1) {
     //instructions from computer
  } else if (trialselectMode==2) {
    // nextTrialType stays the same until it reaches end of block
  }

 // move to new stim
 // each trial type corresponds to an angle
 // 1 /  2  / 3  /  4  /  5
 // 0 / 40 / 80 / 120 / 160 (for 5 stim panels and 200 steps/rev)
  rot_seq = random(0,2); // random number between 0 and 1
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
  Serial.print("currentTrialType is now ");
  Serial.println(currentTrialType);
  stimShuffle=true;
}

void rotatepanel(long relativePos, int rotDirection,  int pauseDuration){
  stimStepper.move(relativePos * rotDirection);
  stimStepper.runToPosition();
  delay(pauseDuration);
}

void getdatafromPC() {
  // receive data from PC and save it into inputBuffer
  if(Serial.available() > 0) {
    char x = Serial.read();
      // the order of these IF clauses is significant

    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parsedata();
    }

    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) {
      bytesRecvd = 0;
      readInProgress = true;
    }
  }
}

void parsedata() {
    // split the data into its parts
    // assumes the data will be received as (eg) 1,0,2
  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(inputBuffer,","); // get the first part (session ON / OFF)
  sessionStatus[0] = atoi(strtokIndx); //  convert to an integer

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  sessionStatus[1] = atoi(strtokIndx); // sessionStatus[1] is to reset counters

  strtokIndx = strtok(NULL, ",");
  //    if (strtokIndx=="True"){
  //    TrialType = 1;
  //  } else if (strtokIndx=="False"){
  //    TrialType = 2;
  // }
  trialType = atoi(strtokIndx)+1;

  if (trialselectMode==3){ // ignore trial type given by PC
    if (trialCount==0){ //just starting -> set trial type accordingly
      trialType = 1;
    }
  }
}

void flushtubing(){
  // Serial.println("flush");
  // flush left
  rewardSolenoid->run(FORWARD);
  rewardSolenoid->setSpeed(255);
  delay(1500);
  rewardSolenoid->run(RELEASE);
  // flush right
  // RightSolenoid->run(FORWARD);
  // RightSolenoid->setSpeed(255);
  // delay(1500);
  // RightSolenoid->run(RELEASE);
}
