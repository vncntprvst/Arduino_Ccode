//define functions;
#include <HdFx_dependencies.h>

bool readbuttons();
void actonbuttons();
void moverail(int speed, unsigned char direction, int duration);
void getdatafromPC();
void parsedata();
bool trialready();
void trialwrapup(unsigned int timeout);
void flushtubing();
void sendTTL(int TTLpin, int instruct);
bool touchPiezo();
void reward(Adafruit_DCMotor* solenoid,int dur);
void sendtoPC(int trialNum, int tType, int trialResult, int rewCount);
void soundout(int instruct);
bool isRailRetracted();
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

// Stepper 2: stim (controlled through Polulu A4988)
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
#define joyBehaviorPin 19
// bool buttonCWpressed = false;
// bool buttonCCWpressed = false;

// lick sensor variables
#define piezoPin A0
#define lickTTLPin 21
float filterFrequency = 10.0; // filters out changes faster that 10 Hz.
FilterOnePole lowpassFilter( LOWPASS, filterFrequency );  // create a one pole (RC) lowpass filter
Average<long> sampleTouch(10);
byte cumulativeTouch = 0 ;
byte mask  = 7; // 7 => 111 (3 successive detection); 15 => 1111
long centeredVal;
long SNR;
unsigned long touchTime = 0;
#define lickInterval 100 // adjust to constraint max allowed lick frequency
#define rewardDuration 80 // 40 calibrated to 1ul
#define SNRthreshold 4

// byte baseline = 1;
unsigned int piezoVal; // max 1024
unsigned int maxDiff;
unsigned int lickCount = 0;

// session and trial variables
#define redButton 17 // for manual override
#define trialTTLPin 7
#define interTrialInterval 2000
unsigned long trialMillis = 0;
byte trialselectMode=0; // 0 -> randomized trials
                        // 1 -> trial-by-tria instrucitons from computer
                        // 2 -> block trial preset (see trialselection).
byte blockSize=15; // this is the number of trials in each block
byte blockPos=1; // where we are in the block of trials
byte trialType = 0;
byte sessionStatus[2] = {0, 0}; // 1/ Run [ON/OFF (1/0)] 2/ Reset (1)
byte trialOutcome=0;
unsigned int trialCount=0;
unsigned int trialInit=0;
unsigned int gracePeriod=1000;
unsigned int responseWindow=1000;
unsigned int rewardCount=0;
bool railMovedBack=false;
bool stimShuffle=false;

// texture panel definitions
#define railPosIRpin 15
int currentTrialType=0; // inital panel position
int nextTrialType;
byte rot_seq; // number of panel rotations
int rotationAngle;

// audio definitions
#define soundTriggerPin 6 // audio output

// GUI-related variables
const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
bool readInProgress = false;
bool newDataFromPC = false;

// other defs
// #define ledPin 13
unsigned long currentMillis = 0;
unsigned long countDown = 0;
bool buttonFWpressed = false;
bool buttonBWpressed = false;
bool railControl = false;
bool initialization = false;
bool flushtubes = false;
bool stimRotation = false;
bool railMotion = false;
