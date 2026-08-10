#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>

// Initialize both PCA9685 boards
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // Card 1: No pads soldered
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x60); // Card 2: A5 pad soldered
BluetoothSerial SerialBT;

// Servo Calibration Constants (Adjust these if your servos strain at the limits)
#define SERVOMIN   150 
#define SERVOMAX   600 
#define SERVO_FREQ 50 
#define LEG_DETECT_THRESHOLD_MV 2500
#define HOME_HIP 90
#define HOME_FEMUR 153
#define HOME_TIBIA 160
#define LIFT_FEMUR 160
#define LIFT_TIBIA 130
#define MOVE_SPEED_DELAY 15
#define WALK_HIP_SWING 18
#define WALK_FEMUR_LIFT 165
#define WALK_TIBIA_LIFT 125
#define WALK_STEP_DELAY 12
#define WALK_SETTLE_DELAY 120
#define LEG3_PUSH_FEMUR 130
#define LEG3_PUSH_TIBIA 130
#define ROTATE_HIP_SWING 20

const char* BLUETOOTH_NAME = "Hexapod-Control";

// Removable leg detect pins for the 30-pin ESP32.
// A leg is available when its detect pin is at or above LEG_DETECT_THRESHOLD_MV.
const int legDetectPins[7] = {
  -1, // Unused (Index 0)
  34, // Leg 1
  35, // Leg 2
  32, // Leg 3
  33, // Leg 4
  25, // Leg 5
  26  // Leg 6
};

bool legAvailable[7] = {
  false, false, false, false, false, false, false
};

// Global array to keep track of the CURRENT angle of every single joint
// Array dimensions: [leg 1-6][joint 1-3] -> initialized to home positions
int currentAngles[7][4] = {
  {0, 0, 0, 0},         // Unused (Index 0)
  {0, HOME_HIP, HOME_FEMUR, HOME_TIBIA}, // Leg 1 [Hip, Femur, Tibia]
  {0, HOME_HIP, HOME_FEMUR, HOME_TIBIA}, // Leg 2
  {0, HOME_HIP, HOME_FEMUR, HOME_TIBIA}, // Leg 3
  {0, HOME_HIP, HOME_FEMUR, HOME_TIBIA}, // Leg 4
  {0, HOME_HIP, HOME_FEMUR, HOME_TIBIA}, // Leg 5
  {0, HOME_HIP, HOME_FEMUR, HOME_TIBIA}  // Leg 6
};

// --- Function Prototypes ---
int angleToPulse(int angle);
void setLegJoint(int leg, int joint, int angle);
void slowMove(int leg, int joint, int targetAngle, int speedDelay);
void setupLegDetectPins();
void updateLegAvailability();
void printLegAvailability();
void printSingleLegAvailability(int leg);
bool isLegAvailable(int leg);
void liftLeg(int leg);
int availableLegCount();
bool areLegsAvailable(const int legs[], int legCount);
bool isOnlyLegAvailable(int leg);
bool areAllLegsAvailable();
int collectAvailableLegs(int legs[]);
bool hasExactLegSet(const int availableLegs[], int availableCount, const int targetLegs[], int targetCount);
bool findThreeLegRecoveryGait(const int availableLegs[], int availableCount, int& liftLegNumber, int& supportLegA, int& supportLegB);
int forwardHipAngle(int leg);
int backwardHipAngle(int leg);
int rotateHipAngle(int direction);
void moveJointGroup(const int legs[], int legCount, int joint, const int targetAngles[], int speedDelay);
void moveLegToGround(int leg);
void liftWalkingLeg(int leg);
void swingWalkingGroupForward(const int legs[], int legCount);
void swingWalkingGroupForwardBySide(const int legs[], int legCount);
void lowerWalkingGroup(const int legs[], int legCount);
void pushWalkingGroupBackward(const int legs[], int legCount);
void pushWalkingGroupBackwardBySide(const int legs[], int legCount);
void prepareWalkingLegs(const int legs[], int legCount);
void walkSingleGroupCycle(const int pushLegs[], int pushLegCount);
void walkAlternatingGroupsCycle(const int groupA[], int groupACount, const int groupB[], int groupBCount);
void walkAlternatingGroupsCycleBySide(const int groupA[], int groupACount, const int groupB[], int groupBCount);
void walkForwardWithSingleGroup(const int pushLegs[], int pushLegCount, int cycles);
void walkForwardWithAlternatingGroups(const int groupA[], int groupACount, const int groupB[], int groupBCount, int cycles);
void walkForwardWithAlternatingGroupsBySide(const int groupA[], int groupACount, const int groupB[], int groupBCount, int cycles);
void walkTripodCycle();
void walkForwardTripod(int cycles);
void walkForwardFourLeg(int cycles);
void walkForwardFiveLegRecovery(int missingLeg, int cycles);
void walkForwardPair(int legA, int legB, int cycles);
void walkSingleLegOnlyCycle(int leg);
void walkForwardSingleLeg(int leg, int cycles);
void walkThreeLegRecoveryCycle(int middleLeg, int supportLegA, int supportLegB);
void walkForwardThreeLegRecovery(int liftLegNumber, int supportLegA, int supportLegB, int cycles);
void walkForward(int cycles);
void liftLegGroup(const int legs[], int legCount);
void rotateRobotCycle(int direction);
void rotateRobot(int direction, int cycles);
void homeRobot();
void setupBluetooth();
void printSerialHelpTo(Print& output);
void printSerialHelp();
void handleCommandInput(String input, Print& output);
void handleSerialCommands();
void handleUsbSerialCommands();
void handleBluetoothCommands();

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  while(!Serial) { delay(10); } 
  setupBluetooth();

  setupLegDetectPins();
  
  // Explicitly assign I2C pins for the 38-pin ESP32
  // GPIO 21 = SDA, GPIO 22 = SCL
  Wire.begin(21, 22); 

  Serial.println("Initializing PCA9685 Drivers...");
  pwm1.begin();
  pwm2.begin();
  
  pwm1.setPWMFreq(SERVO_FREQ);
  pwm2.setPWMFreq(SERVO_FREQ);

  delay(500); // Allow hardware voltages to stabilize

  updateLegAvailability();
  printLegAvailability();
  
  Serial.println("Moving Hexapod to Home Position safely...");
  homeRobot();
  printSerialHelp();
  delay(1000); 
}

void loop() {
  handleSerialCommands();
}

/**
 * Converts an angle (0-180) to the PCA9685 PWM pulse duration (SERVOMIN-SERVOMAX)
 */
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

/**
 * Directly writes an angle to a specific joint and updates the tracking array
 * @param leg - Leg number (1 to 6)
 * @param joint - Joint number (1: Hip, 2: Femur, 3: Tibia)
 * @param angle - Target angle (0 to 180)
 */
void setLegJoint(int leg, int joint, int angle) {
  // Guard rails to prevent mechanical damage over-travel
  angle = constrain(angle, 0, 180); 
  
  int pulse = angleToPulse(angle);
  int baseChannel = 0;
  
  // Determine pin channel mapping on the board (4 channels per leg)
  if (leg == 1 || leg == 4)      baseChannel = 0;
  else if (leg == 2 || leg == 5) baseChannel = 4;
  else if (leg == 3 || leg == 6) baseChannel = 8;
  
  int targetChannel = baseChannel + (joint - 1);

  // Route command to the correct physical PCA9685 card
  if (leg <= 3) {
    pwm1.setPWM(targetChannel, 0, pulse);
  } else {
    pwm2.setPWM(targetChannel, 0, pulse);
  }
  
  // Critical: Keep our software variable synced with physical state
  currentAngles[leg][joint] = angle; 
}

/**
 * Moves a single joint smoothly from its current position to a target position
 * @param speedDelay - Delay in milliseconds between each 1-degree step (higher = slower)
 */
void slowMove(int leg, int joint, int targetAngle, int speedDelay) {
  if (!isLegAvailable(leg)) {
    Serial.print("Skipping move. Leg ");
    Serial.print(leg);
    Serial.println(" is not connected.");
    return;
  }

  int startAngle = currentAngles[leg][joint];
  
  if (startAngle < targetAngle) {
    for (int a = startAngle; a <= targetAngle; a++) {
      setLegJoint(leg, joint, a);
      delay(speedDelay);
    }
  } else {
    for (int a = startAngle; a >= targetAngle; a--) {
      setLegJoint(leg, joint, a);
      delay(speedDelay);
    }
  }
}

void setupLegDetectPins() {
  analogReadResolution(12);

  for (int leg = 1; leg <= 6; leg++) {
    pinMode(legDetectPins[leg], INPUT);
    analogSetPinAttenuation(legDetectPins[leg], ADC_11db);
  }
}

void updateLegAvailability() {
  for (int leg = 1; leg <= 6; leg++) {
    int detectVoltage = analogReadMilliVolts(legDetectPins[leg]);
    legAvailable[leg] = (detectVoltage >= LEG_DETECT_THRESHOLD_MV);
  }
}

void printLegAvailability() {
  Serial.println("Checking removable leg connections...");

  for (int leg = 1; leg <= 6; leg++) {
    printSingleLegAvailability(leg);
  }
}

void printSingleLegAvailability(int leg) {
  if (leg < 1 || leg > 6) {
    Serial.println("Enter a leg number from 1 to 6.");
    return;
  }

  Serial.print("Leg ");
  Serial.print(leg);
  Serial.print(" detect pin D");
  Serial.print(legDetectPins[leg]);
  Serial.print(": ");
  Serial.print(analogReadMilliVolts(legDetectPins[leg]));
  Serial.print(" mV -> ");
  Serial.println(legAvailable[leg] ? "AVAILABLE" : "NOT CONNECTED");
}

bool isLegAvailable(int leg) {
  if (leg < 1 || leg > 6) {
    return false;
  }

  return legAvailable[leg];
}

void liftLeg(int leg) {
  if (leg < 1 || leg > 6) {
    Serial.println("Enter a leg number from 1 to 6.");
    return;
  }

  updateLegAvailability();
  printSingleLegAvailability(leg);

  if (!isLegAvailable(leg)) {
    Serial.print("Cannot lift Leg ");
    Serial.print(leg);
    Serial.println(" because it is not connected.");
    return;
  }

  Serial.print("Lifting Leg ");
  Serial.println(leg);
  slowMove(leg, 2, LIFT_FEMUR, MOVE_SPEED_DELAY);
  slowMove(leg, 3, LIFT_TIBIA, MOVE_SPEED_DELAY);
  Serial.println("Lift complete.");
}

int forwardHipAngle(int leg) {
  if (leg <= 3) {
    return HOME_HIP + WALK_HIP_SWING;
  }

  return HOME_HIP - WALK_HIP_SWING;
}

int backwardHipAngle(int leg) {
  if (leg <= 3) {
    return HOME_HIP - WALK_HIP_SWING;
  }

  return HOME_HIP + WALK_HIP_SWING;
}

void moveJointGroup(const int legs[], int legCount, int joint, const int targetAngles[], int speedDelay) {
  bool moving = true;

  while (moving) {
    moving = false;

    for (int i = 0; i < legCount; i++) {
      int leg = legs[i];
      int targetAngle = constrain(targetAngles[i], 0, 180);
      int currentAngle = currentAngles[leg][joint];

      if (currentAngle < targetAngle) {
        setLegJoint(leg, joint, currentAngle + 1);
        moving = true;
      } else if (currentAngle > targetAngle) {
        setLegJoint(leg, joint, currentAngle - 1);
        moving = true;
      }
    }

    if (moving) {
      delay(speedDelay);
    }
  }
}

void moveLegToGround(int leg) {
  slowMove(leg, 2, HOME_FEMUR, WALK_STEP_DELAY);
  slowMove(leg, 3, HOME_TIBIA, WALK_STEP_DELAY);
}

void liftWalkingLeg(int leg) {
  slowMove(leg, 2, WALK_FEMUR_LIFT, WALK_STEP_DELAY);
  slowMove(leg, 3, WALK_TIBIA_LIFT, WALK_STEP_DELAY);
}

int availableLegCount() {
  int count = 0;

  for (int leg = 1; leg <= 6; leg++) {
    if (isLegAvailable(leg)) {
      count++;
    }
  }

  return count;
}

bool areLegsAvailable(const int legs[], int legCount) {
  for (int i = 0; i < legCount; i++) {
    if (!isLegAvailable(legs[i])) {
      return false;
    }
  }

  return true;
}

bool isOnlyLegAvailable(int leg) {
  return isLegAvailable(leg) && availableLegCount() == 1;
}

bool areAllLegsAvailable() {
  return availableLegCount() == 6;
}

int collectAvailableLegs(int legs[]) {
  int legCount = 0;

  for (int leg = 1; leg <= 6; leg++) {
    if (isLegAvailable(leg)) {
      legs[legCount] = leg;
      legCount++;
    }
  }

  return legCount;
}

bool hasExactLegSet(const int availableLegs[], int availableCount, const int targetLegs[], int targetCount) {
  if (availableCount != targetCount) {
    return false;
  }

  for (int i = 0; i < targetCount; i++) {
    bool found = false;

    for (int j = 0; j < availableCount; j++) {
      if (targetLegs[i] == availableLegs[j]) {
        found = true;
        break;
      }
    }

    if (!found) {
      return false;
    }
  }

  return true;
}

bool findThreeLegRecoveryGait(const int availableLegs[], int availableCount, int& liftLegNumber, int& supportLegA, int& supportLegB) {
  const int triplet123[] = {1, 2, 3};
  const int triplet234[] = {2, 3, 4};
  const int triplet345[] = {3, 4, 5};
  const int triplet456[] = {4, 5, 6};
  const int triplet561[] = {5, 6, 1};
  const int triplet612[] = {6, 1, 2};

  if (hasExactLegSet(availableLegs, availableCount, triplet123, 3)) {
    liftLegNumber = 2;
    supportLegA = 1;
    supportLegB = 3;
    return true;
  }

  if (hasExactLegSet(availableLegs, availableCount, triplet234, 3)) {
    liftLegNumber = 3;
    supportLegA = 2;
    supportLegB = 4;
    return true;
  }

  if (hasExactLegSet(availableLegs, availableCount, triplet345, 3)) {
    liftLegNumber = 4;
    supportLegA = 3;
    supportLegB = 5;
    return true;
  }

  if (hasExactLegSet(availableLegs, availableCount, triplet456, 3)) {
    liftLegNumber = 5;
    supportLegA = 4;
    supportLegB = 6;
    return true;
  }

  if (hasExactLegSet(availableLegs, availableCount, triplet561, 3)) {
    liftLegNumber = 6;
    supportLegA = 5;
    supportLegB = 1;
    return true;
  }

  if (hasExactLegSet(availableLegs, availableCount, triplet612, 3)) {
    liftLegNumber = 1;
    supportLegA = 6;
    supportLegB = 2;
    return true;
  }

  return false;
}

int rotateHipAngle(int direction) {
  return HOME_HIP + (ROTATE_HIP_SWING * direction);
}

void swingWalkingGroupForward(const int legs[], int legCount) {
  int targetAngles[6];
  int liftFemurAngles[6];
  int liftTibiaAngles[6];

  for (int i = 0; i < legCount; i++) {
    liftFemurAngles[i] = WALK_FEMUR_LIFT;
    liftTibiaAngles[i] = WALK_TIBIA_LIFT;

    if (legCount == 2) {
      targetAngles[i] = HOME_HIP + ((i == 0) ? WALK_HIP_SWING : -WALK_HIP_SWING);
    } else {
      targetAngles[i] = forwardHipAngle(legs[i]);
    }
  }

  moveJointGroup(legs, legCount, 2, liftFemurAngles, WALK_STEP_DELAY);
  moveJointGroup(legs, legCount, 3, liftTibiaAngles, WALK_STEP_DELAY);
  moveJointGroup(legs, legCount, 1, targetAngles, WALK_STEP_DELAY);
}

void swingWalkingGroupForwardBySide(const int legs[], int legCount) {
  int targetAngles[6];
  int liftFemurAngles[6];
  int liftTibiaAngles[6];

  for (int i = 0; i < legCount; i++) {
    liftFemurAngles[i] = WALK_FEMUR_LIFT;
    liftTibiaAngles[i] = WALK_TIBIA_LIFT;
    targetAngles[i] = forwardHipAngle(legs[i]);
  }

  moveJointGroup(legs, legCount, 2, liftFemurAngles, WALK_STEP_DELAY);
  moveJointGroup(legs, legCount, 3, liftTibiaAngles, WALK_STEP_DELAY);
  moveJointGroup(legs, legCount, 1, targetAngles, WALK_STEP_DELAY);
}

void lowerWalkingGroup(const int legs[], int legCount) {
  int groundFemurAngles[6];
  int groundTibiaAngles[6];

  for (int i = 0; i < legCount; i++) {
    groundFemurAngles[i] = HOME_FEMUR;
    groundTibiaAngles[i] = HOME_TIBIA;
  }

  moveJointGroup(legs, legCount, 2, groundFemurAngles, WALK_STEP_DELAY);
  moveJointGroup(legs, legCount, 3, groundTibiaAngles, WALK_STEP_DELAY);
}

void pushWalkingGroupBackward(const int legs[], int legCount) {
  int targetAngles[6];

  for (int i = 0; i < legCount; i++) {
    if (legCount == 2) {
      targetAngles[i] = HOME_HIP + ((i == 0) ? -WALK_HIP_SWING : WALK_HIP_SWING);
    } else {
      targetAngles[i] = backwardHipAngle(legs[i]);
    }
  }

  moveJointGroup(legs, legCount, 1, targetAngles, WALK_STEP_DELAY);
}

void pushWalkingGroupBackwardBySide(const int legs[], int legCount) {
  int targetAngles[6];

  for (int i = 0; i < legCount; i++) {
    targetAngles[i] = backwardHipAngle(legs[i]);
  }

  moveJointGroup(legs, legCount, 1, targetAngles, WALK_STEP_DELAY);
}

void prepareWalkingLegs(const int legs[], int legCount) {
  int homeHipAngles[6];
  int liftFemurAngles[6];
  int liftTibiaAngles[6];

  for (int i = 0; i < legCount; i++) {
    homeHipAngles[i] = HOME_HIP;
    liftFemurAngles[i] = WALK_FEMUR_LIFT;
    liftTibiaAngles[i] = WALK_TIBIA_LIFT;
  }

  moveJointGroup(legs, legCount, 2, liftFemurAngles, WALK_STEP_DELAY);
  moveJointGroup(legs, legCount, 3, liftTibiaAngles, WALK_STEP_DELAY);
  moveJointGroup(legs, legCount, 1, homeHipAngles, WALK_STEP_DELAY);
  lowerWalkingGroup(legs, legCount);
}

void walkSingleGroupCycle(const int pushLegs[], int pushLegCount) {
  pushWalkingGroupBackward(pushLegs, pushLegCount);
  swingWalkingGroupForward(pushLegs, pushLegCount);
  lowerWalkingGroup(pushLegs, pushLegCount);
  delay(WALK_SETTLE_DELAY);
}

void walkAlternatingGroupsCycle(const int groupA[], int groupACount, const int groupB[], int groupBCount) {
  swingWalkingGroupForward(groupA, groupACount);
  pushWalkingGroupBackward(groupB, groupBCount);
  lowerWalkingGroup(groupA, groupACount);
  delay(WALK_SETTLE_DELAY);

  swingWalkingGroupForward(groupB, groupBCount);
  pushWalkingGroupBackward(groupA, groupACount);
  lowerWalkingGroup(groupB, groupBCount);
  delay(WALK_SETTLE_DELAY);
}

void walkAlternatingGroupsCycleBySide(const int groupA[], int groupACount, const int groupB[], int groupBCount) {
  swingWalkingGroupForwardBySide(groupA, groupACount);
  pushWalkingGroupBackwardBySide(groupB, groupBCount);
  lowerWalkingGroup(groupA, groupACount);
  delay(WALK_SETTLE_DELAY);

  swingWalkingGroupForwardBySide(groupB, groupBCount);
  pushWalkingGroupBackwardBySide(groupA, groupACount);
  lowerWalkingGroup(groupB, groupBCount);
  delay(WALK_SETTLE_DELAY);
}

void walkForwardWithSingleGroup(const int pushLegs[], int pushLegCount, int cycles) {
  prepareWalkingLegs(pushLegs, pushLegCount);

  for (int cycle = 0; cycle < cycles; cycle++) {
    walkSingleGroupCycle(pushLegs, pushLegCount);
  }
}

void walkForwardWithAlternatingGroups(const int groupA[], int groupACount, const int groupB[], int groupBCount, int cycles) {
  prepareWalkingLegs(groupA, groupACount);
  prepareWalkingLegs(groupB, groupBCount);

  for (int cycle = 0; cycle < cycles; cycle++) {
    walkAlternatingGroupsCycle(groupA, groupACount, groupB, groupBCount);
  }
}

void walkForwardWithAlternatingGroupsBySide(const int groupA[], int groupACount, const int groupB[], int groupBCount, int cycles) {
  prepareWalkingLegs(groupA, groupACount);
  prepareWalkingLegs(groupB, groupBCount);

  for (int cycle = 0; cycle < cycles; cycle++) {
    walkAlternatingGroupsCycleBySide(groupA, groupACount, groupB, groupBCount);
  }
}

void walkTripodCycle() {
  const int tripodA[] = {1, 3, 5};
  const int tripodB[] = {2, 4, 6};
  const int pushA[] = {3, 5};
  const int pushB[] = {2, 4};

  swingWalkingGroupForwardBySide(tripodA, 3);
  liftWalkingLeg(6);
  pushWalkingGroupBackwardBySide(pushB, 2);
  moveLegToGround(6);
  lowerWalkingGroup(tripodA, 3);
  delay(WALK_SETTLE_DELAY);

  swingWalkingGroupForwardBySide(tripodB, 3);
  liftWalkingLeg(1);
  pushWalkingGroupBackwardBySide(pushA, 2);
  moveLegToGround(1);
  lowerWalkingGroup(tripodB, 3);
  delay(WALK_SETTLE_DELAY);
}

void walkForwardTripod(int cycles) {
  const int tripodA[] = {1, 3, 5};
  const int tripodB[] = {2, 4, 6};

  Serial.println("All 6 legs available. Using tripod gait with drag-leg lift: push 3+5 while lifting 1, push 2+4 while lifting 6.");
  prepareWalkingLegs(tripodA, 3);
  prepareWalkingLegs(tripodB, 3);

  for (int cycle = 0; cycle < cycles; cycle++) {
    walkTripodCycle();
  }
}

void walkForwardFourLeg(int cycles) {
  const int pairA[] = {1, 5};
  const int pairB[] = {2, 4};

  if (isLegAvailable(3)) {
    Serial.println("Lifting and holding Leg 3. Walking with legs 1+5 and 2+4.");
    slowMove(3, 2, LIFT_FEMUR, MOVE_SPEED_DELAY);
    slowMove(3, 3, LIFT_TIBIA, MOVE_SPEED_DELAY);
  } else {
    Serial.println("Walking with legs 1+5 and 2+4.");
  }

  walkForwardWithAlternatingGroups(pairA, 2, pairB, 2, cycles);
}

void walkForwardFiveLegRecovery(int missingLeg, int cycles) {
  int groupA[2] = {0, 0};
  int groupB[2] = {0, 0};
  int liftLegNumber = 0;

  switch (missingLeg) {
    case 1:
      groupA[0] = 3;
      groupA[1] = 5;
      groupB[0] = 2;
      groupB[1] = 6;
      liftLegNumber = 4;
      break;
    case 2:
      groupA[0] = 1;
      groupA[1] = 3;
      groupB[0] = 4;
      groupB[1] = 6;
      liftLegNumber = 5;
      break;
    case 3:
      groupA[0] = 1;
      groupA[1] = 5;
      groupB[0] = 2;
      groupB[1] = 4;
      liftLegNumber = 6;
      break;
    case 4:
      groupA[0] = 3;
      groupA[1] = 5;
      groupB[0] = 2;
      groupB[1] = 6;
      liftLegNumber = 1;
      break;
    case 5:
      groupA[0] = 1;
      groupA[1] = 3;
      groupB[0] = 4;
      groupB[1] = 6;
      liftLegNumber = 2;
      break;
    case 6:
      groupA[0] = 1;
      groupA[1] = 5;
      groupB[0] = 2;
      groupB[1] = 4;
      liftLegNumber = 3;
      break;
    default:
      Serial.println("Five-leg recovery cancelled. Missing leg number is invalid.");
      return;
  }

  Serial.print("Five-leg recovery. Missing Leg ");
  Serial.print(missingLeg);
  Serial.print(", lifting Leg ");
  Serial.print(liftLegNumber);
  Serial.print(", walking with ");
  Serial.print(groupA[0]);
  Serial.print("+");
  Serial.print(groupA[1]);
  Serial.print(" and ");
  Serial.print(groupB[0]);
  Serial.print("+");
  Serial.print(groupB[1]);
  Serial.println(".");

  slowMove(liftLegNumber, 2, LIFT_FEMUR, MOVE_SPEED_DELAY);
  slowMove(liftLegNumber, 3, LIFT_TIBIA, MOVE_SPEED_DELAY);
  walkForwardWithAlternatingGroupsBySide(groupA, 2, groupB, 2, cycles);
}

void walkForwardPair(int legA, int legB, int cycles) {
  const int pair[] = {legA, legB};

  Serial.print("Walking forward with pair ");
  Serial.print(legA);
  Serial.print("+");
  Serial.print(legB);
  Serial.println(".");

  walkForwardWithSingleGroup(pair, 2, cycles);
}

void walkSingleLegOnlyCycle(int leg) {
  slowMove(leg, 2, LEG3_PUSH_FEMUR, WALK_STEP_DELAY);
  slowMove(leg, 3, LEG3_PUSH_TIBIA, WALK_STEP_DELAY);
  delay(WALK_SETTLE_DELAY);

  slowMove(leg, 2, HOME_FEMUR, WALK_STEP_DELAY);
  slowMove(leg, 3, HOME_TIBIA, WALK_STEP_DELAY);
  delay(WALK_SETTLE_DELAY);
}

void walkForwardSingleLeg(int leg, int cycles) {
  Serial.print("Only Leg ");
  Serial.print(leg);
  Serial.println(" is available. Using femur/tibia push only. Hip will not move.");

  for (int cycle = 0; cycle < cycles; cycle++) {
    walkSingleLegOnlyCycle(leg);
  }
}

void walkThreeLegRecoveryCycle(int middleLeg, int supportLegA, int supportLegB) {
  const int supportPair[] = {supportLegA, supportLegB};

  liftWalkingLeg(middleLeg);
  pushWalkingGroupBackward(supportPair, 2);
  moveLegToGround(middleLeg);
  delay(WALK_SETTLE_DELAY);

  swingWalkingGroupForward(supportPair, 2);
  lowerWalkingGroup(supportPair, 2);
  delay(WALK_SETTLE_DELAY);
}

void walkForwardThreeLegRecovery(int liftLegNumber, int supportLegA, int supportLegB, int cycles) {
  const int supportPair[] = {supportLegA, supportLegB};

  Serial.print("Three-leg recovery: using Leg ");
  Serial.print(liftLegNumber);
  Serial.print(" as balance/reposition leg and walking with ");
  Serial.print(supportLegA);
  Serial.print("+");
  Serial.print(supportLegB);
  Serial.println(".");

  slowMove(liftLegNumber, 1, HOME_HIP, WALK_STEP_DELAY);
  moveLegToGround(liftLegNumber);
  prepareWalkingLegs(supportPair, 2);

  for (int cycle = 0; cycle < cycles; cycle++) {
    walkThreeLegRecoveryCycle(liftLegNumber, supportLegA, supportLegB);
  }
}

void walkForward(int cycles) {
  const int fourLegs[] = {1, 2, 4, 5};
  int availableLegs[6];
  int legCount = 0;
  int liftLegNumber = 0;
  int supportLegA = 0;
  int supportLegB = 0;
  int missingLeg = 0;

  updateLegAvailability();
  printLegAvailability();
  legCount = collectAvailableLegs(availableLegs);

  Serial.print("Walking forward for ");
  Serial.print(cycles);
  Serial.println(cycles == 1 ? " cycle." : " cycles.");

  if (areAllLegsAvailable()) {
    walkForwardTripod(cycles);
  } else if (legCount == 5) {
    for (int leg = 1; leg <= 6; leg++) {
      if (!isLegAvailable(leg)) {
        missingLeg = leg;
        break;
      }
    }

    walkForwardFiveLegRecovery(missingLeg, cycles);
  } else if (areLegsAvailable(fourLegs, 4)) {
    walkForwardFourLeg(cycles);
  } else if (legCount == 3 && findThreeLegRecoveryGait(availableLegs, legCount, liftLegNumber, supportLegA, supportLegB)) {
    walkForwardThreeLegRecovery(liftLegNumber, supportLegA, supportLegB, cycles);
  } else if (legCount == 2) {
    walkForwardPair(availableLegs[0], availableLegs[1], cycles);
  } else if (legCount == 1) {
    walkForwardSingleLeg(availableLegs[0], cycles);
  } else {
    Serial.println("Forward walk cancelled. Need all 6 legs, the four-leg recovery set, exactly 1 leg, exactly 2 legs, or exactly 3 neighboring legs.");
    return;
  }

  Serial.println("Forward walk complete.");
}

void liftLegGroup(const int legs[], int legCount) {
  int liftFemurAngles[6];
  int liftTibiaAngles[6];

  for (int i = 0; i < legCount; i++) {
    liftFemurAngles[i] = WALK_FEMUR_LIFT;
    liftTibiaAngles[i] = WALK_TIBIA_LIFT;
  }

  moveJointGroup(legs, legCount, 2, liftFemurAngles, WALK_STEP_DELAY);
  moveJointGroup(legs, legCount, 3, liftTibiaAngles, WALK_STEP_DELAY);
}

void rotateRobotCycle(int direction) {
  int legs[6];
  int hipAngles[6];
  int legCount = collectAvailableLegs(legs);

  if (legCount == 0) {
    Serial.println("Rotate cancelled. No legs are connected.");
    return;
  }

  for (int i = 0; i < legCount; i++) {
    hipAngles[i] = rotateHipAngle(direction);
  }

  liftLegGroup(legs, legCount);
  moveJointGroup(legs, legCount, 1, hipAngles, WALK_STEP_DELAY);
  lowerWalkingGroup(legs, legCount);

  for (int i = 0; i < legCount; i++) {
    hipAngles[i] = HOME_HIP;
  }

  moveJointGroup(legs, legCount, 1, hipAngles, WALK_STEP_DELAY);
  delay(WALK_SETTLE_DELAY);
}

void rotateRobot(int direction, int cycles) {
  updateLegAvailability();
  printLegAvailability();

  Serial.print(direction > 0 ? "Rotating right for " : "Rotating left for ");
  Serial.print(cycles);
  Serial.println(cycles == 1 ? " cycle." : " cycles.");

  for (int cycle = 0; cycle < cycles; cycle++) {
    rotateRobotCycle(direction);
  }

  Serial.println("Rotate complete.");
}

/**
 * Positions all 18 servos safely to their default stance configuration on power-up
 */
void homeRobot() {
  updateLegAvailability();

  for (int leg = 1; leg <= 6; leg++) {
    if (!isLegAvailable(leg)) {
      Serial.print("Skipping Leg ");
      Serial.print(leg);
      Serial.println(" because it is not connected.");
      continue;
    }

    Serial.print("Homing Leg "); Serial.println(leg);
    
    // Commands are sequentially sent with small pauses to prevent high current surges
    setLegJoint(leg, 1, HOME_HIP);  // Hip
    delay(40);
    setLegJoint(leg, 2, HOME_FEMUR); // Femur
    delay(40);
    setLegJoint(leg, 3, HOME_TIBIA); // Tibia
    delay(40);
  }
  Serial.println("All 18 servos successfully mapped, tracked, and homed!");
}

void setupBluetooth() {
  SerialBT.begin(BLUETOOTH_NAME);
  Serial.print("Bluetooth Serial ready. Pair with ");
  Serial.println(BLUETOOTH_NAME);
}

void printSerialHelpTo(Print& output) {
  output.println();
  output.println("Serial/Bluetooth commands:");
  output.println("  1-6 = lift that connected leg");
  output.println("  f   = walk forward 1 cycle using the best available gait");
  output.println("  fN  = walk forward N cycles, example f3");
  output.println("  r   = rotate right 1 cycle using all available legs");
  output.println("  rN  = rotate right N cycles, example r3");
  output.println("  l   = rotate left 1 cycle using all available legs");
  output.println("  lN  = rotate left N cycles, example l3");
  output.println("  h   = return all connected legs to initial position");
  output.println("  ?   = print this help");
  output.println();
}

void printSerialHelp() {
  printSerialHelpTo(Serial);
  printSerialHelpTo(SerialBT);
}

void handleCommandInput(String input, Print& output) {
  input.trim();
  input.toLowerCase();

  if (input.length() == 0) {
    return;
  }

  if (input == "h") {
    output.println("Returning connected legs to initial position...");
    homeRobot();
    return;
  }

  if (input == "?") {
    printSerialHelpTo(output);
    return;
  }

  if (input.charAt(0) == 'f') {
    int cycles = 1;

    if (input.length() > 1) {
      cycles = input.substring(1).toInt();
      if (cycles < 1) {
        cycles = 1;
      }
    }

    walkForward(cycles);
    return;
  }

  if (input.charAt(0) == 'r' || input.charAt(0) == 'l') {
    int cycles = 1;
    int direction = (input.charAt(0) == 'r') ? 1 : -1;

    if (input.length() > 1) {
      cycles = input.substring(1).toInt();
      if (cycles < 1) {
        cycles = 1;
      }
    }

    rotateRobot(direction, cycles);
    return;
  }

  if (input.length() == 1 && input.charAt(0) >= '1' && input.charAt(0) <= '6') {
    liftLeg(input.charAt(0) - '0');
    return;
  }

  output.println("Unknown command. Enter 1-6 to lift a leg, f to walk, r/l to rotate, h to home, or ? for help.");
}

void handleSerialCommands() {
  handleUsbSerialCommands();
  handleBluetoothCommands();
}

void handleUsbSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  handleCommandInput(Serial.readStringUntil('\n'), Serial);
}

void handleBluetoothCommands() {
  if (!SerialBT.available()) {
    return;
  }

  handleCommandInput(SerialBT.readStringUntil('\n'), SerialBT);
}
