#include <EEPROM.h>
#include <Servo.h>

#define NUM_SERVOS 6
#define MAX_POSES 10

const int potPins[NUM_SERVOS] = {A0, A1, A2, A3, A4, A5};
const int servoPins[NUM_SERVOS] = {2, 3, 4, 5, 6, 7};
const int saveButtonPin = 8;
const int playButtonPin = 9;

Servo servos[NUM_SERVOS];
int currentPose[NUM_SERVOS];
bool isPlaying = false;
int poseCount = 0;
int playIndex = 0;
unsigned long lastStepTime = 0;
const unsigned long stepDelay = 1000;

void setup() {
  Serial.begin(9600);

  // Uncomment this to clear saved poses
  clearEEPROM();

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
  }

  pinMode(saveButtonPin, INPUT_PULLUP);
  pinMode(playButtonPin, INPUT_PULLUP);

  // Read saved pose count
  poseCount = EEPROM.read(0);#include <EEPROM.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#define NUM_SERVOS 6
#define MAX_POSES 10
#define SMOOTH_STEP 2  // Max degrees per step

const int potPins[NUM_SERVOS] = {A0, A1, A2, A3, A4, A5};
const int servoPins[NUM_SERVOS] = {2, 3, 4, 5, 6, 7};
const int saveButtonPin = 8;

Servo servos[NUM_SERVOS];
int currentPose[NUM_SERVOS];      // live positions (for manual mode)
int targetPose[NUM_SERVOS];       // target positions (during playback)
int currentServoAngles[NUM_SERVOS]; // interpolated servo positions

bool isPlaying = false;
bool inRange = false;
String detectedColor = "";

int poseCount = 0;
int playIndex = 0;
unsigned long lastStepTime = 0;
const unsigned long stepDelay = 50; // shorter for smoother stepping
unsigned long lastMoveTime = 0;
const unsigned long moveInterval = 20; // control smoothing speed

// ROS setup
ros::NodeHandle nh;

void inRangeCallback(const std_msgs::Bool& msg) {
  inRange = msg.data;
}

void colorCallback(const std_msgs::String& msg) {
  detectedColor = msg.data;
}

ros::Subscriber<std_msgs::Bool> sub_inRange("/in_range", &inRangeCallback);
ros::Subscriber<std_msgs::String> sub_detected("/detected_color", &colorCallback);

void setup() {
  //Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub_inRange);
  nh.subscribe(sub_detected);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    currentServoAngles[i] = 90;
    servos[i].write(currentServoAngles[i]);
  }

  pinMode(saveButtonPin, INPUT_PULLUP);

  poseCount = EEPROM.read(0);
  poseCount = constrain(poseCount, 0, MAX_POSES);
}

void loop() {
  nh.spinOnce();

  if (inRange && detectedColor == "yellow") {
    if (!isPlaying) {
      isPlaying = true;
      playIndex = 0;
      lastStepTime = millis();
      loadPose(playIndex, targetPose);
    }
  }

  if (!isPlaying) {
    // Manual knob control
    for (int i = 0; i < NUM_SERVOS; i++) {
      int val = analogRead(potPins[i]);
      int angle = map(val, 0, 1023, 0, 180);
      currentPose[i] = angle;
      servos[i].write(angle);
      currentServoAngles[i] = angle;  // Sync internal state
    }

    if (digitalRead(saveButtonPin) == LOW && poseCount < MAX_POSES) {
      delay(200);
      savePose(poseCount, currentPose);
      poseCount++;
      EEPROM.write(0, poseCount);
      while (digitalRead(saveButtonPin) == LOW);
    }
  } else {
    // Smooth movement
    if (millis() - lastMoveTime >= moveInterval) {
      bool allAtTarget = true;
      for (int i = 0; i < NUM_SERVOS; i++) {
        int delta = targetPose[i] - currentServoAngles[i];
        if (abs(delta) > SMOOTH_STEP) {
          currentServoAngles[i] += (delta > 0) ? SMOOTH_STEP : -SMOOTH_STEP;
          allAtTarget = false;
        } else if (delta != 0) {
          currentServoAngles[i] = targetPose[i]; // snap to target
        }
        servos[i].write(currentServoAngles[i]);
      }

      lastMoveTime = millis();

      // When finished current pose
      if (allAtTarget && millis() - lastStepTime >= stepDelay) {
        playIndex++;
        if (playIndex < poseCount) {
          loadPose(playIndex, targetPose);
          lastStepTime = millis();
        } else {
          isPlaying = false;
        }
      }
    }
  }
}

void savePose(int index, int pose[]) {
  int addr = 1 + index * NUM_SERVOS;
  for (int i = 0; i < NUM_SERVOS; i++) {
    EEPROM.write(addr + i, pose[i]);
  }
}

void loadPose(int index, int pose[]) {
  int addr = 1 + index * NUM_SERVOS;
  for (int i = 0; i < NUM_SERVOS; i++) {
    pose[i] = EEPROM.read(addr + i);
  }
}
