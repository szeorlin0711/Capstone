#include <EEPROM.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>

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

const int homePose[NUM_SERVOS] = {12, 95, 30, 180, 0, 80};

// ROS setup
ros::NodeHandle nh;
std_msgs::String color_msg;
ros::Subscriber<std_msgs::String> color_sub("/detected_colors", colorCallback);

void moveToPose(const int pose[]);

void setup() {
  Serial.begin(9600);

  // Uncomment this to clear saved poses
  clearEEPROM();

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
  }

  pinMode(saveButtonPin, INPUT_PULLUP);
  pinMode(playButtonPin, INPUT_PULLUP);

  // Initialize ROS
  nh.initNode();
  nh.subscribe(color_sub);

  // Read saved pose count
  poseCount = EEPROM.read(0);
  poseCount = constrain(poseCount, 0, MAX_POSES);
}

void loop() {
  static bool lastSaveButtonState = HIGH;
  static bool lastPlayButtonState = HIGH;

  bool currentSaveButtonState = digitalRead(saveButtonPin);
  bool currentPlayButtonState = digitalRead(playButtonPin);

  // Listen for ROS messages
  nh.spinOnce();  // This processes any incoming ROS messages

  // Rest of your logic...

  if (!isPlaying) {
    // Manual knob control...
  } else {
    // Play saved poses...
    if (poseCount > 0 && millis() - lastStepTime >= stepDelay) {
      int pose[NUM_SERVOS];
      loadPose(playIndex, pose);
      moveToPose(pose);

      playIndex++;
      lastStepTime = millis();

      if (playIndex >= poseCount) {
        Serial.println("Playback finished");
        moveToPose(homePose); // Go back to home after playback
        delay(500);
        isPlaying = false;
      }
    }
  }
}

void colorCallback(const std_msgs::String& msg) {
  if (msg.data == "yellow") {
    if (!isPlaying) {
      moveToPose(homePose);  // Move to home before playback
      delay(500);             // Allow time for move
      playIndex = 0;
      lastStepTime = millis();
      isPlaying = true;
      Serial.println("Starting Playback");
    }
  }
}

void savePose(int index, int pose[]) {
  int addr = 1 + index * NUM_SERVOS;
  Serial.print("Saving pose "); Serial.print(index); Serial.print(" at EEPROM addr "); Serial.println(addr);
  for (int i = 0; i < NUM_SERVOS; i++) {
    EEPROM.write(addr + i, pose[i]);
    Serial.print("  Servo "); Serial.print(i); Serial.print(" = "); Serial.println(pose[i]);
  }
}

void loadPose(int index, int pose[]) {
  int addr = 1 + index * NUM_SERVOS;
  Serial.print("Loading pose "); Serial.print(index); Serial.print(" from EEPROM addr "); Serial.println(addr);
  for (int i = 0; i < NUM_SERVOS; i++) {
    pose[i] = EEPROM.read(addr + i);
    Serial.print("  Servo "); Serial.print(i); Serial.print(" = "); Serial.println(pose[i]);
  }
}

void moveToPose(const int pose[]) {
  const int stepDelay = 10; // ms between steps
  const int maxSteps = 30;  // number of steps for smooth motion. More steps = smoother

  int startAngles[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    startAngles[i] = currentPose[i]; // Use the most recent pose as the start
  }

  for (int step = 1; step <= maxSteps; step++) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      int angle = map(step, 0, maxSteps, startAngles[i], pose[i]);
      servos[i].write(angle);
    }
    delay(stepDelay);
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    currentPose[i] = pose[i]; // Update the currentPose after move
    servos[i].write(pose[i]); // Ensure final position
  }
}

void clearEEPROM() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }
  Serial.println("EEPROM cleared!");
}
