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
  poseCount = EEPROM.read(0);
  poseCount = constrain(poseCount, 0, MAX_POSES);
}

void loop() {

  static bool lastSaveButtonState = HIGH;
  static bool lastPlayButtonState = HIGH;

  bool currentSaveButtonState = digitalRead(saveButtonPin);
  bool currentPlayButtonState = digitalRead(playButtonPin);

  if (currentSaveButtonState != lastSaveButtonState){
    if (currentSaveButtonState == LOW) {
      Serial.println("Save Button Pressed");
    } else {
      Serial.println("Save Button Released");
    }
    lastSaveButtonState = currentSaveButtonState;
    delay(50);
  }
  
  if (currentPlayButtonState != lastPlayButtonState) {
    if (currentPlayButtonState == LOW) {
      Serial.println("Play Button Pressed");
    } else {
      Serial.println("Play Button Released");
    }
    lastPlayButtonState = currentPlayButtonState;
    delay(50);
  }

  if (digitalRead(playButtonPin) == LOW) {
    delay(200);
    isPlaying = !isPlaying;
    playIndex = 0;
    lastStepTime = millis();
    while (digitalRead(playButtonPin) == LOW);  // Wait for release
  }

  if (!isPlaying) {
    // Manual knob control
    for (int i = 0; i < NUM_SERVOS; i++) {
      int val = analogRead(potPins[i]);
      int angle = map(val, 0, 1023, 0, 180);
      currentPose[i] = angle;
      servos[i].write(angle);
    }

    if (digitalRead(saveButtonPin) == LOW && poseCount < MAX_POSES) {
      delay(200);
      savePose(poseCount, currentPose);
      poseCount++;
      EEPROM.write(0, poseCount);  // Update pose count
      Serial.print("Saved pose "); Serial.println(poseCount);
      while (digitalRead(saveButtonPin) == LOW);  // Wait for release
    }
  } else {
    // Play saved poses in loop
    if (poseCount > 0 && millis() - lastStepTime >= stepDelay) {
      int pose[NUM_SERVOS];
      loadPose(playIndex, pose);
      for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].write(pose[i]);
      }
      playIndex = (playIndex + 1) % poseCount;
      lastStepTime = millis();
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

void clearEEPROM() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }
  Serial.println("EEPROM cleared!");
}