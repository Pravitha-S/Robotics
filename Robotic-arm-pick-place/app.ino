/*
  app.ino
  ESP32 Robotic Arm with MG995 Servos (360° capable)
  Demonstrates a simple pick & place motion sequence.
  ⚡ Challenge: MG995 reacts to angle commands as speed-dependent motions,
     so controlling speed requires careful timing.
*/

#include <ESP32Servo.h>   // Library to control servos on ESP32

// Declare servo objects
Servo shoulder, wrist, elbow, gripper, base;

// Assign pins for each servo (adjust these if wiring changes)
#define SHOULDER_PIN 13
#define WRIST_PIN    12
#define ELBOW_PIN    15
#define GRIPPER_PIN  14
#define BASE_PIN     27

// Custom function: rotate servo using a "speed-like" value
// speed range: -90 (full reverse) to +90 (full forward), 0 = stop/hold
void rotateWithSpeed(Servo &servo, int speed) {
  speed = constrain(speed, -90, 90);          // limit values
  int pwm = map(speed, -90, 90, 0, 180);      // map -90..90 → 0..180
  servo.write(pwm);                           // write to servo
}

void setup() {
  // Attach servos to pins
  shoulder.attach(SHOULDER_PIN);
  wrist.attach(WRIST_PIN);
  elbow.attach(ELBOW_PIN);
  gripper.attach(GRIPPER_PIN);
  base.attach(BASE_PIN);

  delay(500);  // Initial pause to stabilize

  // === Pick & Place Sequence ===

  // 1. Shoulder move forward
  rotateWithSpeed(shoulder, 30);   // move with "speed" 30
  delay(1000);                     // wait for motion
  rotateWithSpeed(shoulder, 0);    // stop shoulder
  delay(300);

  // 2. Wrist move
  rotateWithSpeed(wrist, 30);
  delay(900);
  rotateWithSpeed(wrist, 0);
  delay(300);

  // 3. Elbow down
  rotateWithSpeed(elbow, 40);
  delay(2200);
  rotateWithSpeed(elbow, 0);
  delay(300);

  // 4. Gripper close (pick object)
  rotateWithSpeed(gripper, 40);    // may need to reverse direction
  delay(500);
  rotateWithSpeed(gripper, 0);
  delay(500);

  // 5. Elbow up
  rotateWithSpeed(elbow, -40);
  delay(700);
  rotateWithSpeed(elbow, 0);
  delay(300);

  // 6. Base rotate sideways
  rotateWithSpeed(base, 10);
  delay(1500);
  rotateWithSpeed(base, 0);
  delay(300);

  // 7. Gripper open (place object)
  rotateWithSpeed(gripper, -40);   // reverse direction to open
  delay(500);
  rotateWithSpeed(gripper, 0);
}

void loop() {
  // Empty: sequence only runs once at startup
}
