#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

// Motor pins
#define M1_pwm 6   // Front Left
#define M1_dir 38
#define M2_pwm 4   // Front Right
#define M2_dir 34
#define M3_pwm 10  // Rear Left
#define M3_dir 26
#define M4_pwm 11  // Rear Right
#define M4_dir 28

// PID variables
float Kp = 4.0;
float Ki = 0.05;
float Kd = 1.0;
float integral = 0;
float lastError = 0;

// Forward speed
int FORWARD_SPEED = 240;
float targetYaw = 0;
bool firstRun = true;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000); // Added delay for stability
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  // Motor pins setup
  pinMode(M1_pwm, OUTPUT);
  pinMode(M1_dir, OUTPUT);
  pinMode(M2_pwm, OUTPUT);
  pinMode(M2_dir, OUTPUT);
  pinMode(M3_pwm, OUTPUT);
  pinMode(M3_dir, OUTPUT);
  pinMode(M4_pwm, OUTPUT);
  pinMode(M4_dir, OUTPUT);

  stopRobot();

  // Get initial yaw and set as target
  mpu.update();
  targetYaw = mpu.getAngleZ();

  Serial.print("Initial target yaw set to: ");
  Serial.println(targetYaw);



  // START DRIVING FORWARD IMMEDIATELY
  Serial.println("STARTING FORWARD DRIVE WITH CORRECTION");
}

void loop() {
  mpu.update();

  if ((millis() - timer) > 10) { // update every 10ms
    float currentAngle = mpu.getAngleZ();

    // Normalize angle between -180 and 180
    currentAngle = fmod(currentAngle, 360.0);
    if (currentAngle > 180) currentAngle -= 360.0;
    if (currentAngle < -180) currentAngle += 360.0;

    // On first run, update target to current angle
    if (firstRun) {
      targetYaw = currentAngle;
      firstRun = false;
      Serial.print("First run - Target yaw updated to: ");
      Serial.println(targetYaw);
    }

    // Calculate PID correction
    float error = targetYaw - currentAngle;

    // Handle angle wrap-around for minimal error
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    integral += error * 0.01; // dt is approximately 0.01s
    integral = constrain(integral, -50, 50); // Anti-windup

    float derivative = (error - lastError) / 0.01;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    // Limit correction to prevent over-steering
    correction = constrain(correction, -100, 100);

    // Calculate motor speeds
    int leftSpeed = FORWARD_SPEED - correction;
    int rightSpeed = FORWARD_SPEED + correction;

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // ALWAYS DRIVE FORWARD WITH CORRECTION
    driveForward(leftSpeed, rightSpeed, error);

    // Debug output with correction direction
    Serial.print("Yaw: ");
    Serial.print(currentAngle, 1);
    Serial.print(" | Target: ");
    Serial.print(targetYaw, 1);
    Serial.print(" | Error: ");
    Serial.print(error, 1);
    Serial.print(" | Correction: ");
    Serial.print(correction, 1);

    // Show correction direction clearly
    if (correction > 5) {
      Serial.print(" | Action: Correcting LEFT");
    } else if (correction < -5) {
      Serial.print(" | Action: Correcting RIGHT");
    } else {
      Serial.print(" | Action: Going STRAIGHT");
    }

    Serial.print(" | L: ");
    Serial.print(leftSpeed);
    Serial.print(" | R: ");
    Serial.print(rightSpeed);
    Serial.print(" | Base: ");
    Serial.println(FORWARD_SPEED);
    
    timer = millis();
  }
}

void driveForward(int speedLeft, int speedRight, int error) {
  // Apply deadzone for small errors (within ±10 degrees)
  if (error > -1 && error < 1) {
    // Small error - drive straight with equal speed
    speedLeft = FORWARD_SPEED;
    speedRight = FORWARD_SPEED;
  }
  else if (error >= 1) {
    // Large positive error - robot turned right, correct left
    // Left motors faster, right motors slower
    speedLeft = FORWARD_SPEED + abs(error) * 2;
    speedRight = FORWARD_SPEED - abs(error) * 2;
  }
  else if (error <= -1) {
    // Large negative error - robot turned left, correct right  
    // Right motors faster, left motors slower
    speedLeft = FORWARD_SPEED - abs(error) * 2;
    speedRight = FORWARD_SPEED + abs(error) * 2;
  }

  // Constrain speeds
  speedLeft = constrain(speedLeft, 0, 255);
  speedRight = constrain(speedRight, 0, 255);

  // Drive motors with corrected speeds
  // Left side motors (M1, M3) - Forward
  digitalWrite(M1_dir, LOW);
  digitalWrite(M3_dir, HIGH);
  analogWrite(M1_pwm, speedLeft);
  analogWrite(M3_pwm, speedLeft);

  // Right side motors (M2, M4) - Forward
  digitalWrite(M2_dir, HIGH);
  digitalWrite(M4_dir, LOW);
  analogWrite(M2_pwm, speedRight);
  analogWrite(M4_pwm, speedRight);
}
void stopRobot() {
  analogWrite(M1_pwm, 0);
  analogWrite(M2_pwm, 0);
  analogWrite(M3_pwm, 0);
  analogWrite(M4_pwm, 0);
}

// Function to manually reset target direction if needed
void resetTargetDirection() {
  mpu.update();
  targetYaw = mpu.getAngleZ();
  integral = 0;
  lastError = 0;
  Serial.print("New target direction set to: ");
  Serial.println(targetYaw);
}
