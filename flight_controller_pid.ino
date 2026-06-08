// ══════════════════════════════════════════════════════════════
//  flight_controller_pid.ino
//  RC Ornithopter — Hybrid RC + PID Pitch Stabilization
//  Hardware: Arduino Nano + MPU6050 + FlySky FS-iA6 + ESC + 2x SG90
//
//  System Overview:
//    RC Transmitter sets the TARGET pitch angle (setpoint).
//    MPU6050 IMU measures the ACTUAL pitch angle via complementary filter.
//    PID algorithm calculates the ERROR and adjusts the elevator servo
//    automatically to keep the ornithopter level against disturbances.
// ══════════════════════════════════════════════════════════════

#include <Servo.h>
#include <Wire.h>

// ── Pin Definitions ──────────────────────────────────────────
const int escPin           = 9;
const int elevatorServoPin = 10;
const int rudderServoPin   = 11;
const int throttleInPin    = 2;
const int pitchRCInPin     = 3;
const int yawRCInPin       = 4;

// ── Servo / ESC Objects ───────────────────────────────────────
Servo flappingMotor;
Servo elevatorServo;
Servo rudderServo;

// ── PID Tuning Constants ──────────────────────────────────────
// IMPORTANT: Tune these on the bench before flying!
// Start with Ki = Kd = 0, increase Kp until oscillation, then add Kd.
float Kp = 1.2;    // Proportional gain
float Ki = 0.05;   // Integral gain (eliminates steady-state error)
float Kd = 0.5;    // Derivative gain (dampens oscillations)

// ── PID Internal State ────────────────────────────────────────
float error         = 0;
float previousError = 0;
float integral      = 0;
float derivative    = 0;
float pidOutput     = 0;

// ── IMU & Timing Variables ────────────────────────────────────
unsigned long previousTime = 0;
float currentPitch   = 0;    // Filtered pitch angle (degrees)
float accelPitch     = 0;    // Pitch from accelerometer
float gyroPitchRate  = 0;    // Pitch rate from gyroscope (deg/s)

// ── MPU6050 Register Addresses ────────────────────────────────
const int MPU_ADDR   = 0x68;
const int PWR_MGMT_1 = 0x6B;
const int ACCEL_XOUT = 0x3B;
const int GYRO_XOUT  = 0x43;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // ── Wake up MPU6050 ──────────────────────────────────────
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);   // Clear sleep bit — wake up
  Wire.endTransmission(true);

  // ── Attach and arm ESC ────────────────────────────────────
  flappingMotor.attach(escPin, 1000, 2000);
  elevatorServo.attach(elevatorServoPin);
  rudderServo.attach(rudderServoPin);

  // Send minimum throttle for ESC arming sequence
  flappingMotor.writeMicroseconds(1000);
  Serial.println("Arming ESC...");
  delay(2000);

  // Centre servos
  elevatorServo.write(90);
  rudderServo.write(90);

  Serial.println("PID Flight Controller Ready.");
  Serial.println("Format: Target | Actual | Error | PID | Servo");
}

void loop() {
  // ── Timing ────────────────────────────────────────────────
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0;  // Convert ms → seconds
  if (elapsedTime <= 0.001) return;   // Skip if too short (prevents division by near-zero)

  // ══════════════════════════════════════════════════════════
  //  STEP 1: Read RC Input → Target Pitch Setpoint
  // ══════════════════════════════════════════════════════════
  int rcPitchSignal = pulseIn(pitchRCInPin, HIGH, 25000);
  int rcYawSignal   = pulseIn(yawRCInPin,   HIGH, 25000);
  int rcThrottle    = pulseIn(throttleInPin, HIGH, 25000);

  // Failsafe defaults
  if (rcPitchSignal == 0) rcPitchSignal = 1500;
  if (rcYawSignal   == 0) rcYawSignal   = 1500;
  if (rcThrottle    == 0) rcThrottle    = 1000;

  // Map RC pitch stick (1000-2000µs) → target pitch angle (-30° to +30°)
  float targetPitch = map(rcPitchSignal, 1000, 2000, -30, 30);

  // Rudder: direct RC passthrough (no PID on yaw)
  int rudderAngle = map(rcYawSignal, 1000, 2000, 45, 135);

  // ══════════════════════════════════════════════════════════
  //  STEP 2: Read MPU6050 → Actual Pitch
  // ══════════════════════════════════════════════════════════

  // -- Read Accelerometer (registers 0x3B–0x40) --
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  // Raw 16-bit values, divided by sensitivity (±2g range = 16384 LSB/g)
  float accX = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  float accY = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  float accZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;

  // -- Read Gyroscope (registers 0x43–0x44: Gyro X only for pitch) --
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  // Divided by sensitivity (±250°/s range = 131 LSB per °/s)
  gyroPitchRate = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;

  // ══════════════════════════════════════════════════════════
  //  STEP 3: Complementary Filter
  //  CRITICAL for ornithopters — suppresses high-freq vibration noise.
  //
  //  Formula: angle = α × (angle + gyro × dt) + (1-α) × accel_angle
  //    α = 0.96 (trusts gyro for short-term, accel for long-term)
  // ══════════════════════════════════════════════════════════
  accelPitch   = atan2(accY, sqrt(accX * accX + accZ * accZ)) * (180.0 / PI);
  currentPitch = 0.96 * (currentPitch + gyroPitchRate * elapsedTime)
               + 0.04 * accelPitch;

  // ══════════════════════════════════════════════════════════
  //  STEP 4: PID Calculation
  //  Output = Kp*e + Ki*∫e·dt + Kd*(de/dt)
  // ══════════════════════════════════════════════════════════
  error     = targetPitch - currentPitch;

  // Integral with anti-windup clamp (prevents integrator saturation)
  integral += error * elapsedTime;
  integral  = constrain(integral, -20.0, 20.0);

  // Derivative (rate of change of error)
  derivative = (error - previousError) / elapsedTime;

  // PID output
  pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Update state for next iteration
  previousError = error;
  previousTime  = currentTime;

  // ══════════════════════════════════════════════════════════
  //  STEP 5: Apply to Actuators
  // ══════════════════════════════════════════════════════════

  // Elevator: base angle is 90° (tail centred). Add PID correction.
  // Constrain to physical limits of servo (45°–135°)
  int finalElevatorAngle = constrain(90 + (int)pidOutput, 45, 135);

  // Apply to servos
  elevatorServo.write(finalElevatorAngle);
  rudderServo.write(rudderAngle);

  // Apply throttle directly to ESC
  flappingMotor.writeMicroseconds(rcThrottle);

  // ══════════════════════════════════════════════════════════
  //  STEP 6: Serial Telemetry
  //  Monitor these values in the Serial Plotter (Tools > Serial Plotter)
  //  to tune PID gains in real time.
  // ══════════════════════════════════════════════════════════
  Serial.print("Target:");    Serial.print(targetPitch, 1);
  Serial.print(",Actual:");   Serial.print(currentPitch, 1);
  Serial.print(",Error:");    Serial.print(error, 2);
  Serial.print(",PID:");      Serial.print(pidOutput, 2);
  Serial.print(",Servo:");    Serial.println(finalElevatorAngle);

  delay(10);  // ~100 Hz loop rate
}
