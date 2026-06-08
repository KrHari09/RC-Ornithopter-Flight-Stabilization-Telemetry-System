// ══════════════════════════════════════════════════════════════
//  flight_controller_basic.ino
//  RC Ornithopter — Manual RC Passthrough (No PID)
//  Hardware: Arduino Nano + FlySky FS-iA6 + ESC + 2x SG90
// ══════════════════════════════════════════════════════════════

#include <Servo.h>
#include <Wire.h>

// ── Pin Definitions ──────────────────────────────────────────
const int escPin            = 9;
const int elevatorServoPin  = 10;
const int rudderServoPin    = 11;
const int throttleInPin     = 2;
const int pitchInPin        = 3;
const int yawInPin          = 4;

// ── Servo Objects ────────────────────────────────────────────
Servo flappingMotor;
Servo elevatorServo;
Servo rudderServo;

// ── RC Input Variables ────────────────────────────────────────
int throttleValue = 0;
int pitchValue    = 0;
int yawValue      = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to 0 = wake up
  Wire.endTransmission(true);

  // Attach servos and ESC
  flappingMotor.attach(escPin, 1000, 2000);
  elevatorServo.attach(elevatorServoPin);
  rudderServo.attach(rudderServoPin);

  // ESC arming: send minimum throttle for 2 seconds
  flappingMotor.writeMicroseconds(1000);
  delay(2000);

  // Centre the tail servos
  elevatorServo.write(90);
  rudderServo.write(90);

  Serial.println("Manual RC Flight Controller Ready.");
}

void loop() {
  // 1. Read RC Receiver PWM signals (1000-2000 microseconds)
  //    Note: pulseIn is blocking. For a more advanced version,
  //    use Pin Change Interrupts or a library like PinChangeInterrupt.
  throttleValue = pulseIn(throttleInPin, HIGH, 25000);
  pitchValue    = pulseIn(pitchInPin,    HIGH, 25000);
  yawValue      = pulseIn(yawInPin,      HIGH, 25000);

  // 2. Signal-loss failsafe: if no signal, default to safe values
  if (throttleValue == 0) throttleValue = 1000; // Cut motor on signal loss
  if (pitchValue    == 0) pitchValue    = 1500; // Centre elevator
  if (yawValue      == 0) yawValue      = 1500; // Centre rudder

  // 3. Map RC inputs to servo angles (45-135° for mechanical travel limits)
  int elevatorAngle = map(pitchValue, 1000, 2000, 45, 135);
  int rudderAngle   = map(yawValue,   1000, 2000, 45, 135);

  // 4. Write outputs to actuators
  flappingMotor.writeMicroseconds(throttleValue); // Controls flapping speed
  elevatorServo.write(elevatorAngle);             // Controls pitch (up/down)
  rudderServo.write(rudderAngle);                 // Controls yaw (left/right)

  // 5. Debug output (comment out during flight to save processing time)
  Serial.print("THR:"); Serial.print(throttleValue);
  Serial.print(" | PITCH:"); Serial.print(pitchValue);
  Serial.print(" | YAW:"); Serial.print(yawValue);
  Serial.print(" | ELEV:"); Serial.print(elevatorAngle);
  Serial.print(" | RUD:"); Serial.println(rudderAngle);

  delay(20); // ~50 Hz loop rate
}

