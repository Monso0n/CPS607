// Motor control pins for Romeo board
#define M1_DIR 4
#define M1_PWM 5  // PWM pin
#define M2_DIR 7
#define M2_PWM 6  // PWM pin

// 6 is echo (receives)
// 7 trigger ()

// Sensor pins
const uint8_t frontLeftSensorPin = 10;   // Front left digital IR sensor (Digital Pin 10)
const uint8_t frontRightSensorPin = 9;   // Front right digital IR sensor (Digital Pin 9)
const uint8_t backMiddleSensorPin = A5;   // Back left analog IR sensor (Analog Pin A5)

void setup() {
  // Initialize motor control pins
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);

  // Initialize Serial communication for debugging (optional)
  Serial.begin(9600);
}

// Function to move forward
void moveForward() {
  setMotor(M1_DIR, M1_PWM, HIGH, 120); // Left motors forward at speed 200
  setMotor(M2_DIR, M2_PWM, HIGH, 120); // Right motors forward at speed 200
}

void reverseShort() {
  setMotor(M1_DIR, M1_PWM, LOW, 120); // Left motors backward
  setMotor(M2_DIR, M2_PWM, LOW, 120); // Right motors backward
  Serial.println("Reversing slightly");
  delay(300);  // Reverse for 0.3 seconds
  stopMotors();
}

void stopMotors() {
  setMotor(M1_DIR, M1_PWM, HIGH, 0); // Stop left motors
  setMotor(M2_DIR, M2_PWM, HIGH, 0); // Stop right motors
  Serial.println("Motors stopped");
}

void turnRight() {
  Serial.println("Turning left");
  // Left wheels forward, right wheels backward to turn left
  setMotor(M1_DIR, M1_PWM, HIGH, 200);  // Left motors forward
  setMotor(M2_DIR, M2_PWM, LOW, 200); // Right motors backward
  delay(500); // Adjust this delay for how long you want to turn
  stopMotors();
}

// Function to control a motor
void setMotor(int dirPin, int pwmPin, int dir, int speed) {
  digitalWrite(dirPin, dir);   // Set motor direction
  analogWrite(pwmPin, speed);  // Set motor speed (0-255)
}

void loop() {
  // Read sensor values
  int frontLeftValue = digitalRead(frontLeftSensorPin);
  int frontRightValue = digitalRead(frontRightSensorPin);
  int backMiddleValue = digitalRead(backMiddleSensorPin);

  // Debugging output (optional)
  Serial.print("Front Left Sensor: "); Serial.print(frontLeftValue);
  Serial.print(" | Front Right Sensor: "); Serial.print(frontRightValue);
  Serial.print(" | Back Middle Sensor: "); Serial.print(backMiddleValue);
  Serial.println();

  // Check for edge detection
  if (frontLeftValue == 1 || frontRightValue == 1) {
    Serial.println("Edge detected!");
    reverseShort();  // Move backward slightly
    turnRight();      // Turn left
  } else {
    moveForward();   // Continue moving forward
  }

  delay(100);  // Small delay to prevent overwhelming the sensors
}
