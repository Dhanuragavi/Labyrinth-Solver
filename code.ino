#include <energia.h>
#include <Servo.h>
// Create a Servo object
Servo myServo;
// Define pin for the servo signal
int servoPin = PD_1; // PWM capable pin (Check the pin map for the Launchpad)
// Define pins for ultrasonic sensor
int trigPin = PF_1; // TRIG pin connected to PF1
int echoPin = PF_2; // ECHO pin connected to PF2
// Define pin for inbuilt LED
int ledPin = RED_LED; // Inbuilt LED pin (assuming it's connected to RED_LED pin)
float minDistance_cm = 15.0; // Minimum distance threshold in cm
float minDist_side = 40.0; // Minimum distance threshold in cm (side)
// Define pins for motor control
#define MOTOR_IN1 PA_2 // Left Int1 (PA2)
#define MOTOR_IN2 PA_3 // Left Int2 (PA3)
#define MOTOR_IN3 PA_4 // Right Int3 (PA4)
#define MOTOR_IN4 PA_5 // Right Int4 (PA5)
// Define pins for encoder
#define ENCODER_A PD_2 // Encoder output A connected to PD2
#define ENCODER_B PD_3 // Encoder output B connected to PD3
volatile long encoderCountLeft = 0; // Encoder count for the left motor
volatile long encoderCountRight = 0; // Encoder count for the right motor
int encoderPinAState = LOW;
int encoderPinBState = LOW;
// Timing for task intervals
unsigned long previousMillisTask1 = 0;
const long intervalTask1 = 100; // Task 1 interval (100 milliseconds)
// PID constants
float Kp = 0.1; // Proportional gain
// Function prototypes
void task1();
void moveForward();
void stop();
void turnRight();
void turnLeft();
void moveBackward();
void readEncoder();
void adjustMotorSpeed();
void setup() {
Serial.begin(9600);
// Initialize pins for ultrasonic sensor
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
pinMode(ledPin, OUTPUT);
// Initialize pins for motor control
pinMode(MOTOR_IN1, OUTPUT);
pinMode(MOTOR_IN2, OUTPUT);
pinMode(MOTOR_IN3, OUTPUT);
pinMode(MOTOR_IN4, OUTPUT);
// Initialize encoder pins
pinMode(ENCODER_A, INPUT_PULLUP);
pinMode(ENCODER_B, INPUT_PULLUP);
// Ensure motors are off
digitalWrite(MOTOR_IN1, LOW);
digitalWrite(MOTOR_IN2, LOW);
digitalWrite(MOTOR_IN3, LOW);
digitalWrite(MOTOR_IN4, LOW);
// Attach the servo to the pin
myServo.attach(servoPin);
// Set initial position
myServo.write(90); // Middle position (90 degrees)
}
void loop() {
unsigned long currentMillis = millis();
// Task 1: Runs every 100 milliseconds
if (currentMillis - previousMillisTask1 >= intervalTask1) {
previousMillisTask1 = currentMillis;
task1();
}
// Read the encoder in each loop iteration
readEncoder();
// Adjust motor speeds to maintain straight movement
adjustMotorSpeed();
// Continue moving forward if no obstacle is detected
moveForward();
}
// Task to read the ultrasonic sensor and control the motor
void task1() {
// Generate 10-microsecond pulse to TRIG pin
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Measure duration of pulse from ECHO pin
float duration_us = pulseIn(echoPin, HIGH);
// Calculate the distance in centimeters
float distance_cm = 0.017 * duration_us;
// Print the distance to Serial Monitor
Serial.print("Distance: ");
Serial.print(distance_cm);
Serial.println(" cm");
// Print encoder counts to Serial Monitor
Serial.print("Encoder Count Left: ");
Serial.println(encoderCountLeft);
Serial.print("Encoder Count Right: ");
Serial.println(encoderCountRight);
// Check if the distance is less than the minimum threshold
if (distance_cm < minDistance_cm) {
// If obstacle is closer than the threshold, turn on the LED and stop the robot
digitalWrite(ledPin, HIGH);
stop();
delay(1000); // Stop for 1 second
while (true) {
// Check the right direction
myServo.write(0); // look right
Serial.println("Servo position: 0 degrees");
delay(1000); // Wait for 1 second
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
float durationRight_us = pulseIn(echoPin, HIGH);
float distanceRight_cm = 0.017 * durationRight_us;
if (distanceRight_cm >= minDist_side) {
// Turn right if the path is clear on the right
turnRight();
delay(750); // Turn right for 1 second
stop();
delay(1000); // Stop for 1 second
// Move servo to 90 degrees
myServo.write(90); // center
Serial.println("Servo position: 90 degrees");
delay(1000); // Wait for 1 second
moveForward();
return; // Exit the loop
} else {
// Check the left direction
myServo.write(180); // look left
Serial.println("Servo position: 180 degrees");
delay(1000); // Wait for 1 second
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
float durationLeft_us = pulseIn(echoPin, HIGH);
float distanceLeft_cm = 0.017 * durationLeft_us;
if (distanceLeft_cm >= minDist_side) {
// Turn left if the path is clear on the left
turnLeft();
delay(750); // Turn left for 1 second
stop();
delay(1000); // Stop for 1 second
// Move servo to 90 degrees
myServo.write(90); // center
Serial.println("Servo position: 90 degrees");
delay(1000); // Wait for 1 second
moveForward();
return; // Exit the loop
}
}
// If both right and left are blocked, move backward
Serial.println("Obstacles detected in all directions. Moving backward.");
moveBackward();
delay(500); // Move backward for 1 second
stop();
delay(1000); // Stop for 1 second
// After moving backward, recheck the right and left directions in a loop
}
} else {
// If distance is greater than or equal to the threshold, turn off the LED
digitalWrite(ledPin, LOW);
moveForward();
}
}
// Function to move forward
void moveForward() {
Serial.println("Moving Forward");
digitalWrite(MOTOR_IN1, HIGH);
digitalWrite(MOTOR_IN2, LOW);
digitalWrite(MOTOR_IN3, LOW);
digitalWrite(MOTOR_IN4, HIGH);
}
// Function to stop
void stop() {
Serial.println("Stopping");
digitalWrite(MOTOR_IN1, LOW);
digitalWrite(MOTOR_IN2, LOW);
digitalWrite(MOTOR_IN3, LOW);
digitalWrite(MOTOR_IN4, LOW);
}
// Function to turn right
void turnRight() {
Serial.println("Turning Right");
digitalWrite(MOTOR_IN1, LOW);
digitalWrite(MOTOR_IN2, HIGH);
digitalWrite(MOTOR_IN3, LOW);
digitalWrite(MOTOR_IN4, HIGH);
}
// Function to turn left
void turnLeft() {
Serial.println("Turning Left");
digitalWrite(MOTOR_IN1, HIGH);
digitalWrite(MOTOR_IN2, LOW);
digitalWrite(MOTOR_IN3, HIGH);
digitalWrite(MOTOR_IN4, LOW);
}
// Function to move backward
void moveBackward() {
Serial.println("Moving Backward");
digitalWrite(MOTOR_IN1, LOW);
digitalWrite(MOTOR_IN2, HIGH);
digitalWrite(MOTOR_IN3, HIGH);
digitalWrite(MOTOR_IN4, LOW);
}
// Function to read the encoder state and update the count
void readEncoder() {
int newEncoderPinAState = digitalRead(ENCODER_A);
int newEncoderPinBState = digitalRead(ENCODER_B);
if (newEncoderPinAState != encoderPinAState) {
if (newEncoderPinAState == encoderPinBState) {
encoderCountLeft--;
} else {
encoderCountLeft++;
}
encoderPinAState = newEncoderPinAState;
}
if (newEncoderPinBState != encoderPinBState) {
if (newEncoderPinAState != encoderPinBState) {
encoderCountRight--;
} else {
encoderCountRight++;
}
encoderPinBState = newEncoderPinBState;
}
}
// Function to adjust motor speed based on encoder counts
void adjustMotorSpeed() {
// Calculate the difference between left and right encoder counts
long error = encoderCountLeft - encoderCountRight;
// Adjust motor speeds proportionally to the error
int leftMotorSpeed = constrain(255 - Kp * error, 0, 255);
int rightMotorSpeed = constrain(255 + Kp * error, 0, 255);
// Set motor speeds
analogWrite(MOTOR_IN1, leftMotorSpeed);
analogWrite(MOTOR_IN2, 0);
analogWrite(MOTOR_IN3, 0);
analogWrite(MOTOR_IN4, rightMotorSpeed);
}
