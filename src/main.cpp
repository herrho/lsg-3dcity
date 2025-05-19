/*
 * DC Motor Control with Arduino Uno
 * 
 * This sketch controls two DC motors using L298N driver,
 * displays information on a 1602 LCD with I2C interface,
 * and uses three LEDs for status indication.
 * 
 * Components:
 * - Arduino Uno
 * - L298N Motor Driver
 * - Two DC motors (2-wire)
 * - 1602 LCD with I2C interface
 * - Three LEDs (red, yellow, green)
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MotorControl.h"

// Define motor control pins for L298N
// Motor 1 pins
const int motor1Pin1 = 8;    // IN1
const int motor1Pin2 = 9;    // IN2
const int motor1Enable = 10; // ENA

// Motor 2 pins
const int motor2Pin1 = 6;    // IN3
const int motor2Pin2 = 7;    // IN4
const int motor2Enable = 11; // ENB

// Create motor control objects
MotorControl motor1(motor1Pin1, motor1Pin2, motor1Enable);
MotorControl motor2(motor2Pin1, motor2Pin2, motor2Enable);

// Initialize the LCD with I2C address (may need to be changed for your specific LCD)
// Common addresses are 0x27 and 0x3F
LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// LED pins
const int redLedPin = 13;     // Running LED
const int yellowLedPin = 12;  // Half-time warning LED
const int greenLedPin = 2;    // Completed LED

// Operation parameters
const int motorSpeed = 100;    // Speed (0-255 for PWM)
const long directionTime = 5000; // Time to run in each direction (5 seconds)
const long waitTime = 15000; // Time to wait before starting (2 seconds)

// Timer variables
unsigned long lastDirectionChange;
bool isForwardDirection = true;
bool isRunning = false;
bool isWaiting = false; // New flag to track waiting phase

// Function declarations
void scanI2CDevices();
void runMotors(bool forward);
void startMotors();
void changeDirection();
void stopMotors();
void startWaiting();

void setup() {
  // Set LED pins as outputs
  pinMode(redLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  
  // Initialize motor controllers
  motor1.init();
  motor2.init();
  
  // Initialize serial communication first for debugging
  Serial.begin(9600);
  Serial.println("DC Motor Control System Ready");
  Serial.println("Scanning for I2C devices...");
  
  // Initialize I2C
  Wire.begin();
  scanI2CDevices();
  
  // Initialize the LCD with a more robust approach
  delay(100);  // Give the LCD time to power up
  lcd.init();
  delay(100);  // Short delay after init
  lcd.backlight();
  delay(50);
  
  // Clear the display and reset cursor
  lcd.clear();
  delay(50);
  
  // Display startup message
  lcd.setCursor(0, 0);
  lcd.print("DC Motor Control");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  
  // Set initial LED state - green on (ready), others off
  digitalWrite(redLedPin, LOW);
  digitalWrite(yellowLedPin, LOW);
  digitalWrite(greenLedPin, HIGH);
  
  // Start the motors after a 2-second delay
  delay(2000);
  startMotors();
}

void loop() {
  if (isRunning) {
    // Check elapsed time
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastDirectionChange;

    if (isWaiting) {
      // Display waiting status on LCD - more carefully formatted
      lcd.clear();  // Clear display first to avoid artifacts
      delay(5);     // Short delay after clear
      
      lcd.setCursor(0, 0);
      lcd.print("Einsteigen bitte");  // Shortened to avoid overflow
      
      lcd.setCursor(0, 1);
      lcd.print("Abfahrt: ");
      int countdown = (waitTime - elapsedTime) / 1000;
      lcd.print(countdown);
      lcd.print("s   ");  // Extra spaces to clear any remaining characters
      
      // LED control - green until last 5 seconds, then yellow
      if ((waitTime - elapsedTime) <= 5000) {
        // Last 5 seconds of waiting - yellow LED on
        digitalWrite(redLedPin, LOW);
        digitalWrite(yellowLedPin, HIGH);
        digitalWrite(greenLedPin, LOW);
      } else {
        // Normal waiting - green LED on
        digitalWrite(redLedPin, LOW);
        digitalWrite(yellowLedPin, LOW);
        digitalWrite(greenLedPin, HIGH);
      }
      
      // Check if wait time has passed
      if (elapsedTime >= waitTime) {
        // Resume motors in the new direction
        isWaiting = false;
        lastDirectionChange = millis(); // Reset timer
        runMotors(isForwardDirection);
        
        // Motors running - red LED on
        digitalWrite(redLedPin, HIGH);
        digitalWrite(yellowLedPin, LOW);
        digitalWrite(greenLedPin, LOW);
        
        Serial.println("Abfahrt");
      }
    } else {
      // Only update the display every 500ms to prevent flickering
      static unsigned long lastDisplayUpdate = 0;
      if (currentTime - lastDisplayUpdate >= 500) {
        lastDisplayUpdate = currentTime;
        
        lcd.clear();
        delay(5);
        
        // Display running status on LCD
        lcd.setCursor(0, 0);
        if (isForwardDirection) {
          lcd.print("Richtung: rechts");
        } else {
          lcd.print("Richtung: links");
        }
        
        lcd.setCursor(0, 1);
        lcd.print("Abfahrt: ");
        int countdown = (directionTime - elapsedTime) / 1000;
        lcd.print(countdown);
        lcd.print("s   ");
      }
      
      // Motors running - red LED on
      digitalWrite(redLedPin, HIGH);
      digitalWrite(yellowLedPin, LOW);
      digitalWrite(greenLedPin, LOW);
      
      // Check if direction change time has passed
      if (elapsedTime >= directionTime) {
        changeDirection();
      }
    }
  }
}

void runMotors(bool forward) {
  if (forward) {
    // Run both motors in forward direction
    motor1.forward(motorSpeed);
    motor2.forward(motorSpeed);
    Serial.println("Motors running forward");
  } else {
    // Run both motors in reverse direction
    motor1.reverse(motorSpeed);
    motor2.reverse(motorSpeed);
    Serial.println("Motors running reverse");
  }
}

void startMotors() {
  // Start timer
  lastDirectionChange = millis();
  isForwardDirection = true;
  isRunning = true;
  isWaiting = false;
  
  // Update LEDs - red on (running), others off
  digitalWrite(redLedPin, HIGH);
  digitalWrite(yellowLedPin, LOW);
  digitalWrite(greenLedPin, LOW);
  
  // Run motors in initial (forward) direction
  runMotors(isForwardDirection);
  
  Serial.println("Motors started in forward direction");
}

void changeDirection() {
  // Toggle direction
  isForwardDirection = !isForwardDirection;
  
  // First stop the motors and start waiting
  startWaiting();
  
  Serial.print("Direction changed to: ");
  Serial.println(isForwardDirection ? "forward" : "reverse");
}

void startWaiting() {
  // Stop the motors during the waiting period
  motor1.stop();
  motor2.stop();
  
  // Set waiting flag and update timer
  isWaiting = true;
  lastDirectionChange = millis();
  
  // Update green LED to indicate waiting (initial waiting phase)
  digitalWrite(redLedPin, LOW);
  digitalWrite(yellowLedPin, LOW);
  digitalWrite(greenLedPin, HIGH);
  
  Serial.println("Motors paused, waiting before changing direction");
}

void stopMotors() {
  isRunning = false;
  
  // Stop both motors
  motor1.stop();
  motor2.stop();
  
  // Update LEDs - green on (completed)
  digitalWrite(redLedPin, LOW);
  digitalWrite(yellowLedPin, LOW);
  digitalWrite(greenLedPin, HIGH);
  
  // Update LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Operation");
  lcd.setCursor(0, 1);
  lcd.print("Completed");
  
  Serial.println("Motors stopped");
}

void scanI2CDevices() {
  byte error, address;
  int deviceCount = 0;
  
  Serial.println("Scanning for I2C devices...");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println();
      
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" I2C device(s)");
  }
}
