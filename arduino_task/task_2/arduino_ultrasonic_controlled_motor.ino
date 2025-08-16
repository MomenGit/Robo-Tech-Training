#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize LCD (address typically 0x27 or 0x3F)
LiquidCrystal_I2C lcd(32, 16, 2);

// Pin definitions
const int switchPin = 7;
const int motorPin1 = 5;
const int motorPin2 = 6;
const int enablePin = 3;
const int potPin = A0;
const int trigPin = 9;
const int echoPin = 10;

// Variables
int systemState = 0; // 0=off, 1=on
int motorSpeed = 0;
int targetSpeed = 0;
int lastDistance = 0;
const int thresholdDistance = 15; // 15cm

void setup() {
  // Initialize components
  pinMode(switchPin, INPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.print("System Off");
  
  // Ensure motor is off initially
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  analogWrite(enablePin, 0);
  
  Serial.begin(9600);
}

void loop() {
  // Check system switch
  if (digitalRead(switchPin) == HIGH && systemState == 0) {
    // Turn system on
    systemState = 1;
    lcd.clear();
    lcd.print("System On");
    delay(1000);
  } else if (digitalRead(switchPin) == LOW && systemState == 1) {
    // Turn system off
    systemState = 0;
    motorSpeed = 0;
    targetSpeed = 0;
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, 0);
    lcd.clear();
    lcd.print("System Off");
    delay(1000);
    return;
  }
  
  if (systemState == 1) {
    // Read potentiometer for target speed
    targetSpeed = map(analogRead(potPin), 0, 1023, 0, 255);
    
    // Read ultrasonic sensor
    long duration, distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;
    
    Serial.print("duration: ");
    Serial.println(duration);
    Serial.print("distance: ");
    Serial.println(distance);
    
    // Adjust target speed based on distance
    if (distance <= thresholdDistance && distance > 0) {
      // Object detected within range - reduce speed proportionally
      //targetSpeed = map(distance, 0, thresholdDistance, 0, targetSpeed);
      targetSpeed = 0;
    }
    
    // Smooth motor speed changes
    if (motorSpeed < targetSpeed) {
      motorSpeed++;
    } else if (motorSpeed > targetSpeed) {
      motorSpeed--;
    }
    
    // Apply motor speed
    if (motorSpeed > 0) {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      analogWrite(enablePin, motorSpeed);
    } else {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      analogWrite(enablePin, 0);
    }
    
    // Update LCD display
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print("cm");
    
    lcd.setCursor(0, 1);
    lcd.print("Speed: ");
    lcd.print(map(motorSpeed, 0, 255, 0, 100));
    lcd.print("%");
    
    delay(100); // Small delay for stability
  }
}