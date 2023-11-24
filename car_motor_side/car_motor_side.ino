#include <LiquidCrystal.h>
#include <Wire.h>

#define Motor_forward         0
#define Motor_return          1
#define Motor_L_dir_pin       7
#define Motor_R_dir_pin       8
#define Motor_L_pwm_pin       9
#define Motor_R_pwm_pin       10

LiquidCrystal lcd(25, 27, 5, 4, 11, 12); // (RS, E, D4, D5, D6, D7)
int buttonPin = 15; // Button connected to digital pin 15
int buttonState = LOW;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int buttonPressCount = 0;

const int compassAddress = 0x60;     // CMPS14 I2C address
unsigned long lastCompassUpdate = 0;
const unsigned long compassInterval = 1000;

volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;
float wheelDiameter = 6.2; // Example wheel diameter in cm

void leftMotorInterrupt() {
  Serial.println("lefttt");
  leftPulses++;
}

void rightMotorInterrupt() {
  Serial.println("righttt");
  rightPulses++;
}

void setup() {
  lcd.begin(20, 4); // Change these values according to your LCD configuration

  pinMode(Motor_L_dir_pin, OUTPUT);
  pinMode(Motor_R_dir_pin, OUTPUT);
  pinMode(Motor_L_pwm_pin, OUTPUT);
  pinMode(Motor_R_pwm_pin, OUTPUT);

  pinMode(2, INPUT);
  pinMode(3, INPUT);

  attachInterrupt(digitalPinToInterrupt(Motor_L_pwm_pin), leftMotorInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(Motor_R_pwm_pin), rightMotorInterrupt, RISING);

  pinMode(buttonPin, INPUT_PULLUP);

  Wire.begin();

  Serial.begin(115200);
  Serial1.begin(115200);
}

String getDirection(int bearing) {
  String directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  int index = round(bearing / 45.0) % 8;
  return directions[index];
}

void directionsCompass() {
  Wire.beginTransmission(compassAddress);
  Wire.write(1);
  Wire.endTransmission(false);

  Wire.requestFrom(compassAddress, 1, true);
  if (Wire.available() >= 1) {
    byte raw = Wire.read();
    int bearing = map(raw, 0, 255, 0, 360);
    
    // Apply the offset
    int offset = 35;
    bearing += offset;

    // Ensure the bearing is within the 0-360 degrees range
    bearing = (bearing + 360) % 360;

    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print(String(bearing) + " degrees " + getDirection(bearing));
  }
}

void handleButtonPress() {
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        Serial.println("pressing");
        buttonPressCount++;
      }
    }
  }

  lastButtonState = reading;

  lcd.setCursor(0, 0);
  lcd.print("Button Presses: ");
  lcd.print(buttonPressCount);
}

void handleJoystickData(String data) {
  int xPercentage, yPercentage;
  if (sscanf(data.c_str(), "Received data: X:%d,Y:%d", &xPercentage, &yPercentage) == 2) {
    int pwm_R = 0;
    int pwm_L = 0;

    pwm_L = map(yPercentage, -100, 100, -255, 255);
    pwm_R = map(yPercentage, -100, 100, -255, 255);

    pwm_L += xPercentage;
    pwm_R -= xPercentage;

    pwm_L = constrain(pwm_L, -255, 255);
    pwm_R = constrain(pwm_R, -255, 255);

    digitalWrite(Motor_L_dir_pin, (pwm_L >= 0) ? Motor_forward : Motor_return);
    digitalWrite(Motor_R_dir_pin, (pwm_R >= 0) ? Motor_forward : Motor_return);

    analogWrite(Motor_L_pwm_pin, abs(pwm_L));
    analogWrite(Motor_R_pwm_pin, abs(pwm_R));

//    Serial.print("L:");
//    Serial.print(abs(pwm_L));
//    Serial.print(",R:");
//    Serial.println(abs(pwm_R));
  }
}

void printDistance() {
  float leftDistance = (leftPulses * 3.14159265358979323846 * wheelDiameter) / 360.0;
  float rightDistance = (rightPulses * 3.14159265358979323846 * wheelDiameter) / 360.0;
  float totalDistance = (leftDistance + rightDistance) / 2.0;

  lcd.setCursor(0, 2);
  lcd.print("L: ");
  lcd.print(leftPulses);
  lcd.print(" R: ");
  lcd.print(rightPulses);

  lcd.setCursor(0, 3);
  lcd.print("Distance: ");
  lcd.print(totalDistance, 2);
  lcd.print(" cm");

  Serial.print("L:");
  Serial.print(leftPulses);
  Serial.print("; R:");
  Serial.print(rightPulses);
  Serial.print("; Distance: ");
  Serial.print(totalDistance, 2);
  Serial.println(" cm");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastCompassUpdate >= compassInterval) {
    directionsCompass();
    lastCompassUpdate = currentMillis;
  } 

  String receivedData = Serial1.readStringUntil('\n');
  Serial.println(receivedData);

  handleButtonPress();

  if (receivedData.length() > 2) {
    handleJoystickData(receivedData);
  }

  printDistance();
}
