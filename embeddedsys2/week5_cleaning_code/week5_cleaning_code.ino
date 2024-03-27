#include <LiquidCrystal.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>


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

// Define variables for distance control
int distanceToMove = 0;
bool moveForward = true;
int lidar_stop_value = 15;

bool rotate = false;

LIDARLite_v4LED myLidarLite;

// Create an instance of the TCS34725 sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


void leftMotorInterrupt() {
//  Serial.println("lefttt");
//  Serial2.println("lefttt");
  leftPulses++;
}

void rightMotorInterrupt() {
//  Serial.println("righttt");
//  Serial2.println("righttt");
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

  attachInterrupt(digitalPinToInterrupt(2), leftMotorInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(3), rightMotorInterrupt, RISING);

  pinMode(buttonPin, INPUT_PULLUP);

  Wire.begin();

  digitalWrite(SCL, LOW);
  digitalWrite(SDA, LOW);

  Serial.begin(9600);
  Serial2.begin(9600);

  // Lidar setup
  myLidarLite.configure(0);

  // Initialize the sensor
  if (!tcs.begin()) {
    Serial.println("Could not find a valid TCS34725 sensor, check wiring!");
    while (1);
  }
}

String getDirection(int bearing) {
  String directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  int index = round(bearing / 45.0) % 8;
  return directions[index];
}

int getLidarDistance() {
    // Trigger a single range measurement
  myLidarLite.takeRange();

  // Wait for the measurement to complete
  while (myLidarLite.getBusyFlag()) {
    delay(10);
  }

  // Read and print the distance
  uint16_t distance = myLidarLite.readDistance();
  return distance;
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
    int offset = 30;
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

//  lcd.setCursor(0, 0);
//  lcd.print("Button Presses: ");
//  lcd.print(buttonPressCount);
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

    digitalWrite(Motor_L_dir_pin, (pwm_L >= 0) ? Motor_return : Motor_forward);
    digitalWrite(Motor_R_dir_pin, (pwm_R >= 0) ? Motor_forward : Motor_return);

    analogWrite(Motor_L_pwm_pin, abs(pwm_L));
    analogWrite(Motor_R_pwm_pin, abs(pwm_R));

//    Serial.print("L:");
//    Serial.print(abs(pwm_L));
//    Serial.print(",R:");
//    Serial.println(abs(pwm_R));
  }
}

float totalDistance = 0;

void printDistance() {
  float leftDistance = (leftPulses * 3.1415 * wheelDiameter) / 360.0;
  float rightDistance = (rightPulses * 3.1415 * wheelDiameter) / 360.0;
  totalDistance = (leftDistance + rightDistance) / 2.0;

//  lcd.setCursor(0, 3);
//  lcd.print("Dist. motor: ");
//  lcd.print(totalDistance, 2);
//  lcd.print(" cm");

//  Serial.print("L:");
//  Serial.print(leftPulses);
//  Serial.print("; R:");
//  Serial.print(rightPulses);
//  Serial.print("; Distance: ");
//  Serial.print(totalDistance, 2);
//  Serial.println(" cm");
}

unsigned long previousMillis = 0;
const long interval_color = 1000;  // Check color every 1 second

String recognizeColor(uint16_t red, uint16_t green, uint16_t blue, uint16_t clearVal) {
  Serial.println("---");
  Serial.println(clearVal + red + green + blue);

   Serial.println("---");
  Serial.println("Clear: " + String(clearVal));
  Serial.println("Red: " + String(red));
  Serial.println("Green: " + String(green));
  Serial.println("Blue: " + String(blue));
  
 
  
  lcd.setCursor(0, 0);
  lcd.print("                   ");
  lcd.setCursor(0, 0);

  String color;

  if (clearVal > 2000) {
    lcd.println("White");
  }

  // Determine which color has the highest value
  else if (red > green && red > blue) {
    lcd.print("Red");
    color = "Red";
  } else if (green > red && green > blue) {
    lcd.print("Green");
    color = "Green";
  } else if (blue > red && blue > green) {
    lcd.print("Blue");
    color = "Blue";
  } else {
    lcd.print("Unknown");
    color = "Unknown";
  }

  return color;

//  if (sum >= 990 && sum <= 1100) {
//    lcd.print("red");
//    return "red";
//  } else if (sum >= 800 && sum <= 900){
//    lcd.print("green");
//    return "green";
//  } else if (sum >= 900 && sum <= 990){
//    lcd.print("blue");
//    return "blue";
//  } else {
//    lcd.print("unknown");
//    return "unknown";
//  }

//  // Check if the RGB values fall within the ranges for red
//  if ((red >= 230 && red <= 320) && (green >= 320 && green <= 440) && (blue >= 380 && blue <= 480)) {
//    lcd.print("red");
//    return "red";
//  }
//  
//  // Check if the RGB values fall within the ranges for green
//  if ((red >= 150 && red <= 200) && (green >= 270 && green <= 320) && (blue >= 330 && blue <= 370)) {
//    lcd.print("green");
//    return "green";
//  }
//  
//  // Check if the RGB values fall within the ranges for blue
//  if ((red >= 100 && red <= 120) && (green >= 90 && green <= 110) && (blue >= 325 && blue <= 345)) {
//    lcd.print("blue");
//    return "blue";
//  }
//  
//  // If none of the above conditions are met, the color is not recognized
//  lcd.print("unknown");
//  return "unknown";
}



void loop() {
  unsigned long currentMillis = millis();

  // Check color every 'interval' milliseconds
  if (currentMillis - previousMillis >= interval_color) {
    previousMillis = currentMillis;

    // Read the RGB color values from the sensor
    uint16_t c, r, g, b;
    tcs.getRawData(&c, &r, &g, &b);

    // Determine the color based on the RGB values
    String detectedColor = recognizeColor(c, r, g, b);

    // Print the detected color to the serial monitor
//    Serial.println(detectedColor);
  }
  
  if (currentMillis - lastCompassUpdate >= compassInterval) {
    directionsCompass();
    lastCompassUpdate = currentMillis;
  } 


  String receivedData = readSerial2Line();
  
//  Serial.println(receivedData);
  processMessage(receivedData);

//  handleButtonPress();

//  if (receivedData.length() > 2) {
//    handleJoystickData(receivedData);
//  }

  printDistance();
//  handleRotateCommand(receivedData);

//  if(receivedData.indexOf("Move distance received:") != -1){
//    int startIndex = receivedData.indexOf(":") + 1;
//    int endIndex = receivedData.indexOf("cm");
//
//    String extractedText = receivedData.substring(startIndex, endIndex);
//
//    serialCommunication(extractedText);
//    
//      if (distanceToMove > 0) {
//        moveCar(distanceToMove);
//        distanceToMove = 0; // Reset the distance
//      }
//  }

  // Lidar
  uint16_t distance = getLidarDistance();
//  Serial.println("Distance: " + String(distance) + " cm");
  lcd.setCursor(0, 2);
  lcd.print("                   ");
  lcd.setCursor(0, 2);
  lcd.print("Dist. lidar: ");
  lcd.print(distance);
  
  // Testing lidar, remove me later!
//  moveLidarTest((int) distance);
}

String readSerial2Line() {
  static String inputString = "";
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      String line = inputString;
      inputString = "";
      return line;
    } else {
      inputString += c;
    }
  }
  return ""; // Return an empty string if a complete line is not yet available
}

void handleRotateCommand(String data) {
  int targetAngle_deg;
//  Serial.println(data);
  if (sscanf(data.c_str(), "Rotate command received: rotate:%ddeg", &targetAngle_deg) == 1) {
    int currentBearing = getCurrentBearing();
    int targetBearing = (currentBearing + targetAngle_deg + 360) % 360;
//
//    Serial.print("Rotate command received: ");
//    Serial.print(targetAngle_deg);
//    Serial.println(" degrees");

    rotateTo(targetBearing);
  }
}

int getCurrentBearing() {
  Wire.beginTransmission(compassAddress);
  Wire.write(1);
  Wire.endTransmission(false);

  Wire.requestFrom(compassAddress, 1, true);
  if (Wire.available() >= 1) {
    byte raw = Wire.read();
    int bearing = map(raw, 0, 255, 0, 360);

    int offset = -35;
    bearing += offset;

    return (bearing + 360) % 360;
  }

  return 0;
}

void rotateTo(int targetBearing) {
  int currentBearing;
  int angleDifference;
  do {
    currentBearing = getCurrentBearing();

    // Calculate the shortest angle difference
    angleDifference = (targetBearing - currentBearing + 180) % 360 - 180;
    int rotationSpeed = map(abs(angleDifference), 0, 180, 50, 255);
    rotationSpeed = 150;

    if (angleDifference > 0) {
      // Rotate clockwise
      digitalWrite(Motor_R_dir_pin, Motor_forward);
      digitalWrite(Motor_L_dir_pin, Motor_forward);
    } else {
      // Rotate counterclockwise
      digitalWrite(Motor_R_dir_pin, Motor_return);
      digitalWrite(Motor_L_dir_pin, Motor_return);
    }

    analogWrite(Motor_L_pwm_pin, rotationSpeed);
    analogWrite(Motor_R_pwm_pin, rotationSpeed);

    delay(50);

  } while (abs(angleDifference) > 5); // Adjust the threshold as needed

  digitalWrite(Motor_L_pwm_pin, 0);
  digitalWrite(Motor_R_pwm_pin, 0);
}


void moveCar(int distance) { // distance in cm.
  // Set the motor speeds
  int motorSpeed = 160; // Adjust the speed as needed

  if (moveForward) {
    // Move forward
    digitalWrite(Motor_L_dir_pin, Motor_return);
    digitalWrite(Motor_R_dir_pin, Motor_forward);
  } else {
    // Move backward
    digitalWrite(Motor_L_dir_pin, Motor_forward);
    digitalWrite(Motor_R_dir_pin, Motor_return);
  }

  // Apply PWM to the motors
  analogWrite(Motor_L_pwm_pin, motorSpeed);
  analogWrite(Motor_R_pwm_pin, motorSpeed);

  // Variables to keep track of the distance moved
  uint16_t initialDistance = getLidarDistance();
  uint16_t movedDistance = 0;

  // Array to store recent lidar readings for calculating moving average
  const int numReadings = 5; // Number of readings to use for moving average
  uint16_t readings[numReadings];
  uint16_t total = 0;
  uint16_t average = 0;
  int index = 0;

  // Initialize readings array
  for (int i = 0; i < numReadings; i++) {
    readings[i] = initialDistance;
    total += readings[i];
  }

  // Continue moving until the specified distance is reached
  while (movedDistance < distance) {
    // Check lidar distance
    uint16_t lidarDistance = getLidarDistance();

    // Calculate moving average
    total = total - readings[index] + lidarDistance;
    readings[index] = lidarDistance;
    index = (index + 1) % numReadings;
    average = total / numReadings;

    // Calculate the distance moved since the start
    movedDistance = initialDistance - average;
    
    // Delay for a short time to avoid overwhelming the system
    delay(10);
  }

  // Stop the motors
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
}


void handleLCDCommand(String lcdText) {
  lcd.clear(); // Clear the LCD display
  lcd.setCursor(0, 0);
  lcd.print(lcdText); // Display the text on the LCD
}

void handleLidarStopCommand(String value) {
  uint16_t distance = getLidarDistance(); // Current lidar reading.
  Serial.println("Current lidar reading: " + distance);
  if (distance < value.toInt()) { // backup if previous lidar stop value is smaller than now.
    moveForward = true;
  }
  lidar_stop_value = value.toInt();
  Serial.println("Got lidar stop dist command!");
}

void handlePrintCommand(String stat) {
  if (stat == "Hi" || stat == "hi") {
    Serial.println("Hi!");
  } else if (stat == "Hello") {
    Serial.println("Hello there!");
  }
}

void handleDistanceCommand(String distanceStr) {
  distanceToMove = distanceStr.toInt(); // Convert the distance to an integer

  // Determine the direction (forward or backward)
  if (distanceToMove < 0) {
    moveForward = true;
    distanceToMove = -distanceToMove;
  } else {
    moveForward = false;
  }
  Serial.println(distanceToMove);
  moveCar(distanceToMove);
}

void processMessage(String message) {
  if (message.length() > 0){
    if (message.startsWith("LIDAR distance received:")) {
    // Extract distance from the message
    int index = message.indexOf(":") + 2;
    String distance = message.substring(index);
//    Serial.println("LIDAR distance received: " + distance);
    
    // Call the corresponding function to handle the message
    handleLidarStopCommand(distance);
    } else if (message.startsWith("Compass value received:")) {
      // Handle compass value
      Serial.println("Compass value received");
      int index = message.indexOf(":") + 2;
      String rotateDeg = message.substring(index);
      
      int currentBearing = getCurrentBearing();
      int bearingDifference = abs(currentBearing + rotateDeg.toInt());
      rotateTo(bearingDifference);
      
    } else if (message.startsWith("GET_SENSOR_DATA")) {
      // Get lidar value
    uint16_t distance = getLidarDistance();
    // Get compass value
    int currentBearing = getCurrentBearing();
    String output = "Lidar reading: " + String(distance) + ", Compass value: " + String(currentBearing);
    Serial2.println(output);
    } else if (message.startsWith("Move distance received:")) {
      // Extract move distance from the message
      int index = message.indexOf(":") + 2;
      String distance = message.substring(index);
      Serial.println("Move distance received: " + distance);
      // Call the corresponding function to handle the message
      handleDistanceCommand(distance);
    } else if (message.startsWith("Rotate command received:")) {
      // Extract data from the message
      String data = message.substring(message.indexOf(":") + 1);
      // Call the function to handle the rotate command
      handleRotateCommand(data);
    } else {
      // Unknown message
      Serial.println("Unknown message: " + message);
      // You can handle unknown messages here if needed
    }
  }
}
