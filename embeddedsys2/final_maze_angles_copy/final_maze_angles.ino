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

const int compassAddress = 0x60;     // CMPS14 I2C address
unsigned long lastCompassUpdate = 0;
const unsigned long compassInterval = 1000;

volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;

// Define variables for distance control
int distanceToMove = 0;
bool moveForward = true;
int lidar_stop_value = 15;

bool rotate = false;

LIDARLite_v4LED myLidarLite;

// Create an instance of the TCS34725 sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


void leftMotorInterrupt() {
  leftPulses++;
}

void rightMotorInterrupt() {
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
    int offset = 0;
    bearing += offset;

    // Ensure the bearing is within the 0-360 degrees range
    bearing = (bearing + 360) % 360;

    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print(String(bearing) + " degrees " + getDirection(bearing));
  }
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
  }
}

unsigned long previousMillis = 0;
const long interval_color = 300;  // Check color every N milis
int motorSpeed = 100;

bool mazeTaskRun = false;

String recognizeColor(uint16_t red, uint16_t green, uint16_t blue, uint16_t clearVal) {
//  Serial.println("---");
//  Serial.println(clearVal + red + green + blue);
//
//  Serial.println("---");
//  Serial.println("Clear: " + String(clearVal));
//  Serial.println("Red: " + String(red));
//  Serial.println("Green: " + String(green));
//  Serial.println("Blue: " + String(blue));
  
  lcd.setCursor(0, 0);
  lcd.print("                   ");
  lcd.setCursor(0, 0);

  String color;

  if (clearVal > 900) {
    lcd.println("White");
    return "White";
  } else if (clearVal < 500) {
    lcd.println("Black");
    return "Black";
  }

  // Determine which color has the highest value
  else if (clearVal < 1070 && red > 165 && green < 300) {
    lcd.print("Red");
    color = "Red";
    if (mazeTaskRun) {
      redWall();
    }
  } else if (green > 190 && blue < 340) {
    lcd.print("Green");
    color = "Green";
    greenWall();
  } else if (blue > red && blue > green) {
    lcd.print("Blue");
    color = "Blue";
    blueWall();
  } else {
    lcd.print("Unknown");
    color = "Unknown";
  }

  Serial.println(color);

  return color;
}

String detectedColor = "Unknown";
unsigned long lastMoveTime = 0;
const unsigned long moveInterval = 200;

void loop() {
  unsigned long currentMillis = millis();

  // Check color every 'interval' milliseconds
  if (currentMillis - previousMillis >= interval_color) {
    previousMillis = currentMillis;

    // Read the RGB color values from the sensor
    uint16_t c, r, g, b;
    tcs.getRawData(&c, &r, &g, &b);

    // Determine the color based on the RGB values
    detectedColor = recognizeColor(c, r, g, b);

    // Print the detected color to the serial monitor
//    Serial.println(detectedColor);
  }
  
  if (currentMillis - lastCompassUpdate >= compassInterval) {
    directionsCompass();
    lastCompassUpdate = currentMillis;
  } 


  String receivedData = readSerial2Line();

  if(receivedData != ""){
     processMessage(receivedData);
  }

  if (mazeTaskRun){
    moveInMaze();
  }

//   unsigned long currentTime = millis();
//  if (mazeTaskRun && (currentTime - lastMoveTime >= moveInterval)) {
//    // Update the last move time
//    lastMoveTime = currentTime;
//    // Run moveInMaze() in a separate thread
//    moveInMaze();
//  }

//  if (receivedData.length() > 2) {
//    handleJoystickData(receivedData);
//  }

  // Lidar
//  uint16_t distance = getLidarDistance();
////  Serial.println("Distance: " + String(distance) + " cm");
//  lcd.setCursor(0, 2);
//  lcd.print("                   ");
//  lcd.setCursor(0, 2);
//  lcd.print("Dist. lidar: ");
//  lcd.print(distance);
}

void stopCar() {
  // Stop the motors
  digitalWrite(Motor_L_pwm_pin, 0);
  digitalWrite(Motor_R_pwm_pin, 0);
}
int initialCompass = 192;

//// Predefine compass value for the back panel of the maze.
//int compassDirBackPnl = 150;
int rotationEvent = 0; // every 90deg/10deg=9 events we finish checking a side.
bool rotateSide = false; // False - left, true - right.
void moveInMaze(){
  if (mazeTaskRun) {
      uint16_t distance = getLidarDistance(); // Get lidar reading
      Serial.println("Maze Runner");
      Serial.println(detectedColor);
      if(rotationEvent >= 3){ // 90 deg already rotated -> check the opposite 90deg.
        stopCar();
        rotationEvent = 0;
        if(rotateSide){
          Serial.println("Switch to left angles.");
          rotateSide = false;
          // Rotate the car to face the initial angle.
//          int currentBearing = getCurrentBearing();
//          int bearingDifference = abs(currentBearing - 110);
          rotateTo(initialCompass);
          handleDistanceCommand("-7");
        } else {
          Serial.println("Switch to right angles.");
          rotateSide = true;
          // Rotate the car to face the initial angle.
//          int currentBearing = getCurrentBearing();
//          int bearingDifference = abs(currentBearing + 110);
          rotateTo(initialCompass);
          handleDistanceCommand("-7");
        }
      }
      if (detectedColor == "Black"){
        if (distance <= 24) { // obstacle within 24 cm.
          handleDistanceCommand("-10");
          int currentBearing = getCurrentBearing();
          int bearingDifference = 0;
          if (!rotateSide){
            bearingDifference = abs(currentBearing - 30);
          } else {
            bearingDifference = abs(currentBearing + 30);
          }
           rotateTo(bearingDifference);
           rotationEvent++;
        }
    
        digitalWrite(Motor_L_dir_pin, Motor_forward);
        digitalWrite(Motor_R_dir_pin, Motor_return);
    
        analogWrite(Motor_L_pwm_pin, motorSpeed);
        analogWrite(Motor_R_pwm_pin, motorSpeed);
      } else if (detectedColor == "Red") {
//        stopCar(); // Stop the car
//        handleDistanceCommand("-10");
//        int currentBearing = getCurrentBearing();
//        int bearingDifference = 0;
//        if (!rotateSide){
//          bearingDifference = abs(currentBearing - 15);
//        } else {
//          bearingDifference = abs(currentBearing + 15);
//        }
//        rotateTo(bearingDifference);
//        rotationEvent++;
      }  else if (detectedColor == "Blue") {
      motorSpeed = 80; // Decrease car speed to 25%
      motorSpeed = 80;
      motorSpeed = 80;
      } else if (detectedColor == "Green") {
      motorSpeed = 120; // Decrease car speed to 75%
      motorSpeed = 120;
      motorSpeed = 120;
      } else {
        // Unknown or white color - stop moving
        stopCar();
    }
  }
}

void redWall(){
  stopCar(); // Stop the car
  handleDistanceCommand("-10");
  int currentBearing = getCurrentBearing();
  int bearingDifference = 0;
  if (!rotateSide){
     bearingDifference = abs(currentBearing - 30);
  } else {
    bearingDifference = abs(currentBearing + 30);
  }
  rotateTo(bearingDifference);
  handleDistanceCommand("-5");
  rotationEvent++;
}

void blueWall(){
  motorSpeed = 80; // Decrease car speed to 25%
}

void greenWall(){
  motorSpeed = 120; // Decrease car speed to 75%
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

    int offset = 0;
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
    rotationSpeed = 120;

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

  } while (abs(angleDifference) > 2); // Adjust the threshold as needed

  digitalWrite(Motor_L_pwm_pin, 0);
  digitalWrite(Motor_R_pwm_pin, 0);
}


void moveCar(int distance) { // distance in cm.
  // Calculate the time required to move the specified distance (adjust as needed)
  unsigned long moveTime = distance * 130; // 100 milliseconds per centimeter, adjust this value based on your motor and robot configuration

  // Set the motor speeds
  int motorSpeed = 120; // Adjust the speed as needed

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

  // Allow the robot to move for the specified time
  delay(moveTime);

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
//  Serial.println(distanceToMove);
  moveCar(distanceToMove);
}

String confPulses() {
  int initialPulseCount = leftPulses;
  
  // Move forward 8 cm.
  handleDistanceCommand("8");
  int forwardPulses = leftPulses - initialPulseCount;
  Serial.println(leftPulses);
  Serial.println(initialPulseCount);
  // Move backwards 8 cm.
  handleDistanceCommand("-8");
  
  float avgPulsesPerCm = forwardPulses / 5.0; // 16 cm total travel (8 cm forward + 8 cm backward)
  
  String result = String(avgPulsesPerCm, 2); // 2 decimal places
  
  result += " pulses/cm";
  Serial.println(result);
  return result;
}
void processMessage(String message) {
  if (message.length() > 0){
    if (message.startsWith("LIDAR distance received:")) {
    //Extract distance from the message
    int index = message.indexOf(":") + 2;
    String distance = message.substring(index);
    //Serial.println("LIDAR distance received: " + distance);
    
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
    } else if (message.startsWith("GET_PULSES")) {
      // Get pulses avg.
      String output = confPulses();
      Serial2.println(output);
      lcd.setCursor(0, 3);
      lcd.print("                   ");
      lcd.setCursor(0, 3);
      lcd.print(output);
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
    } else if (message.startsWith("START")) {
      Serial.println("AAA");
      mazeTaskRun = true;
      rotateTo(initialCompass);
      
//      moveInMaze();
    } else if (message.startsWith("STOP")) {
      Serial.println("BBB");
      mazeTaskRun = false;
      stopCar();
    } else {
      // Unknown message
      Serial.println("Unknown message: " + message);
      // You can handle unknown messages here if needed
    }
  }
}
