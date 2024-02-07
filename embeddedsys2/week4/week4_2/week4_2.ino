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

LIDARLite_v4LED myLidarLite;

// Create an instance of the TCS34725 sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


void leftMotorInterrupt() {
//  Serial.println("lefttt");
//  Serial1.println("lefttt");
  leftPulses++;
}

void rightMotorInterrupt() {
//  Serial.println("righttt");
//  Serial1.println("righttt");
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

  Serial.begin(115200);
  Serial1.begin(115200);

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

String recognizeColor(uint16_t red, uint16_t green, uint16_t blue) {
//  Serial.println(red);
//  Serial.println(green);
//  Serial.println(blue);
  lcd.setCursor(0, 0);
  lcd.print("                   ");
  lcd.setCursor(0, 0);

  if(red > green && red > blue){
   lcd.print ("red");
   return "red";
  }
  if(green > red && green > blue){
    lcd.print ("green");
    return "green";
  }
  if(blue > red && blue > green){
   lcd.print ("blue");
   return "blue";
  }
}


void loop() {
  unsigned long currentMillis = millis();

  // Check color every 'interval' milliseconds
  if (currentMillis - previousMillis >= interval_color) {
    previousMillis = currentMillis;

    // Read the RGB color values from the sensor
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    // Determine the color based on the RGB values
    String detectedColor = recognizeColor(r, g, b);

    // Print the detected color to the serial monitor
//    Serial.println(detectedColor);
  }
  
  if (currentMillis - lastCompassUpdate >= compassInterval) {
    directionsCompass();
    lastCompassUpdate = currentMillis;
  } 


  String receivedData = readSerial1Line();
  Serial.println(receivedData);

//  handleButtonPress();

//  if (receivedData.length() > 2) {
//    handleJoystickData(receivedData);
//  }

  printDistance();
  handleRotateCommand(receivedData);

  if(receivedData.indexOf("Move command received: move:") != -1){
    int startIndex = receivedData.indexOf("move:") + 5; // 5 is the length of "move:"
    int endIndex = receivedData.indexOf("cm");

    String extractedText = receivedData.substring(startIndex, endIndex);

    serialCommunication(extractedText);
    
      if (distanceToMove > 0) {
        moveCar(distanceToMove);
        distanceToMove = 0; // Reset the distance
      }
  }

  // Lidar
  uint16_t distance = getLidarDistance();
  Serial.println("Distance: " + String(distance) + " cm");
  lcd.setCursor(0, 2);
  lcd.print("                   ");
  lcd.setCursor(0, 2);
  lcd.print("Dist. lidar: ");
  lcd.print(distance);
  
  // Testing lidar, remove me later!
  moveLidarTest((int) distance);
}

String readSerial1Line() {
  static String inputString = "";
  while (Serial1.available()) {
    char c = Serial1.read();
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

int getPotentiometerValue(){
  // Read the analog value from the potentiometer
  int analogValue = analogRead(A15);

  // Map the analog value to the desired range [0, 30]
  int mappedValue = map(analogValue, 0, 1023, 20, 40);
  return mappedValue;
}

void moveLidarTest(int distance){
  lcd.setCursor(0, 3);
  lcd.print("                ");
  lcd.setCursor(0, 3);
  lcd.print("Dist. trav.: ");
  lcd.print(totalDistance);

  //Serial.println(getPotentiometerValue());
  if (distance > (lidar_stop_value - 3) && distance < (lidar_stop_value + 3)) {
    // rotate left.
    int currentBearing = getCurrentBearing();
    int leftSide = (currentBearing - 87) % 360;
    rotateTo(leftSide);
  } else if (distance < lidar_stop_value) {
    moveForward = true;
    moveCar(lidar_stop_value - distance); // move backwards
  } else if(distance > lidar_stop_value){
    moveForward = false;
    moveCar(distance - lidar_stop_value); // move forward
  } 
}

//void checkRotation() {
//  // Get the current compass bearing
//  int currentBearing = getCurrentBearing();
//
//  // Check left side lidar reading
//  int leftSide = (currentBearing + 90) % 360;
//  rotateTo(leftSide);
//  uint16_t leftDistance = getLidarDistance(); // Get left lidar reading
//  Serial.println("Left distance: " + String(leftDistance) + " cm");
//  
//  // Check right side lidar reading
//  int rightSide = (currentBearing - 90) % 360;
//  rotateTo(rightSide);
//  uint16_t rightDistance = getLidarDistance(); // Get right lidar reading
//  Serial.println("Right distance: " + String(rightDistance) + " cm");
//
//  // Rotate the car to the side with the longest distance
//  if (leftDistance > rightDistance) {
//    if (currentBearing != leftSide) {
//      Serial.println("Rotated to the left side.");
//      rotateTo(leftSide);
//      Serial.println("Rotated to the left side.");
//    } else {
//      Serial.println("Already on the left side.");
//    }
//  } else {
//    if (currentBearing != rightSide) {
//      Serial.println("Rotateeeee to the right side.");
//      rotateTo(rightSide);
//      Serial.println("Rotated to the right side.");
//    } else {
//      Serial.println("Already on the right side.");
//    }
//  }
//}


void handleRotateCommand(String data) {
  int targetAngle_deg;
  Serial.println(data);
  if (sscanf(data.c_str(), "Rotate command received: rotate:%ddeg", &targetAngle_deg) == 1) {
    int currentBearing = getCurrentBearing();
    int targetBearing = (currentBearing + targetAngle_deg + 360) % 360;

    Serial.print("Rotate command received: ");
    Serial.print(targetAngle_deg);
    Serial.println(" degrees");

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

    angleDifference = (targetBearing - currentBearing + 360) % 360;
    if (angleDifference > 180) {
      angleDifference -= 360; // Adjust the angle difference to be in the range [-180, 180]
    }
    
    int rotationSpeed = map(abs(angleDifference), 0, 180, 255, 50); // Inverted speed mapping
    rotationSpeed = min(rotationSpeed, 150); // Cap maximum rotation speed

    if (angleDifference > 0 && angleDifference <= 180) {
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

     Serial.println("---");
    Serial.println(abs(angleDifference));

    delay(50);

  } while (abs(angleDifference) > 5); // Adjust the threshold as needed

  digitalWrite(Motor_L_pwm_pin, 0);
  digitalWrite(Motor_R_pwm_pin, 0);
}



void serialCommunication(String message) {
  
    Serial.print("Message received, content: ");
    Serial.println(message);

    // Check if the message contains the LCD command
    if (message.startsWith("LCD:")) {
      // Extract the text to be displayed on the LCD
      String lcdText = message.substring(4); // Remove "LCD:" from the message
      lcd.clear(); // Clear the LCD display
      lcd.setCursor(0, 0);
      lcd.print(lcdText); // Display the text on the LCD
    } else if ("Lidar stop:"){
      // Extract the text to be displayed on the LCD
      String value = message.substring(11); // Remove "LCD:" from the message
      if(lidar_stop_value < value.toInt()){ // backup if previous lidar stop value is smaller than now.
        moveForward = true; 
        moveCar(value.toInt() - lidar_stop_value);
      }
      lidar_stop_value = value.toInt();
      Serial.println("Got lidar stop dist command!");
    } else if (message.indexOf("Print:") != -1) {
      // Handle "Print" command as before
      int pos_s = message.indexOf("Print");

      if (pos_s > -1) {
        Serial.println("Command = Print ");
        pos_s = message.indexOf(":");

        if (pos_s > -1) {
          String stat = message.substring(pos_s + 1);
          if (stat == "Hi" || stat == "hi") {
            Serial.println("Hi!");
          } else if (stat == "Hello") {
            Serial.println("Hello there!");
          }
        }
      }
    } else if (message.startsWith("dist:")) {
      // Extract the distance value from the message
      String distanceStr = message.substring(5); // Remove "dist:" from the message
      distanceToMove = distanceStr.toInt(); // Convert the distance to an integer

      // Determine the direction (forward or backward)
      if (distanceToMove < 0) {
        moveForward = false;
        distanceToMove = -distanceToMove;
      } else {
        moveForward = true;
      }
    } else {
      Serial.println("No valid command found.");
    }
  
}

void moveCar(int distance) { // distance in cm.
  // Calculate the time required to move the specified distance (adjust as needed)
  unsigned long moveTime = distance * 50; // 100 milliseconds per centimeter, adjust this value based on your motor and robot configuration

  // Set the motor speeds
  int motorSpeed = 180; // Adjust the speed as needed

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
