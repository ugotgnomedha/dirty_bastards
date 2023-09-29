#include <LiquidCrystal.h>

LiquidCrystal lcd(25, 27, 5, 4, 3, 2); // (RS, E, D4, D5, D6, D7)
//LiquidCrystal lcd2(13, 12, 11, 10, 7, 6); // Second LCD (change pin numbers accordingly)

int buttonPin = 19; // Button connected to digital pin 19
int buttonState = LOW;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int buttonPressCount = 0;

// GROUP NAME RELATED <>
unsigned long previousMillisScroll = 0;
unsigned long previousMillisBlink = 0;

const long scrollInterval = 600;
const long blinkInterval = 600; // Blink interval for "Group Name:"

bool isGroupNameVisible = true; // Track if "Group Name:" is visible or not
// GROUP NAME RELATED </>

// LED SWITCHES <>
//const int potBrightnessPin = A4;  // Potentiometer for brightness control
//const int potDirectionPin = A5;   // Potentiometer for direction control
//const int led1Pin = 44;            // PWM-capable pin for LED 1
////const int led2Pin = 46;           // PWM-capable pin for LED 2
//bool ledStatus = true;
// LED SWITCHES </>

void setup() {
  // Initialize the LCD
  lcd.begin(20, 4); // Change these values according to your LCD configuration

  lcd.setCursor(0, 0);
  lcd.print("Joystick:");

  // GROUP NAME RELATED <>
  // Initialize the second LCD
  //lcd2.begin(16, 2); // Change these values according to your LCD configuration

  // Start the scrolling and blinking for the group name display
//  lcd2.setCursor(0, 0);
//  lcd2.print("Group Name:");

//  lcd2.setCursor(0, 1);
//  lcd2.print("                    "); // Clear the second line initially
  // GROUP NAME RELATED </>

  // Set up ADC for reading analog values
  analogReference(DEFAULT); // Use default reference voltage (usually 5V)

  // Initialize the button pin
  pinMode(buttonPin, INPUT_PULLUP);

// LED SWITCHES <>
//  pinMode(led1Pin, OUTPUT);
  //pinMode(led2Pin, OUTPUT);
// LED SWITCHES </>

  Serial.begin(9600);
}

// GROUP NAME RELATED <>
//void scrollGroupName() {
//  static int textPosition = 0;
//  int groupNameLength = strlen("Dirty Bastards ");
//  lcd2.setCursor(0, 1);
//  lcd2.print("                    "); // Clear the entire second row
//
//  for (int i = 0; i < 16; i++) {
//    int charIndex = (textPosition + i) % groupNameLength;
//    lcd2.setCursor(i, 1);
//    lcd2.print("Dirty Bastards "[charIndex]);
//  }
//
//  textPosition = (textPosition + 1) % groupNameLength;
//}

//void blinkGroupName() {
//  if (isGroupNameVisible) {
//    lcd2.setCursor(0, 0);
//    lcd2.print("                    "); // Clear the entire first row
//  } else {
//    lcd2.setCursor(0, 0);
//    lcd2.print("Group Name:");
//  }
//
//  isGroupNameVisible = !isGroupNameVisible;
//}
// GROUP NAME RELATED </>

// LED SWITCHES <>
//void ledSwitches(){
//  int brightnessValue = analogRead(potBrightnessPin);
//  int directionValue = analogRead(potDirectionPin);
//
//  int mappedBrightness = map(brightnessValue, 0, 1023, 0, 1023);
//
//  bool isDirectionPositive = directionValue > 512;
//
//  int led1Brightness, led2Brightness;
//
//  if (isDirectionPositive) {
//    led1Brightness = 1023 - mappedBrightness;
//    //led2Brightness = mappedBrightness;
//  } else {
//    led1Brightness = mappedBrightness;
//    //led2Brightness = 1023 - mappedBrightness;
//  }
//  
//  //analogWrite(led1Pin, map(led1Brightness, 0, 1023, 0, 255));
//  //analogWrite(led2Pin, map(led2Brightness, 0, 1023, 0, 255));
//}
// LED SWITCHES </>

void loop() {
  // LED SWITCHES <>
//  if(ledStatus == true){
//    ledSwitches();
//  }
  // LED SWITCHES </>
  
  // GROUP NAME RELATED <>
  // Check and update the scrolling text
  unsigned long currentMillis = millis();
//  if (currentMillis - previousMillisScroll >= scrollInterval) {
//    previousMillisScroll = currentMillis;
//    scrollGroupName();
//  }

  // Check and update the blinking text
//  if (currentMillis - previousMillisBlink >= blinkInterval) {
//    previousMillisBlink = currentMillis;
//    blinkGroupName();
//  }
  // GROUP NAME RELATED </>
  
  int xValue = analogRead(A8); // Connect X output of the joystick to A8
  int yValue = analogRead(A9); // Connect Y output of the joystick to A9

  // Calculate joystick percentage
  int xPercentage = map(xValue, 0, 1023, -100, 100);
  int yPercentage = map(yValue, 0, 1023, -100, 100);
  lcd.setCursor(12, 0);
  lcd.print(xPercentage);
  lcd.print("%  ");

  lcd.setCursor(0, 1);

  // Calculate the position of the joystick indicator
  int indicatorPosition = map(xPercentage, -100, 100, 0, 19);

  // Print the joystick indicator (a vertical bar) on the bottom row of the LCD
  lcd.print("-100%    0%     100%");
  
  // Clear the last row of the LCD
  lcd.setCursor(0, 2);
  lcd.print("                    "); // Clear the entire row
  lcd.setCursor(indicatorPosition, 2);
  lcd.write('|');

  // Read the button state
  int reading = digitalRead(buttonPin);

  // Check for a button press
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      // If the button is pressed, increment the button press count
      if (buttonState == LOW) {
        buttonPressCount++;

//        if(ledStatus == false){
//          ledStatus = true;
//        } else {
//          ledStatus = false;
//           // Turn off the LEDs when the button is pressed
//          //analogWrite(led1Pin, 0);
//          //analogWrite(led2Pin, 0);  
//        }
      }
    }
  }

  lastButtonState = reading;

  // Print button press count on the last row of the LCD
  lcd.setCursor(0, 3);
  lcd.print("Button Presses: ");
  lcd.print(buttonPressCount);

  // Print the voltages and joystick percentages to the Serial Monitor
  Serial.print(" V\tJoystick: ");
  Serial.print(xPercentage);

  delay(50);
}
