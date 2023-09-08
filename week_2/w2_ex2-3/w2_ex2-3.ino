#include <LiquidCrystal.h>

LiquidCrystal lcd(9, 8, 5, 4, 3, 2); // (RS, E, D4, D5, D6, D7)
int buttonPin = 19; // Button connected to digital pin 19
int buttonState = LOW;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int buttonPressCount = 0;

void setup() {
  // Initialize the LCD
  lcd.begin(20, 4); // Change these values according to your LCD configuration

  lcd.setCursor(0, 0);
  lcd.print("Joystick:");

  // Set up ADC for reading analog values
  analogReference(DEFAULT); // Use default reference voltage (usually 5V)

  // Initialize the button pin
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(9600);
}

void loop() {
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
