#include <LiquidCrystal.h>

#define Motor_forward         0
#define Motor_return          1
#define Motor_L_dir_pin       7
#define Motor_R_dir_pin       8
#define Motor_L_pwm_pin       9
#define Motor_R_pwm_pin       10

LiquidCrystal lcd(25, 27, 5, 4, 3, 2); // (RS, E, D4, D5, D6, D7)
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

//  Serial.begin(9600);
}

void loop() {
  int xValue = analogRead(A8); // Connect X output of the joystick to A8
  int yValue = analogRead(A9); // Connect Y output of the joystick to A9

  // Calculate joystick percentages
  int xPercentage = map(xValue, 0, 1023, -100, 100);
  int yPercentage = map(yValue, 0, 1023, -100, 100);
  
  // Display joystick values on the LCD
  lcd.setCursor(12, 0);
  lcd.print(xPercentage);
  lcd.print("%  ");
  
  lcd.setCursor(0, 1);
  lcd.print(yPercentage);
  lcd.print("%  ");
  
  lcd.setCursor(0, 2);
  lcd.print("                    "); // Clear the entire row
  lcd.setCursor(0, 2);
  lcd.print("X: ");
  lcd.print(xPercentage);
  lcd.print("%  Y: ");
  lcd.print(yPercentage);
  lcd.print("%");

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

  // Motor control logic based on joystick values
  int pwm_R = 0;
  int pwm_L = 0;

  // Determine motor speeds based on Y values for forward and backward motion
  pwm_L = map(yPercentage, -100, 100, -255, 255);
  pwm_R = map(yPercentage, -100, 100, -255, 255);

  // Apply X-axis control for differential steering
  pwm_L += xPercentage;
  pwm_R -= xPercentage;

  // Ensure motor values are within the valid range (0 to 255)
  pwm_L = constrain(pwm_L, -255, 255);
  pwm_R = constrain(pwm_R, -255, 255);

  // Set motor directions
  if (pwm_L >= 0) {
    digitalWrite(Motor_L_dir_pin, Motor_forward);
  } else {
    digitalWrite(Motor_L_dir_pin, Motor_return);
  }

  if (pwm_R >= 0) {
    digitalWrite(Motor_R_dir_pin, Motor_forward);
  } else {
    digitalWrite(Motor_R_dir_pin, Motor_return);
  }

  // Write PWM values to motors (taking the absolute values)
  analogWrite(Motor_L_pwm_pin, abs(pwm_L));
  analogWrite(Motor_R_pwm_pin, abs(pwm_R));

  // Print motor PWM values to Serial Monitor
//  Serial.print(" LeftPWM: ");
//  Serial.print(abs(pwm_L));
//  Serial.print(" RightPWM: ");
//  Serial.println(abs(pwm_R));

  delay(20);
}
