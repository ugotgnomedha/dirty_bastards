#include <LiquidCrystal.h>

LiquidCrystal lcd(9, 8, 5, 4, 3, 2); // (RS, E, D4, D5, D6, D7)

void setup() {
  // Initialize the LCD
  lcd.begin(20, 4); // Change these values according to your LCD configuration

  // Print initial messages
  lcd.print("Pot Voltage:");
  lcd.setCursor(0, 1);
  lcd.print("Joystick:");

  // Set up ADC for reading analog values
  analogReference(DEFAULT); // Use default reference voltage (usually 5V)

  Serial.begin(9600);
}

void loop() {
  // Read the analog voltage from the potentiometer
  int potValue = analogRead(A15); // Connect potentiometer to analog pin A15
  int xValue = analogRead(A8); // Connect X output of the joystick to A8
  int yValue = analogRead(A9); // Connect Y output of the joystick to A9

  float potVoltage = map(potValue, 0, 1023, 0, 5000) / 1000.0; // Map to voltage (0-5V)

  // Calculate joystick percentage
  int xPercentage = map(xValue, 0, 1023, -100, 100);
  int yPercentage = map(yValue, 0, 1023, -100, 100);

  lcd.setCursor(13, 0);
  lcd.print(potVoltage, 2); // Display pot voltage with 2 decimal places

  lcd.setCursor(12, 1);
  lcd.print(xPercentage);
  lcd.print("%  ");

  lcd.setCursor(0, 2);

  // Calculate the position of the joystick indicator
  int indicatorPosition = map(xPercentage, -100, 100, 0, 19);

  // Print the joystick indicator (a vertical bar) on the bottom row of the LCD
  lcd.print("-100%    0%     100%");
  // Clear the last row of the LCD
  lcd.setCursor(0, 3);
  lcd.print("                    "); // Clear the entire row
  lcd.setCursor(indicatorPosition, 3);
  lcd.write('|');

  // Print the voltages and joystick percentages to the Serial Monitor
  Serial.print("Pot Voltage: ");
  Serial.print(potVoltage, 2);
  Serial.print(" V\tJoystick: ");
  Serial.print(xPercentage);

  delay(100);
}
