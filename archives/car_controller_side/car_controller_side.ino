#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

LiquidCrystal lcd(25, 27, 5, 4, 3, 2); // (RS, E, D4, D5, D6, D7)
SoftwareSerial espSerial(2, 3); // SoftwareSerial for communication with the ESP8266

void setup() {
  // Initialize the LCD
  lcd.begin(20, 4); // Change these values according to your LCD configuration
  lcd.setCursor(0, 0);
  lcd.print("Joystick:");

  // Set up ADC for reading analog values
  analogReference(DEFAULT); // Use default reference voltage (usually 5V)

  // Initialize SoftwareSerial for ESP8266 communication
  espSerial.begin(115200);

  // Initialize Serial for debugging
  Serial.begin(115200);
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

  // Send joystick values to the ESP8266
  espSerial.print("X:");
  espSerial.print(xPercentage);
  espSerial.print(",Y:");
  espSerial.println(yPercentage);

  Serial.print("X:");
  Serial.print(xPercentage);
  Serial.print(",Y:");
  Serial.println(yPercentage);

  delay(20);
}

