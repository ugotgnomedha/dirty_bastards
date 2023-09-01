#include <LiquidCrystal.h>

// Initialize the LCD
LiquidCrystal lcd(12, 11, 5, 4, 3, 2); // (RS, E, D4, D5, D6, D7)

void setup() {
  // Initialize the LCD
  lcd.begin(20, 4); // Change these values according to your LCD configuration

  // Print initial messages
  lcd.print("Pot Voltage:");
  lcd.setCursor(0, 1);
  lcd.print("3.3V Voltage:");

  // Set up ADC for reading analog values
  analogReference(DEFAULT); // Use default reference voltage (usually 5V)

  // Set up Serial communication for debugging (optional)
  Serial.begin(9600);
}

void loop() {
  // Read the analog voltage from the potentiometer
  int potValue = analogRead(A0); // Connect potentiometer to analog pin A0
  float potVoltage = map(potValue, 0, 1023, 0, 5000) / 1000.0; // Map to voltage (0-5V)

  // Read the voltage of the 3.3V pin
  float voltage3V3 = 3.3; // The 3.3V voltage is constant

  // Display the voltages on the LCD
  lcd.setCursor(13, 0);
  lcd.print(potVoltage, 2); // Display pot voltage with 2 decimal places

  lcd.setCursor(13, 1);
  lcd.print(voltage3V3, 2); // Display 3.3V voltage with 2 decimal places

  // Print the voltages to the Serial Monitor (optional)
  Serial.print("Pot Voltage: ");
  Serial.print(potVoltage, 2);
  Serial.print(" V\t3.3V Voltage: ");
  Serial.print(voltage3V3, 2);
  Serial.println(" V");

  delay(1000); // Delay for readability (adjust as needed)
}
