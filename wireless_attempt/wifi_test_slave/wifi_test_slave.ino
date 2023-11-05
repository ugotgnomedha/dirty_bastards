void setup() {
  Serial.begin(9600); // Initialize the serial communication at 9600 bps
}

void loop() {
  Serial.println("Hello, ESP8266!"); // Send data over serial
  delay(1000); // Wait for a second
}
