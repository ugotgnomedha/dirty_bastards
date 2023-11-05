void setup() {
  Serial.begin(9600); // Initialize the serial communication at 9600 bps
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n'); // Read data from Arduino
    Serial.print("Received: ");
    Serial.println(data);
    
    // Add your code to process the data here
  }
}
