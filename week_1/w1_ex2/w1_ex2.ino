// Define the LED pins
const int redLED = A1;
const int yellowLED = A2;
const int greenLED = A3;

void setup() {
  // Initialize LED pins as OUTPUT
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  
  // Start serial communication at 9600 baud
  Serial.begin(9600);
}

void loop() {
  // Check if there is serial data available
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    // Handle user commands
    switch (command) {
      case 'R':
      case 'r':
        // Turn on the red LED
        digitalWrite(redLED, HIGH);
        digitalWrite(yellowLED, LOW);
        digitalWrite(greenLED, LOW);
        break;
        
      case 'Y':
      case 'y':
        // Turn on the yellow LED
        digitalWrite(redLED, LOW);
        digitalWrite(yellowLED, HIGH);
        digitalWrite(greenLED, LOW);
        break;
        
      case 'G':
      case 'g':
        // Turn on the green LED
        digitalWrite(redLED, LOW);
        digitalWrite(yellowLED, LOW);
        digitalWrite(greenLED, HIGH);
        break;
        
      case 'O':
      case 'o':
        // Turn off all LEDs
        digitalWrite(redLED, LOW);
        digitalWrite(yellowLED, LOW);
        digitalWrite(greenLED, LOW);
        break;
    }
  }
}
