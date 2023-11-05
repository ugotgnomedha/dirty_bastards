#include <ESP8266WiFi.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

int serverFound;
int serverPort = 80;
const int ledPin = 2; // GPIO pin for the LED

void setup() {
  Serial.begin(115200);
  delay(10);

  // Connect to Wi-Fi
  WiFi.begin("iPhone_", "gagarin11");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize mDNS (Multicast DNS) with a service name
  MDNS.begin("esp8266-client");

  // Set the LED pin as an output
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // Discover the server using mDNS
  Serial.println("Discovering server...");
  serverFound = MDNS.queryService("esp8266-server", "tcp");
  delay(5000);

  if (serverFound) {
    Serial.println("Server discovered!");
    Serial.print("Server IP address: ");
    Serial.println(MDNS.IP(0));

    // Blink the LED to indicate Wi-Fi connection
    blinkLED();

    // Send a "Hello" message to the server
    WiFiClient client;
    if (client.connect(MDNS.IP(0), serverPort)) {
      client.print("Hello");
      client.stop();
    }

    // Delay for 1 second
    delay(1000);
  } else {
    Serial.println("Server not found!");
    // You can add a delay here before attempting to discover the server again
    delay(5000); // Wait for 5 seconds before the next discovery attempt
  }
}

void blinkLED() {
  // Blink the LED three times with a short delay to indicate Wi-Fi connection
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}
