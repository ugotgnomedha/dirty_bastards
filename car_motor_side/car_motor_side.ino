#define Motor_forward         0
#define Motor_return          1
#define Motor_L_dir_pin       7
#define Motor_R_dir_pin       8
#define Motor_L_pwm_pin       9
#define Motor_R_pwm_pin       10

void setup() {
  pinMode(Motor_L_dir_pin, OUTPUT);
  pinMode(Motor_R_dir_pin, OUTPUT);
  pinMode(Motor_L_pwm_pin, OUTPUT);
  pinMode(Motor_R_pwm_pin, OUTPUT);

  Serial.begin(115200);
  Serial1.begin(115200); // Initialize Serial1 for communication with ESP8266
}

void loop() {
  if (Serial1.available() > 0) {
    String receivedData = Serial1.readStringUntil('\n');

    // Parse received joystick values
    int xPercentage, yPercentage;
    if (sscanf(receivedData.c_str(), "Received data: X:%d,Y:%d", &xPercentage, &yPercentage) == 2) {
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
      Serial.print("L:");
      Serial.print(abs(pwm_L));
      Serial.print(",R:");
      Serial.println(abs(pwm_R));
    }
  }

  // Your motor control logic here (e.g., button control)
}

