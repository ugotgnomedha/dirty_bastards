#include <DFRobot_LIS2DH12.h> 
#include <LiquidCrystal.h> 
#include <Wire.h>
#include "LIDARLite_v4LED.h" 
#include <Adafruit_Sensor.h> 
#include <Adafruit_TCS34725.h>

#define Motor_forward 0
#define Motor_return 1
#define Motor_L_dir_pin 7
#define Motor_R_dir_pin 8
#define Motor_L_pwm_pin 9
#define Motor_R_pwm_pin 10

LiquidCrystal lcd(25, 27, 5, 4, 11, 12); // (RS, E, D4, D5, D6, D7) LIDARLite_v4LED lidar;
DFRobot_LIS2DH12 LIS;
void setup() { Wire.begin(); Serial.begin(115200); while(!Serial); delay(100);
// Motor pins setup pinMode(Motor_L_dir_pin, OUTPUT); pinMode(Motor_R_dir_pin, OUTPUT); pinMode(Motor_L_pwm_pin, OUTPUT); pinMode(Motor_R_pwm_pin, OUTPUT);
// Initialize LCD lcd.begin(20, 4);
// Display initial message lcd.setCursor(0, 0); lcd.print("Arduino Lego Car"); lcd.setCursor(0, 1); lcd.print("Ready to drive!");
if (!lidar.begin()) {
lcd.clear();
lcd.print("LIDAR init failed!"); while (1); // halt if LIDAR init fails
}
while(LIS.init(LIS2DH12_RANGE_16GA) == -1){ //Equipment connection exception or I2C address error
Serial.println("No I2C devices found");

 delay(1000); }
}
void acceleration(void) {
int16_t x, y, z;
delay(100); LIS.readXYZ(x, y, z); LIS.mgScale(x, y, z);
float ax = x * 9.81 / 1000.0; float ay = y * 9.81 / 1000.0; float az = z * 9.81 / 1000.0;
Serial.print("Acceleration x: "); // Print acceleration Serial.print(ax);
Serial.print(" m/s2 \ty: ");
Serial.print(ay);
Serial.print(" m/s2 \tz: "); Serial.print(az); Serial.println(" m/s2");
}
void loop() {
// Your driving logic here
// Forward
acceleration();
digitalWrite(Motor_L_dir_pin, Motor_forward); digitalWrite(Motor_R_dir_pin, Motor_return);
// this has to be in return to go
forward:DD
analogWrite(Motor_L_pwm_pin, 200); // Slow speed analogWrite(Motor_R_pwm_pin, 200); // Slow speed delay(2000); // Run forward for 2 seconds
// Stop analogWrite(Motor_L_pwm_pin, 0); analogWrite(Motor_R_pwm_pin, 0); delay(500); // Wait for 1 second
// Backward
digitalWrite(Motor_L_dir_pin, Motor_return); forward:DD
// this has to be in return to go analogWrite(Motor_L_pwm_pin, 200); // Slow speed
digitalWrite(Motor_R_dir_pin, Motor_forward);

 analogWrite(Motor_R_pwm_pin, 200); // Slow speed delay(2000); // Run backward for 2 seconds
// Stop analogWrite(Motor_L_pwm_pin, 0); analogWrite(Motor_R_pwm_pin, 0); delay(500); // Wait for 1 second
// Read LIDAR distance
uint16_t distance = lidar.readDistance();
// Display LIDAR reading on LCD lcd.clear();
lcd.setCursor(0, 0); lcd.print("Distance: "); lcd.print(distance);
lcd.print(" cm"); }
