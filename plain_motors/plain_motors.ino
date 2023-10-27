#define Motor_forward         0
#define Motor_return          1
#define Motor_L_dir_pin       7
#define Motor_R_dir_pin       8
#define Motor_L_pwm_pin       9
#define Motor_R_pwm_pin       10

void setup()
{
    Serial.begin(9600);
}

bool ran = false;

void loop()
{
  if (ran == false){
    int pwm_R = 0;
    int pwm_L = 0;

    // Move straight.
    digitalWrite(Motor_R_dir_pin, Motor_forward);
    digitalWrite(Motor_L_dir_pin, Motor_forward);
    pwm_L = 255;
    pwm_R = 255;
    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);
    delay(850);

    // Stop.
    pwm_L = 0;
    pwm_R = 0;
    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);

    // Turn 90 degrees.
    digitalWrite(Motor_R_dir_pin, Motor_return);
    digitalWrite(Motor_L_dir_pin, Motor_forward);
    pwm_L = 255;
    pwm_R = 0;
    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);
    delay(750);

    // Stop.
    pwm_L = 0;
    pwm_R = 0;
    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);

    // Move straight.
    digitalWrite(Motor_R_dir_pin, Motor_forward);
    digitalWrite(Motor_L_dir_pin, Motor_forward);
    pwm_L = 255;
    pwm_R = 255;
    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);
    delay(550);

    // Stop
    pwm_L = 0;
    pwm_R = 0;
    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);

    ran = true;
  }
}
