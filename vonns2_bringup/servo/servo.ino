#include <ESP32Servo.h>

Servo servo;
int servo_pin = 23;
int inferior_limit_angle = 0; // From 0°
int superior_limit_angle = 60; // To 60°

int servo_push_button_pin = 22;
bool is_rotation_allowed = false; 

int servo_blue_led_pin = 15;
int servo_green_led_pin = 2;
int servo_red_led_pin = 4;

void setup() {
  Serial.begin(115200);
  delay(50);

  servo.attach(servo_pin);
  servo.write(inferior_limit_angle);
  delay(50);

  pinMode(servo_push_button_pin, INPUT);
  delay(50);

  pinMode(servo_blue_led_pin, OUTPUT);
  delay(50);
  pinMode(servo_green_led_pin, OUTPUT);
  delay(50);
  pinMode(servo_red_led_pin, OUTPUT);
  delay(50);

  digitalWrite(servo_blue_led_pin, LOW);
  digitalWrite(servo_green_led_pin, LOW);
  digitalWrite(servo_red_led_pin, LOW);
  delay(50);
} 

void loop() {
  // Reading push button
  is_rotation_allowed = digitalRead(servo_push_button_pin);

  // If the push is pushed
  if(is_rotation_allowed == true){
    rotate_servo(inferior_limit_angle, superior_limit_angle);
  } 
  // If the push is not pushed
  else{
    // Servo is at 0°
    servo.write(inferior_limit_angle);
    Serial.println(inferior_limit_angle);

    // Red led is activated
    digitalWrite(servo_blue_led_pin, LOW);
    digitalWrite(servo_green_led_pin, LOW);
    digitalWrite(servo_red_led_pin, HIGH);
  }
  
  delay(250);
}

void rotate_servo(int inferior_limit_angle, int superior_limit_angle){
  for(int servo_angle = inferior_limit_angle; servo_angle <= superior_limit_angle; servo_angle += 1){
    servo.write(servo_angle); // Write angle in servomotor
    Serial.println(servo_angle); // Send angle though serial port
    delay(250);
    
    // Activate green LED at the superior angle
    if(servo_angle == superior_limit_angle){
      digitalWrite(servo_blue_led_pin, LOW);
      digitalWrite(servo_green_led_pin, HIGH);
      digitalWrite(servo_red_led_pin, LOW);
    }
    // Activate blue LED at any other angle
    else{
      digitalWrite(servo_blue_led_pin, HIGH);
      digitalWrite(servo_green_led_pin, LOW);
      digitalWrite(servo_red_led_pin, LOW);
    }
  }
}
