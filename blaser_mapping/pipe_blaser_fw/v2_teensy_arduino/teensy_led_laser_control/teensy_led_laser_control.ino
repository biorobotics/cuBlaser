const int laser_pin = 14;
const int led_pin = 15;
const int onboard_led_pin = 13;

int LED_STATE = LOW;
int LASER_STATE = LOW;

void setup() {
  pinMode(led_pin, OUTPUT);
  pinMode(laser_pin, OUTPUT);
  pinMode(onboard_led_pin, OUTPUT);
  digitalWrite(laser_pin, LASER_STATE);
  digitalWrite(led_pin, LED_STATE);
  Serial.begin(9600);
}

void loop() {
  while (Serial.available())
  {
    String cmd = Serial.readString();
    LED_STATE = cmd[0] - '0';
    LASER_STATE = cmd[1] - '0';
    digitalWrite(laser_pin, LASER_STATE);
    digitalWrite(led_pin, LED_STATE);
  }
  
}
