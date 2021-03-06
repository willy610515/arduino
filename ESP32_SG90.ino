#include <ESP32Servo.h>

Servo myservo;

int pos=0;
int servoPin = 18;
const byte interruptPin = 0;
int inPin = 0;

#if defined(ARDUINO_ESP32S2_DEV)
int servoPin = 17;
#else
int servoPin = 18;
#endif

void setup() {
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), rotate, FALLING);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 1000, 2000); 
}

void loop() {
    if (digitalRead(inPin)==1)
    rotate;
}

void rotate() {
  pos=pos+30;
  myservo.write(pos);
  delay(100); 
  
      if (pos>=180){
    pos=0;
    myservo.write(pos);
    delay(100); 
  }
}
