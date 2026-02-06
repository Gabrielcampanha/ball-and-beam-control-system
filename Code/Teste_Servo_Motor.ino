//--------Teste servo motor-----------
#include <ESP32Servo.h>
Servo myservo;

void setup() {
myservo.attach(13);
}

void loop() {
myservo.write(127); //Teste para encontrar o ângulo em que a barra fique na horizontal
}
