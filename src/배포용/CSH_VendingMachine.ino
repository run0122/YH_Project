#include <Servo.h>

Servo myservo;

int pos1 = 0;
int pos = 0;

void setup() {

  Serial.begin(9600);
}

void servoDrop(int servoNumber){
  myservo.attach(servoNumber);
    delay(100);
    for (pos1 = 0; pos1 < 360; pos1 += 1) 
    { 
      if(pos1 >= 180)
        pos = 359-pos1;
      else
        pos = pos1;  
      myservo.write(pos);            
      delay(15);                      
    }
    delay(500);
    myservo.detach();
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == '1'){
      servoDrop(11)
    }
    else if (cmd == '2'){
      servoDrop(10)
    }
    else if (cmd == '3'){
      servoDrop(9)
    }
  }
}