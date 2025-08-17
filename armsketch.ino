#include <Servo.h>

Servo servo1;  
Servo servo2;  
Servo servo3;  
Servo servo4;  
Servo servo5;  
Servo servo6;  

void setup() {
  servo1.attach(11);
  servo2.attach(10);
  servo3.attach(9);
  servo4.attach(6);
  servo5.attach(5);
  servo6.attach(3);

  servo1.write(90);
  servo2.write(90);
  servo3.write(0);
  servo4.write(90);
  servo5.write(90);
  servo6.write(90);

  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');  // Read the incoming string
    Serial.print("Received: ");
    Serial.println(data);

    int angles[6];  // to store parsed angles
    int index = 0;
    char *token = strtok((char*)data.c_str(), ",");

    while (token != NULL && index < 6) {
      angles[index++] = atoi(token);  // convert string to integer
      token = strtok(NULL, ",");
    }

    if (index == 6) {  // only set if we got 6 values
      servo1.write(angles[0]);
      servo2.write(angles[1]);
      servo3.write(angles[2]);
      servo4.write(angles[3]);
      servo5.write(angles[4]);
      servo6.write(angles[5]);
    } else {
      Serial.println("Error: Expected 6 values");
    }
  }
}
