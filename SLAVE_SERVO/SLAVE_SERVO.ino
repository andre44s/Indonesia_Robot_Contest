#include <Servo.h>

//Parsing
String dataIn;
String arrayData[10];
boolean parsing = false;
boolean receive = false;

int servo1Pin = PA0;
int servo2Pin = PA1;
int servo3Pin = PA2;
int servo4Pin = PA6;
int servo5Pin = PA7;

int servo6Pin = PA3;
int servo7Pin = PB1;
int servo8Pin = PB0;

int servo15Degree = 0;
int servo6Degree = 0;
int servo7Degree = 0;
int servo8Degree = 0;

Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo5;
Servo Servo6;
Servo Servo7;
Servo Servo8;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);

  Servo1.attach(servo1Pin);
  Servo2.attach(servo2Pin);
  Servo3.attach(servo3Pin);
  Servo4.attach(servo4Pin);
  Servo5.attach(servo5Pin);
  Servo6.attach(servo6Pin);
  Servo7.attach(servo7Pin);
  Servo8.attach(servo8Pin);
}

void loop() {
  cekData();
  cekDataSerial();

  Servo1.write(servo15Degree);
  Servo2.write(servo15Degree);
  Servo3.write(servo15Degree);
  Servo4.write(servo15Degree);
  Servo5.write(servo15Degree);
  Servo6.write(servo6Degree);
  Servo7.write(servo7Degree);
  Servo8.write(servo8Degree);

  Serial.print(servo15Degree);
  Serial.print("  ");
  Serial.print(servo6Degree);
  Serial.print("  ");
  Serial.print(servo7Degree);
  Serial.print("  ");
  Serial.println(servo8Degree);
}

void cekData() {
  while (Serial3.available() > 0) {
    char inChar = (char)Serial3.read();
    if (inChar == '*') {
      dataIn = "";
      receive = true;
    }

    if (receive) {
      dataIn += inChar;
      if (inChar == '#') {
        parsing = true;
      }
    }

    if (parsing) {
      parsingData();
      parsing = false;
      receive = false;
    }
  }
}

void cekDataSerial() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '*') {
      dataIn = "";
      receive = true;
    }

    if (receive) {
      dataIn += inChar;
      if (inChar == '#') {
        parsing = true;
      }
    }

    if (parsing) {
      parsingData();
      parsing = false;
      receive = false;
    }
  }
}

void parsingData() {
  int j = 0;
  arrayData[j] = "";

  for (int i = 1; i < dataIn.length(); i++) {
    if ((dataIn[i] == '#') || (dataIn[i] == ',')) {
      j++;
      arrayData[j] = "";
    }
    else {
      arrayData[j] = arrayData[j] + dataIn[i];
    }
  }
  servo15Degree = arrayData[0].toFloat();
  servo6Degree = arrayData[1].toFloat();
  servo7Degree = arrayData[2].toFloat();
  servo8Degree = arrayData[3].toFloat();
}
