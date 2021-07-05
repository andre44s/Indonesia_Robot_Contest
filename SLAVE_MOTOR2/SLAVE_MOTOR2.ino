#include <SoftwareSerial.h>

SoftwareSerial Serial1(7, 6);

//Parsing
String dataIn;
String arrayData[10];
boolean parsing = false;
boolean receive = false;

//Motor
const int mpwm1 = 4; // Pin PWM
const int mpwm2 = 5;
const int mtr1 = 2; // Pin Motor 1
const int mtr2 = 3; // Pin Motor 2
int setRPM1 = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

  pinMode(mtr1, OUTPUT);
  pinMode(mtr2, OUTPUT);
  pinMode(mpwm1, OUTPUT);
  pinMode(mpwm2, OUTPUT);
}

void loop() {
  cekData();
  cekDataSerial();

  Serial.println(setRPM1);

  runMotor();
}

void runMotor() {
  if (setRPM1 > 0) {
    //CounterClockWise
    analogWrite(mpwm1, abs(setRPM1));
    analogWrite(mpwm2, 0);
    digitalWrite(mtr1, HIGH);
    digitalWrite(mtr2, HIGH);
    //Serial.println("Up");
  }

  if (setRPM1 < 0) {
    //ClockWise
    analogWrite(mpwm1, 0);
    analogWrite(mpwm2, abs(setRPM1));
    digitalWrite(mtr1, HIGH);
    digitalWrite(mtr2, HIGH);
    //Serial.println("Down");
  }

  if (setRPM1 == 0) {
    //Stop
    analogWrite(mpwm1, 0);
    analogWrite(mpwm2, 0);
    digitalWrite(mtr1, LOW);
    digitalWrite(mtr2, LOW);
  }
}

void cekData() {
  while (Serial1.available() > 0) {
    char inChar = (char)Serial1.read();
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
  setRPM1 = arrayData[0].toFloat();
}
