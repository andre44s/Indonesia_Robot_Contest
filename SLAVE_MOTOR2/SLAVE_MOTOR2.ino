#include <SoftwareSerial.h>

SoftwareSerial myserial(3, 4);

//Parsing
String dataIn;
String arrayData[10];
boolean parsing = false;
boolean receive = false;

//Motor
const int mpwm1 = 5; // Pin PWM
const int mpwm2 = 6;
const int mtr1 = 9; // Pin Motor 1
const int mtr2 = 10; // Pin Motor 2
int setRPM1;

void setup() {
  Serial.begin(115200);
  myserial.begin(9600);

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
    digitalWrite(mpwm1, LOW);
    digitalWrite(mpwm2, HIGH);
    analogWrite(mtr1, abs(setRPM1));
    analogWrite(mtr2, abs(setRPM1));
    Serial.println("Up");
  }

  if (setRPM1 < 0) {
    //ClockWise
    digitalWrite(mpwm1, HIGH);
    digitalWrite(mpwm2, LOW);
    analogWrite(mtr1, abs(setRPM1));
    analogWrite(mtr2, abs(setRPM1));
    Serial.println("Down");
  }

  if (setRPM1 == 0) {
    //Stop
    digitalWrite(mpwm1, LOW);
    digitalWrite(mpwm2, LOW);
    analogWrite(mtr1, 0);
    analogWrite(mtr2, 0);
  }
}

void cekData() {
  while (myserial.available() > 0) {
    char inChar = (char)myserial.read();
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
