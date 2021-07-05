//Parsing
String dataIn;
String arrayData[10];
boolean parsing = false;
boolean receive = false;

//Motor 1
const int mpwm1 = PB1; // Pin PWM
const int mtr1a = PA8; // Pin Motor A
const int mtr1b = PA9; // Pin Motor B
#define ENC_COUNT_REV1 540 //Jumlah Gigi Encoder
const int enc1a = PB14; //Pin Encoder
const int enc1b = PB15; //Pin Encoder
long revolutions1 = 0; // Putaran dalam satuan waktu
long setRPM1 = 0; //Setpoint RPM
long currentRPM1 = 0; //Kecepatan yang dibaca

//Motor 2
const int mpwm2 = PB0; // Pin PWM
const int mtr2a = PA10; // Pin Motor A
const int mtr2b = PC11; // Pin Motor B
#define ENC_COUNT_REV2 540 //Jumlah Gigi Encoder
const int enc2a = PB12; //Pin Encoder
const int enc2b = PB13; //Pin Encoder
long revolutions2 = 0; // Putaran dalam satuan waktu
long setRPM2 = 0; //Setpoint RPM
long currentRPM2 = 0; //Kecepatan yang dibaca

//Motor 3
const int mpwm3 = PA7; // Pin PWM
const int mtr3a = PB3; // Pin Motor 1
const int mtr3b = PA15; // Pin Motor 2
#define ENC_COUNT_REV3 540 //Jumlah Gigi Encoder
const int enc3a = PB6; //Pin Encoder
const int enc3b = PB7; //Pin Encoder
long revolutions3 = 0; // Putaran dalam satuan waktu
long setRPM3 = 0; //Setpoint RPM
long currentRPM3 = 0; //Kecepatan yang dibaca

//Motor 4
const int mpwm4 = PA6; // Pin PWM
const int mtr4a = PB4; // Pin Motor A
const int mtr4b = PB5; // Pin Motor B
#define ENC_COUNT_REV4 540 //Jumlah Gigi Encoder
const int enc4a = PB8; //Pin Encoder
const int enc4b = PB9; //Pin Encoder
long revolutions4 = 0; // Putaran dalam satuan waktu
long setRPM4 = 0; //Setpoint RPM
long currentRPM4 = 0; //Kecepatan yang dibaca

long previousMillis = 0; // Waktu Sebelumnya
long currentMillis = 0; // Waktu Sekarang
unsigned long interval = 100; //Satuan Waktu untuk revolution1

//PID
float PIDValue1 = 0, pwmValue1 = 0;
double error1, previouserror1;
double P_1 = 0, I_1 = 0, D_1 = 0;

float PIDValue2 = 0, pwmValue2 = 0;
double error2, previouserror2;
double P_2 = 0, I_2 = 0, D_2 = 0;

float PIDValue3 = 0, pwmValue3 = 0;
double error3, previouserror3;
double P_3 = 0, I_3 = 0, D_3 = 0;

float PIDValue4 = 0, pwmValue4 = 0;
double error4, previouserror4;
double P_4 = 0, I_4 = 0, D_4 = 0;

float Kp1 = 25;    //Kp MOTOR 1
float Ki1 = 0;    //Ki MOTOR 1
float Kd1 = 30;      //Kd MOTOR 1

float Kp2 = 17;    //Kp MOTOR 2
float Ki2 = 0;    //Ki MOTOR 2
float Kd2 = 20;      //Kd MOTOR 2

float Kp3 = 15;    //Kp MOTOR 3
float Ki3 = 0;    //Ki MOTOR 3
float Kd3 = 20;      //Kd MOTOR 3

float Kp4 = 15;    //Kp MOTOR 3
float Ki4 = 0;    //Ki MOTOR 3
float Kd4 = 20;      //Kd MOTOR 3

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  pinMode(enc1a, INPUT);
  pinMode(enc1b, INPUT);
  pinMode(mpwm1, PWM);
  pinMode(mtr1a, OUTPUT);
  pinMode(mtr1b, OUTPUT);

  pinMode(enc2a, INPUT);
  pinMode(enc2b, INPUT);
  pinMode(mpwm2, PWM);
  pinMode(mtr2a, OUTPUT);
  pinMode(mtr2b, OUTPUT);

  pinMode(enc3a, INPUT);
  pinMode(enc3b, INPUT);
  pinMode(mpwm3, PWM);
  pinMode(mtr3a, OUTPUT);
  pinMode(mtr3b, OUTPUT);

  pinMode(enc4a, INPUT);
  pinMode(enc4b, INPUT);
  pinMode(mpwm4, PWM);
  pinMode(mtr4a, OUTPUT);
  pinMode(mtr4b, OUTPUT);

  previousMillis = millis();
  attachInterrupt(digitalPinToInterrupt(enc1a), func1, CHANGE); //Attach interrupt
  attachInterrupt(digitalPinToInterrupt(enc1b), func1, CHANGE); //Attach interrupt
  attachInterrupt(digitalPinToInterrupt(enc2a), func2, CHANGE); //Attach interrupt
  attachInterrupt(digitalPinToInterrupt(enc2b), func2, CHANGE); //Attach interrupt
  attachInterrupt(digitalPinToInterrupt(enc3a), func3, CHANGE); //Attach interrupt
  attachInterrupt(digitalPinToInterrupt(enc3b), func3, CHANGE); //Attach interrupt
  attachInterrupt(digitalPinToInterrupt(enc4a), func4, CHANGE); //Attach interrupt
  attachInterrupt(digitalPinToInterrupt(enc4b), func4, CHANGE); //Attach interrupt
}

void loop() {
  cekData();
  cekDataSerial();

  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    // Calculate RPM
    currentRPM1 = (float)(revolutions1 * 600 / ENC_COUNT_REV1);
    currentRPM2 = (float)(revolutions2 * 600 / ENC_COUNT_REV2);
    currentRPM3 = (float)(revolutions3 * 600 / ENC_COUNT_REV3);
    currentRPM4 = (float)(revolutions4 * 600 / ENC_COUNT_REV4);

    Serial.print(setRPM1);
    Serial.print("\t");
    Serial.print(currentRPM1);
    Serial.print("\t");
    Serial.print(setRPM2);
    Serial.print("\t");
    Serial.print(currentRPM2);
    Serial.print("\t");
    Serial.print(setRPM3);
    Serial.print("\t");
    Serial.print(currentRPM3);
    Serial.print("\t");
    Serial.print(setRPM4);
    Serial.print("\t");
    Serial.println(currentRPM4);
    Serial.flush();

    VPID1();
    VPID2();
    VPID3();
    VPID4();

    revolutions1 = 0;
    revolutions2 = 0;
    revolutions3 = 0;
    revolutions4 = 0;

  }
  runMotor();
}

void runMotor() {
  pwmWrite(mpwm1, pwmValue1);
  pwmWrite(mpwm2, pwmValue2);
  pwmWrite(mpwm3, pwmValue3);
  pwmWrite(mpwm4, pwmValue4);

  if (setRPM1 > 0) {
    //CounterClockWise
    digitalWrite(mtr1a, HIGH);
    digitalWrite(mtr1b, LOW);
  }

  if (setRPM1 < 0) {
    //ClockWise
    digitalWrite(mtr1a, LOW);
    digitalWrite(mtr1b, HIGH);
  }

  if (setRPM1 == 0) {
    //Stop
    pwmWrite(mpwm1, 0);
    digitalWrite(mtr1a, HIGH);
    digitalWrite(mtr1b, HIGH);
  }

  if (setRPM2 > 0) {
    //CounterClockWise
    digitalWrite(mtr2a, HIGH);
    digitalWrite(mtr2b, LOW);
  }

  if (setRPM2 < 0) {
    //ClockWise
    digitalWrite(mtr2a, LOW);
    digitalWrite(mtr2b, HIGH);
  }

  if (setRPM2 == 0) {
    //Stop
    pwmWrite(mpwm2, 0);
    digitalWrite(mtr2a, HIGH);
    digitalWrite(mtr2b, HIGH);
  }

  if (setRPM3 > 0) {
    //CounterClockWise
    digitalWrite(mtr3a, HIGH);
    digitalWrite(mtr3b, LOW);
  }

  if (setRPM3 < 0) {
    //ClockWise
    digitalWrite(mtr3a, LOW);
    digitalWrite(mtr3b, HIGH);
  }

  if (setRPM3 == 0) {
    //Stop
    pwmWrite(mpwm3, 0);
    digitalWrite(mtr3a, HIGH);
    digitalWrite(mtr3b, HIGH);
  }


  if (setRPM4 > 0) {
    //CounterClockWise
    digitalWrite(mtr4a, HIGH);
    digitalWrite(mtr4b, LOW);
  }

  if (setRPM4 < 0) {
    //ClockWise
    digitalWrite(mtr4a, LOW);
    digitalWrite(mtr4b, HIGH);
  }

  if (setRPM4 == 0) {
    //Stop
    pwmWrite(mpwm4, 0);
    digitalWrite(mtr4a, HIGH);
    digitalWrite(mtr4b, HIGH);
  }
}

void cekData() {
  while (Serial2.available() > 0) {
    char inChar = (char)Serial2.read();
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
  setRPM2 = arrayData[1].toFloat();
  setRPM3 = arrayData[2].toFloat();
  setRPM4 = arrayData[3].toFloat();
}

void func1() {
  revolutions1++;
}

void func2() {
  revolutions2++;
}

void func3() {
  revolutions3++;
}

void func4() {
  revolutions4++;
}

void VPID1() {

  error1 = abs(setRPM1) - currentRPM1;
  P_1 = error1;
  I_1 = I_1 + error1;
  D_1 = error1 - previouserror1;

  if (I_1 > 65535) {
    I_1 = 65535;
  }
  if (I_1 < (0)) {
    I_1 = (0);
  }

  PIDValue1 = (Kp1 * P_1) + (Ki1 * I_1) + (Kd1 * D_1);
  previouserror1 = error1;
  pwmValue1 = pwmValue1 + PIDValue1;

  if (pwmValue1 >= 65535) {
    pwmValue1 = 65535;
  }
  if (pwmValue1 <= (0)) {
    pwmValue1 = (0);
  }
  if (setRPM1 == 0) {
    pwmValue1 = 0;
  }
}

void VPID2() {

  error2 = abs(setRPM2) - currentRPM2;
  P_2 = error2;
  I_2 = I_2 + error2;
  D_2 = error2 - previouserror2;

  if (I_2 > 65535) {
    I_2 = 65535;
  }
  if (I_2 < (0)) {
    I_2 = (0);
  }

  PIDValue2 = (Kp2 * P_2) + (Ki2 * I_2) + (Kd2 * D_2);
  previouserror2 = error2;
  pwmValue2 = pwmValue2 + PIDValue2;

  if (pwmValue2 >= 65535) {
    pwmValue2 = 65535;
  }
  if (pwmValue2 <= (0)) {
    pwmValue2 = (0);
  }
  if (setRPM2 == 0) {
    pwmValue2 = 0;
  }
}

void VPID3() {

  error3 = abs(setRPM3) - currentRPM3;
  P_3 = error3;
  I_3 = I_3 + error3;
  D_3 = error3 - previouserror3;

  if (I_3 > 65535) {
    I_3 = 65535;
  }
  if (I_3 < (0)) {
    I_3 = (0);
  }

  PIDValue3 = (Kp3 * P_3) + (Ki3 * I_3) + (Kd3 * D_3);
  previouserror3 = error3;
  pwmValue3 = pwmValue3 + PIDValue3;

  if (pwmValue3 >= 65535) {
    pwmValue3 = 65535;
  }
  if (pwmValue3 <= (0)) {
    pwmValue3 = (0);
  }
  if (setRPM3 == 0) {
    pwmValue3 = 0;
  }
}

void VPID4() {

  error4 = abs(setRPM4) - currentRPM4;
  P_4 = error4;
  I_4 = I_4 + error4;
  D_4 = error4 - previouserror4;

  if (I_4 > 65535) {
    I_4 = 65535;
  }
  if (I_4 < (0)) {
    I_4 = (0);
  }

  PIDValue4 = (Kp4 * P_4) + (Ki4 * I_4) + (Kd4 * D_4);
  previouserror4 = error4;
  pwmValue4 = pwmValue4 + PIDValue4;

  if (pwmValue4 >= 65535) {
    pwmValue4 = 65535;
  }
  if (pwmValue4 <= (0)) {
    pwmValue4 = (0);
  }
  if (setRPM4 == 0) {
    pwmValue4 = 0;
  }
}
