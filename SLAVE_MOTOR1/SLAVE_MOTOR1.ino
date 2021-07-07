//Parsing
String dataIn;
String arrayData[10];
boolean parsing = false;
boolean receive = false;

//Motor
const int mpwm = PB1; // Pin PWM
const int mtr1 = PB12; // Pin Motor 1
const int mtr2 = PB13; // Pin Motor 2
const int switchA = PB14; //Pin Encoder
const int switchB = PB15; //Pin Encoder
int stateA = 0;
int stateB = 0;
int setRPM1 = 0;

long previousMillis = 0; // Waktu Sebelumnya
long currentMillis = 0; // Waktu Sekarang
unsigned long interval = 100; //Satuan Waktu untuk revolution1


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  pinMode(mtr1, OUTPUT);
  pinMode(mtr2, OUTPUT);
  pinMode(switchA, INPUT_PULLUP);    //Pin Encoder Motor 1
  pinMode(switchB, INPUT_PULLUP);    //Pin Encoder Motor 1
  pinMode(mpwm, PWM);
}

void loop() {
  cekData();
  cekDataSerial();

  stateA = digitalRead(switchA);
  stateB = digitalRead(switchB);

  if ((setRPM1 > 0 && stateA == 0) || (setRPM1 < 0 && stateB == 0) || (setRPM1 == 0)) {
    setRPM1 = 0;
  }

  Serial.println(setRPM1);
  runMotor();
}

void runMotor() {
  if (setRPM1 > 0) {
    pwmWrite(mpwm, abs(setRPM1) * 256);
    //CounterClockWise
    digitalWrite(mtr1, HIGH);
    digitalWrite(mtr2, LOW);
    //Serial.println("Up");
  }

  if (setRPM1 < 0) {
    pwmWrite(mpwm, abs(setRPM1) * 256);
    digitalWrite(mtr1, LOW);
    digitalWrite(mtr2, HIGH);
    //Serial.println("Down");
  }

  if (setRPM1 == 0) {
    //Stop
    pwmWrite(mpwm, 0);
    digitalWrite(mtr1, HIGH);
    digitalWrite(mtr2, HIGH);
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
}
