#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <SoftwareSerial.h>

int a, b, c, d;

SoftwareSerial Serial4(2, 3); //rx tx

RF24 radio(A10, A11);   // nRF24L01 (CE, CSN)
const byte address[6] = "000111"; // Address

//Variable Motor
int rpm_x, rpm_y, rotateSpeed;
int rpm_1, rpm_2, rpm_3, rpm_4;
int max_rpm = 100;

//Variabel Sensor PING
//Sensor Depan = 1
const int TRIG_PIN1 = 4;
const int ECHO_PIN1 = 5;
long duration1;
int distance1;

//Sensor Kanan = 2
const int TRIG_PIN2 = 8;
const int ECHO_PIN2 = 9;
long duration2;
int distance2;

//Sensor Belakang = 3
const int TRIG_PIN3 = 11;
const int ECHO_PIN3 = 10;
long duration3;
int distance3;

//Sensor Kiri = 4
const int TRIG_PIN4 = 7;
const int ECHO_PIN4 = 6;
long duration4;
int distance4;

int stateServo13  = false;
int stateServo45  = false;
int stateServo6  = false;
int stateServo7 = false;
int stateServo8 = false;

int dataServo13 = 0;
int dataServo4 = 0;
int dataServo5 = 0;
int dataServo6 = 0;
int dataServo7 = 0;
int dataServo8 = 0;

int defaultXO = 0;
int defaultXC = 100;
int default3O = 0;
int default3C = 100;

int stateLengan = 0;
int dataLengan = 0;

int devider = 0;
int inDelay = 30000;

long long millisButton = 0, currentMillis = 0, prevMillis = 0;

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  byte LX;
  byte LY;
  byte RX;
  byte RY;
  byte XButton;
  byte TButton;
  byte SButton;
  byte CButton;
  byte UButton;
  byte DButton;
  byte LButton;
  byte RButton;
  byte L1;
  byte L2;
  byte L3;
  byte R1;
  byte R2;
  byte R3;
  byte Step;
};
Data_Package data; //Create a variable with the above structure

//Parsing
String dataInTeta;
String arrayDataTeta[10];
boolean parsingTeta = false;
boolean receiveTeta = false;
float dataGyro, yawTarget = 0;

//PID Gerak Robot
//Variable PID Maju Mundur
float PIDValueT = 0, gainValueT = 0;
double errorT, previouserrorT;
double P_T = 0, I_T = 0, D_T = 0;
float KpT = 3;    //Kp
float KiT = 0;    //Ki
float KdT = 1;      //Kd

int proxySensor1 = A4;

int stateSensor1;

int relay1 = A0;
int relay2 = A1;
int relay3 = A2;
int relay4 = A3;

int loc1 = 0;
int loc2 = 0;
int loc3 = 0;
int loc4 = 0;

int stateAuto = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // Configuracion del puerto serial de comunicacion con el ESCLAVO 1
  Serial2.begin(115200); // Configuracion del puerto serial de comunicacion con el ESCLAVO 2
  Serial3.begin(115200);
  Serial4.begin(115200);

  pinMode(proxySensor1, INPUT);

  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening(); //  Set the module as receiver
  resetData();

  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  digitalWrite(relay3, HIGH);
  digitalWrite(relay4, HIGH);

  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);

  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);

  pinMode(TRIG_PIN3, OUTPUT);
  pinMode(ECHO_PIN3, INPUT);

  pinMode(TRIG_PIN4, OUTPUT);
  pinMode(ECHO_PIN4, INPUT);

  prevMillis = millis();
}

void loop() {
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
  }

  currentMillis = millis();
  if (currentMillis - prevMillis >= 100) {
    prevMillis = currentMillis;

    //Pembacaan dan perhitungan sensor PING
    //Sensor Depan (1)
    digitalWrite(TRIG_PIN1, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN1, LOW);
    duration1 = pulseIn(ECHO_PIN1, HIGH, inDelay);
    distance1 = duration1 / 28 / 2 ;

    //Sensor Kanan (2)
    digitalWrite(TRIG_PIN2, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN2, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN2, LOW);
    duration2 = pulseIn(ECHO_PIN2, HIGH, inDelay);
    distance2 = duration2 / 28 / 2 ;

    //Sensor Belakang (3)
    digitalWrite(TRIG_PIN3, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN3, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN3, LOW);
    duration3 = pulseIn(ECHO_PIN3, HIGH, inDelay);
    distance3 = duration3 / 28 / 2 ;

    //Sensor Belakang (4)
    digitalWrite(TRIG_PIN4, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN4, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN4, LOW);
    duration4 = pulseIn(ECHO_PIN4, HIGH, inDelay);
    distance4 = duration4 / 28 / 2 ;
  }

  stateSensor1 = digitalRead(proxySensor1);
  printSpeed();
  cekDataGyro();
  PIDTeta();

  bacaRemote();
  sendSlave();


  if (stateAuto) {
    autoLocation();
  }
  else {
    rpm_x = (map(data.LX, 0, 255, -100, 100) * max_rpm) / devider;
    rpm_y = (map(data.LY, 0, 255, -100, 100) * max_rpm) / devider;
    rotateSpeed = (map(data.RX, 0, 255, -100, 100) * max_rpm) / (devider * 2);
  }
}

void bacaRemote() {
  //Cek adakah input dari controller
  if (data.LX != 128 || data.LY != 128 || data.RX != 128 && stateAuto == 0) {
    //diagonal
    if ((data.LY != 128 || data.LX != 128) && data.RX == 128) {
      moveMotor();
    }

    //putar kiri
    else if (data.LY == 128 && data.LX == 128 && data.RX != 128) {
      rotateMotor();
      yawTarget = dataGyro;
    }
  }

  //Bagian Stop Motor
  if (data.LY == 128 && data.LX == 128 && data.RX == 128 && stateAuto == 0) {
    if (errorT > 1 || errorT < -1) {
      fixMotor();
    }
    else {
      stopMotor();
    }
  }

  //Input Controller
  //Servo 7
  if (data.TButton && currentMillis - millisButton >= 200) {
    Serial.println("Triangle is pushed");
    stateServo7 = !stateServo7;
    millisButton = millis();
  }

  if (stateServo7) {
    dataServo7 = default3O;
  }
  else {
    dataServo7 = default3C;
  }

  //Servo 6
  if (data.SButton && currentMillis - millisButton >= 200) {
    Serial.println("Square is pushed");
    stateServo6 = !stateServo6;
    millisButton = millis();
  }

  if (stateServo6) {
    dataServo6 = default3O;
  }
  else {
    dataServo6 = default3C;
  }

  //Servo 8
  if (data.CButton && currentMillis - millisButton >= 200) {
    Serial.println("Circle is pushed");
    stateServo8 = !stateServo8;
    millisButton = millis();
  }

  if (stateServo8) {
    dataServo8 = default3O;
  }
  else {
    dataServo8 = default3C;
  }

  //Servo 1 - 3
  if (data.XButton && currentMillis - millisButton >= 200) {
    Serial.println("X is pushed");
    stateServo13 = !stateServo13;
    millisButton = millis();
  }

  if (stateServo13) {
    dataServo13 = defaultXO;
  }
  else {
    dataServo13 = defaultXC;
  }

  if (data.UButton) {
    Serial.println("Up Button is pressed");
    digitalWrite(relay3, LOW);
  }
  else {
    //Serial.println("Up Button is not pressed");
    digitalWrite(relay3, HIGH);
  }
  if (data.LButton) {
    Serial.println("Left Button is pressed");
    digitalWrite(relay4, LOW);
  }
  else {
    //Serial.println("Left Button is not pressed");
    digitalWrite(relay4, HIGH);
  }
  if (data.RButton) {
    Serial.println("Right Button is pressed");
    digitalWrite(relay2, LOW);
  }
  else {
    //Serial.println("Right Button is not pressed");
    digitalWrite(relay2, HIGH);
  }


  //Lengan
  if (data.DButton && currentMillis - millisButton >= 200) {
    Serial.println("Down Button is pressed");
    stateLengan = !stateLengan;
    millisButton = millis();
  }
  if (stateLengan) {
    dataLengan = -150;
  }
  else {
    if (stateSensor1) {
      dataLengan = 185;
    }
    else {
      dataLengan = 5;
    }
  }

  if (data.L1) {
    Serial.println("L1 is pushed");
    devider = 400;
  }

  if (!data.L1) {
    devider = 100;
  }

  if (data.L2) {
    Serial.println("L2 is pushed");
  }
  if (data.L3) {
    Serial.println("L3 is pushed");
  }
  if (data.R1) {
    stateAuto = 1;
  }
  else {
    stateAuto = 0;
  }
  if (data.R2) {
    Serial.println("R2 is pushed");
  }
  if (data.R3) {
    Serial.println("R3 is pushed");
  }

  if (data.R3 && currentMillis - millisButton >= 500) {
    Serial.println("R3 is pushed");
    stateServo45 = !stateServo45;
    millisButton = millis();
  }

  if (stateServo45) {
    dataServo4 = 0;
    dataServo5 = 110;
  }
  else {
    dataServo4 = 60;
    dataServo5 = 70;
  }
}

void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.LX = 128;
  data.LY = 128;
  data.RX = 128;
  data.RY = 128;
  data.XButton = 0;
  data.TButton = 0;
  data.SButton = 0;
  data.CButton = 0;
  data.UButton = 0;
  data.DButton = 0;
  data.LButton = 0;
  data.RButton = 0;
  data.L1 = 0;
  data.L2 = 0;
  data.L3 = 0;
  data.R1 = 0;
  data.R2 = 0;
  data.R3 = 0;
}

void PIDTeta() {

  errorT = yawTarget - dataGyro;
  P_T = errorT;
  I_T = I_T + errorT;
  D_T = errorT - previouserrorT;

  if (I_T > 10) {
    I_T = 10;
  }
  if (I_T < (-10)) {
    I_T = (-10);
  }

  PIDValueT = (KpT * P_T) + (KiT * I_T) + (KdT * D_T);
  previouserrorT = errorT;
  gainValueT = PIDValueT;

  if (gainValueT >= 25) {
    gainValueT = 25;
  }

  if (gainValueT <= (-25)) {
    gainValueT = (-25);
  }
}

void sendData() {
  Serial2.print("*");
  Serial2.print(rpm_1);
  Serial2.print(",");
  Serial2.print(rpm_2);
  Serial2.print(",");
  Serial2.print(rpm_3);
  Serial2.print(",");
  Serial2.print(rpm_4);
  Serial2.print("#");
}

void sendSlave() {
  Serial3.print("*");
  Serial3.print(dataServo13);
  Serial3.print(",");
  Serial3.print(dataServo4);
  Serial3.print(",");
  Serial3.print(dataServo5);
  Serial3.print(",");
  Serial3.print(dataServo6);
  Serial3.print(",");
  Serial3.print(dataServo7);
  Serial3.print(",");
  Serial3.print(dataServo8);
  Serial3.print("#");

  Serial4.print("*");
  Serial4.print(dataLengan);
  Serial4.println("#");
}

void rotateMotor() {
  rpm_1 = -rotateSpeed;
  rpm_2 = -rotateSpeed;
  rpm_3 = rotateSpeed;
  rpm_4 = rotateSpeed;
  sendData();
}

void fixMotor() {
  rpm_1 = -gainValueT;
  rpm_2 = -gainValueT;
  rpm_3 = gainValueT;
  rpm_4 = gainValueT;
  sendData();
}

void moveMotor() {
  rpm_1 = rpm_y - rpm_x - gainValueT;
  rpm_2 = rpm_y + rpm_x - gainValueT;
  rpm_3 = rpm_y + rpm_x + gainValueT;
  rpm_4 = rpm_y - rpm_x + gainValueT;
  sendData();
}

void stopMotor() {
  rpm_1 = 0;
  rpm_2 = 0;
  rpm_3 = 0;
  rpm_4 = 0;
  sendData();
}

void cekDataGyro() {
  while (Serial1.available() > 0) {
    char inCharTeta = (char)Serial1.read();
    if (inCharTeta == '*') {
      dataInTeta = "";
      receiveTeta = true;
    }

    if (receiveTeta) {
      dataInTeta += inCharTeta;
      if (inCharTeta == '#') {
        parsingTeta = true;
      }
    }

    if (parsingTeta) {
      parsingDataTeta();
      parsingTeta = false;
      receiveTeta = false;
    }
  }
}

void parsingDataTeta() {
  int j = 0;
  arrayDataTeta[j] = "";

  for (int i = 1; i < dataInTeta.length(); i++) {
    if ((dataInTeta[i] == '#') || (dataInTeta[i] == ',')) {
      j++;
      arrayDataTeta[j] = "";
    }
    else {
      arrayDataTeta[j] = arrayDataTeta[j] + dataInTeta[i];
    }
  }
  dataGyro = arrayDataTeta[0].toFloat();
}

void printSpeed() {
  Serial.print(distance1);
  Serial.print("=");
  Serial.print(loc1);
  Serial.print("   ");
  Serial.print(distance2);
  Serial.print("=");
  Serial.print(loc2);
  Serial.print("   ");
  Serial.print(distance3);
  Serial.print("=");
  Serial.print(loc3);
  Serial.print("   ");
  Serial.print(distance4);
  Serial.print("=");
  Serial.print(loc4);
  Serial.print("   ");
  Serial.print(rpm_y);
  Serial.print("    ");
  Serial.println(rpm_x);
}

void autoLocation() {
  switch (data.Step) {
    case 0:
      loc1 = distance1;
      loc2 = distance2;
      loc3 = distance3;
      loc4 = distance4;
      break;

    case 1:
      loc1 = 25;
      loc2 = 84;
      loc3 = 91;
      loc4 = distance4;
      break;

    case 2:
      loc1 = 10;
      loc2 = 32;
      loc3 = 105;
      loc4 = distance4;
      break;

    case 3:
      loc1 = 51;
      loc2 = 271;
      loc3 = 63;
      loc4 = 346;
      break;

    case 4:
      loc1 = distance1;
      loc2 = 345;
      loc3 = 60;
      loc4 = 273;
      break;

    case 5:
      loc1 = 8;
      loc2 = distance2;
      loc3 = 105;
      loc4 = 94;
      break;

    case 6:
      loc1 = 31;
      loc2 = distance2;
      loc3 = distance3;
      loc4 = 33;
      break;

    case 7:
      loc1 = distance1;
      loc2 = 372;
      loc3 = 87;
      loc4 = 246;
      break;

    case 8:
      loc1 = 27;
      loc2 = 292;
      loc3 = 87;
      loc4 = 325;
      break;
  }

  //Mundur
  if (distance1 < loc1 || distance3 > loc3) {
    a = abs(loc1 - distance1);
    b = abs(loc3 - distance3);

    if (a > b) {
      rpm_y = a + 15;
    }
    else if (a < b) {
      rpm_y = b + 15;
    }
    else {
      rpm_y = (a + b) / 2 + 15;
    }

    if (rpm_y > 50) {
      rpm_y = 50;
    }
  }


  //Maju
  if (distance1 > loc1 || distance3 < loc3) {
    a = abs(loc1 - distance1) * -1;
    b = abs(loc3 - distance3) * -1;

    if (a > b) {
      rpm_y = b - 15;
    }
    else if (a < b) {
      rpm_y = a - 15;
    }
    else {
      rpm_y = (a + b) / 2 - 15;
    }

    if (rpm_y < -50) {
      rpm_y = -50;
    }
  }

  //Move Y
  if ((distance1 < loc1 + 3 && distance1 > loc1 - 3)  && (distance3 < loc3 + 3 && distance3 > loc3 - 3)) {
    rpm_y = 0;
  }

  //Kanan
  if (distance2 > loc2 || distance4 < loc4) {
    c = abs(loc2 - distance2);
    d = abs(loc4 - distance4);

    if (c > d) {
      rpm_x = c + 15;
    }
    else if (c < d) {
      rpm_x = d + 15;
    }
    else {
      rpm_x = (c + d) / 2 + 15;
    }

    if (rpm_x > 50) {
      rpm_x = 50;
    }
  }

  //Kiri
  if (distance2 < loc2 || distance4 > loc4) {
    c = abs(loc2 - distance2) * -1;
    d = abs(loc4 - distance4) * -1;

    if (c > d) {
      rpm_x = d - 15;
    }
    else if (c < d) {
      rpm_x = c - 15;
    }
    else {
      rpm_x = (c + d) / 2 - 15;
    }

    if (rpm_x < -50) {
      rpm_x = -50;
    }
  }

  //Move X
  if ((distance2 < loc2 + 3 && distance2 > loc2 - 3)  && (distance4 < loc4 + 3 && distance4 > loc4 - 3)) {
    rpm_x = 0;
  }

  moveMotor();
}
