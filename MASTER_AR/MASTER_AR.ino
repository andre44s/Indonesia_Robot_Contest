#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>

RF24 radio(A10, A11);   // nRF24L01 (CE, CSN)
const byte address[6] = "101010";

//Variable Motor
int rpm_x, rpm_y, rotateSpeed;
int rpm_1, rpm_2, rpm_3;
int max_rpm = 100;

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
  byte state1;
  byte state2;
  byte state3;
  byte state4;
  byte state5;
  byte state6;
  byte state7;
  byte state8;
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
float KpT = 10;    //Kp
float KiT = 0;    //Ki
float KdT = 15;      //Kd

int servo1Pin = 10;
int servo2Pin = 11;
int servo3Pin = 12;
int servo4Pin = 13;
int servo5Pin = A4;

Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo5;

int relay1 = A0;
int relay2 = A1;
int relay3 = A2;
int relay4 = A3;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // Configuracion del puerto serial de comunicacion con el ESCLAVO 1
  Serial2.begin(115200); // Configuracion del puerto serial de comunicacion con el ESCLAVO 2

  Servo1.attach(servo1Pin);
  Servo2.attach(servo2Pin);
  Servo3.attach(servo3Pin);
  Servo4.attach(servo4Pin);
  Servo5.attach(servo5Pin);

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

  Servo1.write(30);
  Servo2.write(30);
  Servo3.write(30);
  Servo4.write(30);
  Servo5.write(30);

  digitalWrite(relay1,HIGH);
  digitalWrite(relay2,HIGH);
  digitalWrite(relay3,HIGH);
  digitalWrite(relay4,HIGH);
}

void loop() {
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
  }

  cekDataGyro();
  PIDTeta();
  printSpeed();

  bacaRemote();
  rpm_x = (map(data.LX, 0, 255, -100, 100) * max_rpm) / 100;
  rpm_y = (map(data.LY, 0, 255, -100, 100) * max_rpm) / 100;
  rotateSpeed = (map(data.RX, 0, 255, -100, 100) * max_rpm) / 200;
}

void bacaRemote() {
  //Cek adakah input dari controller
  if (data.LX != 128 || data.LY != 128 || data.RX != 128) {
    //diagonal
    if ((data.LY != 128 || data.LX != 128) && data.RX == 128) {
      Serial.print("Move : ");
      moveMotor();
    }

    //putar kiri
    else if (data.LY == 128 && data.LX == 128 && data.RX != 128) {
      Serial.print("Putar Kiri : ");
      Serial.println(rotateSpeed);
      rotateMotor();
      yawTarget = dataGyro;
    }
  }

  //Bagian Stop Motor
  if (data.LY == 128 && data.LX == 128 && data.RX == 128) {
    if (errorT > 1 || errorT < -1) {
      fixMotor();
    }
    else {
      stopMotor();
      Serial.println("Stop");
    }
  }

  //Input Controller
  if (data.TButton) {
    Serial.println("Triangle is pushed");
    Servo1.write(30);
    Servo2.write(30);
    Servo3.write(30);
    Servo4.write(30);
    Servo5.write(30);
  }
  if (data.SButton) {
    Serial.println("Square is pushed");
    digitalWrite(relay1, HIGH);
  }
  if (!data.SButton) {
    digitalWrite(relay1, LOW);
  }
  if (data.CButton) {
    Serial.println("Circle is pushed");
    Servo1.write(130);
    Servo2.write(130);
    Servo3.write(130);
    Servo4.write(130);
    Servo5.write(130);
  }
  if (data.XButton) {
    Serial.println("X is pushed");
  }
  if (data.UButton) {
    Serial.println("Up Button is pressed");
    digitalWrite(relay3, LOW);
  }
  else{
    //Serial.println("Up Button is not pressed");
    digitalWrite(relay3, HIGH);
  }
  if (data.LButton) {
    Serial.println("Left Button is pressed");
    digitalWrite(relay4, LOW);
  }
  else{
    //Serial.println("Left Button is not pressed");
    digitalWrite(relay4, HIGH);
  }
  if (data.RButton) {
    Serial.println("Right Button is pressed");
    digitalWrite(relay2, LOW);
  }
  else{
    //Serial.println("Right Button is not pressed");
    digitalWrite(relay2, HIGH);
  }
  if (data.DButton) {
    Serial.println("Down Button is pressed");
  }
  if (data.L1) {
    Serial.println("L1 is pushed");
  }
  if (data.L2) {
    Serial.println("L2 is pushed");
  }
  if (data.L3) {
    Serial.println("L3 is pushed");
  }
  if (data.R1) {
    Serial.println("R1 is pushed");
    digitalWrite(relay1, HIGH);
  }
  if (!data.R1) {
    digitalWrite(relay1, LOW);
  }
  if (data.R2) {
    Serial.println("R2 is pushed");
  }
  if (data.R3) {
    Serial.println("R3 is pushed");
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
  data.state1 = 0;
  data.state2 = 0;
  data.state3 = 0;
  data.state4 = 0;
  data.state5 = 0;
  data.state6 = 0;
  data.state7 = 0;
  data.state8 = 0;
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

  if (gainValueT >= 30) {
    gainValueT = 30;
  }

  if (gainValueT <= (-30)) {
    gainValueT = (-30);
  }
}

void sendData() {
  Serial2.print("*");
  Serial2.print(rpm_1);
  Serial2.print(",");
  Serial2.print(rpm_2);
  Serial2.print(",");
  Serial2.print(rpm_3);
  Serial2.print("#");
}

void rotateMotor() {
  rpm_1 = -rotateSpeed;
  rpm_2 = -rotateSpeed;
  rpm_3 = rotateSpeed;
  sendData();
}

void fixMotor() {
  rpm_1 = -gainValueT * 1.5;
  rpm_2 = -gainValueT * 1.5;
  rpm_3 = gainValueT * 1.5;
  sendData();
}

void moveMotor() {
  rpm_1 = rpm_y + rpm_x - gainValueT;
  rpm_3 = rpm_y + -rpm_x + gainValueT;
  rpm_2 = rpm_x + gainValueT;
  sendData();
}

void stopMotor() {
  rpm_1 = 0;
  rpm_2 = 0;
  rpm_3 = 0;
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
  Serial.print(data.LX);
  Serial.print(" ");
  Serial.print(data.LY);
  Serial.print(" ");
  Serial.print(data.RX);
  Serial.print(" ");
  Serial.print(data.RY);
  Serial.println(" ");
  Serial.flush();
}
