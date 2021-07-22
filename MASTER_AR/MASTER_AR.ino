#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>

RF24 radio(A10, A11);   // nRF24L01 (CE, CSN)
const byte address[6] = "111111";

//Variable Motor
int rpm_x, rpm_y, rotateSpeed;
int rpm_1, rpm_2, rpm_3;
int max_rpm = 200;

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
float KpT = 8;    //Kp
float KiT = 0;    //Ki
float KdT = 0;      //Kd

int servo1Pin = A4;
Servo Servo1;

int relay1 = A0;
int relay2 = A1;
int relay3 = A2;

int limit1 = 6;
int limit2 = 7;

int stateRelay1 = 0;
int stateRelay2 = 0;
int stateRelay3 = 0;

int stateServo1 = 0;
int stateMotor1 = true;

int stateLimit1 = 0;
int stateLimit2 = 0;

int dataLengan = 0;

int devider = 0;

long long millisButton = 0, currentMillis = 0, previousMillis = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // Configuracion del puerto serial de comunicacion con el ESCLAVO 1
  Serial2.begin(115200); // Configuracion del puerto serial de comunicacion con el ESCLAVO 2
  Serial3.begin(9600);

  Servo1.attach(servo1Pin);

  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);

  pinMode(limit1, INPUT_PULLUP);
  pinMode(limit2, INPUT_PULLUP);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening(); //  Set the module as receiver
  resetData();

  previousMillis = millis();

  Servo1.write(0);

  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  digitalWrite(relay3, HIGH);
}

void loop() {
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
  }

  currentMillis = millis();
  if (currentMillis - previousMillis > 100) {
    previousMillis = currentMillis;
    sendDataMotor();
    printSpeed();
  }

  cekDataGyro();
  PIDTeta();

  stateLimit1 = digitalRead(limit1);
  stateLimit2 = digitalRead(limit2);

  bacaRemote();
  rpm_x = (map(data.LX, 0, 255, -100, 100) * max_rpm) / devider;
  rpm_y = (map(data.LY, 0, 255, -100, 100) * max_rpm) / devider;
  rotateSpeed = (map(data.RX, 0, 255, -100, 100) * max_rpm) / (devider * 2);

}

void bacaRemote() {
  //Cek adakah input dari controller
  if (data.LX != 128 || data.LY != 128 || data.RX != 128) {
    //diagonal
    if ((data.LY != 128 || data.LX != 128) && data.RX == 128) {
      //      Serial.print("Move : ");
      moveMotor();
    }

    //putar kiri
    else if (data.LY == 128 && data.LX == 128 && data.RX != 128) {
      //      Serial.print("Putar Kiri : ");
      //      Serial.println(rotateSpeed);
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
      //Serial.println("Stop");
    }
  }

  //Input Controller
  //Triangle
  if (data.TButton && currentMillis - millisButton >= 1000) {
    Serial.println("Triangle is pushed");
    stateRelay1 = !stateRelay1;
    millisButton = millis();
  }
  if (stateRelay1) {
    digitalWrite(relay1, HIGH);
    //Serial.println("RELAY 1 ON");
  }
  else {
    digitalWrite(relay1, LOW);
    //Serial.println("RELAY 1 OFF");
  }

  //Square
  if (data.SButton && currentMillis - millisButton >= 1000) {
    Serial.println("Square is pushed");
    stateRelay2 = !stateRelay2;
    millisButton = millis();
  }
  if (stateRelay2) {
    digitalWrite(relay2, HIGH);
    //Serial.println("RELAY 2 ON");
  }
  else {
    digitalWrite(relay2, LOW);
    //Serial.println("RELAY 2 OFF");
  }

  //Circle
  if (data.CButton && currentMillis - millisButton >= 1000) {
    Serial.println("Circle is pushed");
    stateRelay3 = !stateRelay3;
    millisButton = millis();
  }
  if (stateRelay3) {
    digitalWrite(relay3, HIGH);
    //Serial.println("RELAY 3 ON");
  }
  else {
    digitalWrite(relay3, LOW);
    //Serial.println("RELAY 3 OFF");
  }

  //Circle
  if (data.XButton && currentMillis - millisButton >= 1000) {
    Serial.println("Circle is pushed");
    stateServo1 = !stateServo1;
    millisButton = millis();
  }
  if (stateServo1) {
    Servo1.write(0);
    //Serial.println("SERVO 1 ON");
  }
  else {
    Servo1.write(100);
    //Serial.println("SERVO 1 OFF");
  }

  if (data.UButton && currentMillis - millisButton >= 1000) {
    //Serial.println("Circle is pushed");
    stateMotor1 = !stateMotor1;
    millisButton = millis();
  }
  if (stateMotor1 == true && stateLimit2 == HIGH) {
    dataLengan = 50;
    //Serial.println("Motor Up");
  }
  else if (stateMotor1 == false && stateLimit1 == HIGH) {
    dataLengan = -200;
    //Serial.println("Motor Down");
  }
  else {
    dataLengan = 0;
    //Serial.print("Motor Stop");
  }

  if (data.LButton) {
    Serial.println("Left Button is pressed");
  }
  if (data.RButton) {
    Serial.println("Right Button is pressed");
  }
  if (data.DButton) {
    Serial.println("Down Button is pressed");
  }

  if (data.L1) {
    Serial.println("L1 is pushed");
    devider = 250;
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
    Serial.println("R1 is pushed");
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
  Serial2.print("#");
}

void sendDataMotor() {
  Serial3.print("*");
  Serial3.print(dataLengan);
  Serial3.print("#");
}

void rotateMotor() {
  rpm_1 = rotateSpeed;
  rpm_2 = -rotateSpeed;
  rpm_3 = -rotateSpeed;
  sendData();
}

void fixMotor() {
  rpm_1 = gainValueT;
  rpm_2 = -gainValueT;
  rpm_3 = -gainValueT;
  sendData();
}

void moveMotor() {
  rpm_1 = rpm_y + rpm_x + gainValueT;
  rpm_3 = rpm_y + -rpm_x - gainValueT;
  rpm_2 = rpm_x - gainValueT;
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
  Serial.print(data.RX);
  Serial.print(" ");
  Serial.print(data.LY);
  Serial.print(" ");
  Serial.print(dataLengan);
  Serial.println(" ");
  Serial.flush();
}
