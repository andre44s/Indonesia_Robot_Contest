#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial Serial4(2, 3); //rx tx

RF24 radio(A10, A11);   // nRF24L01 (CE, CSN)
const byte address[6] = "000111"; // Address

//Variable Motor
int rpm_x, rpm_y, rotateSpeed;
int rpm_1, rpm_2, rpm_3, rpm_4;
int max_rpm = 100;

int stateServo15  = false;
int stateServo6  = false;
int stateServo7 = false;
int stateServo8 = false;

int dataServo15 = 30;
int dataServo6 = 30;
int dataServo7 = 30;
int dataServo8 = 30;

int defaultXO = 0;
int defaultXC = 70;
int default3O = 0;
int default3C = 120;

int stateLengan = 0;
int dataLengan = 0;

long long millisButton = 0, currentMillis = 0;

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
float KpT = 10;    //Kp
float KiT = 0;    //Ki
float KdT = 15;      //Kd

int proxySensor1 = 4;
int proxySensor2 = 5;
int proxySensor3 = 6;
int proxySensor4 = 7;

int stateSensor1;
int stateSensor2;
int stateSensor3;
int stateSensor4;

int relay1 = A0;
int relay2 = A1;
int relay3 = A2;
int relay4 = A3;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // Configuracion del puerto serial de comunicacion con el ESCLAVO 1
  Serial2.begin(115200); // Configuracion del puerto serial de comunicacion con el ESCLAVO 2
  Serial3.begin(115200);
  Serial4.begin(115200);

  pinMode(proxySensor1, INPUT);
  pinMode(proxySensor2, INPUT);
  pinMode(proxySensor3, INPUT);
  pinMode(proxySensor4, INPUT);

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
}

void loop() {
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
  }

  currentMillis = millis();

  stateSensor1 = digitalRead(proxySensor1);
  stateSensor2 = digitalRead(proxySensor2);
  stateSensor3 = digitalRead(proxySensor3);
  stateSensor4 = digitalRead(proxySensor4);

  stateSensor1 = HIGH;
  stateSensor2 = HIGH;
  stateSensor3 = HIGH;
  stateSensor4 = HIGH;

  cekDataGyro();
  PIDTeta();
  printSpeed();

  bacaRemote();
  sendSlave();
  rpm_x = (map(data.LX, 0, 255, -100, 100) * max_rpm) / 100;
  rpm_y = (map(data.LY, 0, 255, -100, 100) * max_rpm) / 100;
  rotateSpeed = (map(data.RX, 0, 255, -100, 100) * max_rpm) / 200;
}

void bacaRemote() {
  //Cek adakah input dari controller
  if (data.LX != 128 || data.LY != 128 || data.RX != 128) {
    //diagonal
    if ((data.LY != 128 || data.LX != 128) && data.RX == 128) {
      if (data.LY > 128 && stateSensor3 == LOW) {
        data.LY = 128;
      }
      if (data.LY < 128 && stateSensor1 == LOW) {
        data.LY = 128;
      }
      if (data.LX > 128 && stateSensor2 == LOW) {
        data.LX = 128;
      }
      if (data.LX < 128 && stateSensor4 == LOW) {
        data.LX = 128;
      }
      moveMotor();
    }

    //putar kiri
    else if (data.LY == 128 && data.LX == 128 && data.RX != 128) {
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

  //Servo 1 - 5
  if (data.XButton && currentMillis - millisButton >= 200) {
    Serial.println("X is pushed");
    stateServo15 = !stateServo15;
    millisButton = millis();
  }

  if (stateServo15) {
    dataServo15 = defaultXO;
  }
  else {
    dataServo15 = defaultXC;
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
    dataLengan = -256;
  }
  else {
    dataLengan = 150;
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
  Serial2.print(",");
  Serial2.print(rpm_4);
  Serial2.print("#");
}

void sendSlave() {
  Serial3.print("*");
  Serial3.print(dataServo15);
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
  rpm_1 = -gainValueT * 1.5;
  rpm_2 = -gainValueT * 1.5;
  rpm_3 = gainValueT * 1.5;
  rpm_4 = gainValueT * 1.5;
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
  Serial.print(dataLengan);
  Serial.print(" ");
  Serial.print(dataServo15);
  Serial.print(" ");
  Serial.print(dataServo6);
  Serial.print(" ");
  Serial.print(dataServo7);
  Serial.print(" ");
  Serial.println(dataServo8);
}
