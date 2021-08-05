//Library
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>

//NRF24 Variable Definition
RF24 radio(A10, A11);   // nRF24L01 (CE, CSN)
const byte address[6] = "111111";
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
Data_Package data;

//Parsing Data
String dataInTeta;
String arrayDataTeta[10];
boolean parsingTeta = false;
boolean receiveTeta = false;
float dataGyro, yawTarget = 0;

//PID Gerak Robot
float PIDValueT = 0, gainValueT = 0;
double errorT, previouserrorT;
double P_T = 0, I_T = 0, D_T = 0;
float KpT = 8;    //Kp
float KiT = 0;    //Ki
float KdT = 0;    //Kd

//Pin Definition
//Servo
const int servo1Pin = A0; //Right Holder
const int servo2Pin = A1; //Left Holder
const int servo3Pin = A2; //Left Gripper
const int servo4Pin = A3; //Right Gripper
Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
//Relay
const int relay1 = 6; //Right Motor Relay
const int relay2 = 7; //Left Motor Relay
//Limit Switch
const int limit1 = 4; //Bottom Limit Switch
const int limit2 = 5; //Top Limit Switch
//Proxy Sensor
const int proxy = A4;

//Pin State
//Relay
int stateRelay1 = 0;
int stateRelay2 = 0;
//Limit Switch
int stateLimit1 = 0;
int stateLimit2 = 0;
//Servo
int stateServo1 = 0;
int stateServo2 = 0;
int stateServo3 = 0;
int stateServo4 = 0;
//Sensor
int stateSensor1 = 0;
//Slave Motor
int stateMotor1 = 0;

//Number Variable
int dataLengan = 0;
int devider = 0;
long long millisButton = 0, currentMillis = 0, previousMillis = 0;
int rpm_x, rpm_y, rotateSpeed;
int rpm_1, rpm_2, rpm_3;
int max_rpm = 100;


void setup() {
  //Serial
  Serial.begin(115200);
  Serial1.begin(115200); // Configuracion del puerto serial de comunicacion con el ESCLAVO 1
  Serial2.begin(115200); // Configuracion del puerto serial de comunicacion con el ESCLAVO 2
  Serial3.begin(9600);

  //Servo
  Servo1.attach(servo1Pin);
  Servo2.attach(servo2Pin);
  Servo3.attach(servo3Pin);
  Servo4.attach(servo4Pin);
  Servo1.write(0);
  Servo2.write(0);
  Servo3.write(0);
  Servo4.write(0);

  //Relay
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);

  //Limit Switch
  pinMode(limit1, INPUT_PULLUP);
  pinMode(limit2, INPUT_PULLUP);

  //Proxy
  pinMode(proxy, INPUT);

  //NRF24
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening(); //  Set the module as receiver
  resetData();

  //Timer
  previousMillis = millis();
}

void loop() {
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
  }

  //Timer every 100ms
  currentMillis = millis();
  if (currentMillis - previousMillis > 100) {
    previousMillis = currentMillis;
    sendDataMotor();
    printSpeed();
  }

  //Gyro Checking
  cekDataGyro();
  PIDTeta();

  //Read Sensor
  stateLimit1 = digitalRead(limit1);
  stateLimit2 = digitalRead(limit2);
  stateSensor1 = digitalRead(proxy);

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
    stateServo1 = !stateServo1;
    millisButton = millis();
  } if (stateServo1) {
    Servo1.write(0);
  } else {
    Servo1.write(100);
  }

  //Square]
  if (data.SButton && currentMillis - millisButton >= 1000) {
    Serial.println("Square is pushed");
    stateServo2 = !stateServo2;
    millisButton = millis();
  } if (stateServo2) {
    Servo2.write(0);
  } else {
    Servo2.write(100);
  }

  //Circle
  if (data.CButton && currentMillis - millisButton >= 1000) {
    Serial.println("Circle is pushed");
    stateServo3 = !stateServo3;
    millisButton = millis();
  } if (stateServo3) {
    Servo3.write(0);
  } else {
    Servo3.write(100);
  }

  //X
  if (data.XButton && currentMillis - millisButton >= 1000) {
    Serial.println("Circle is pushed");
    stateServo4 = !stateServo4;
    millisButton = millis();
  } if (stateServo4) {
    Servo4.write(0);
  } else {
    Servo4.write(100);
  }

  if (data.UButton && currentMillis - millisButton >= 1000) {
    //Serial.println("Circle is pushed");
    stateMotor1 = !stateMotor1;
    millisButton = millis();
  }
  if (stateMotor1 == true && stateLimit2 == HIGH) {
    if (stateSensor1) {
      dataLengan = 185;
    }
    else {
      dataLengan = 5;
    }
  }
  else if (stateMotor1 == false && stateLimit1 == HIGH) {
    dataLengan = -100;
  }
  else {
    dataLengan = 0;
  }

  //Left Motor Activation
  if (data.LButton) {
    Serial.println("Left Button is pressed");
    digitalWrite(relay2, LOW);
  } else {
    digitalWrite(relay2, HIGH);
  };

  //Right Motor Activation
  if (data.RButton) {
    Serial.println("Right Button is pressed");
    digitalWrite(relay1, LOW);
  } else {
    digitalWrite(relay1, HIGH);
  };


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
  Serial.print(stateLimit1);
  Serial.print(" ");
  Serial.print(stateLimit2);
  Serial.print(" ");
  Serial.println(stateSensor1);
}
