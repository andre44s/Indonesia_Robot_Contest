#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <PS2X_lib.h>
#include <LiquidCrystal_I2C.h>

//Remote Lama 0x27 
RF24 radio(7, 8);   // nRF24L01 (CE, CSN)
const uint64_t  address = 0xF0F0F0F0F0F0F001L; // Address

PS2X ps2x;
char printBawah[15];
String clawPrint, kickerPrint, pneuPrint;

LiquidCrystal_I2C lcd(0x27, 16, 2); //0x27 atau 0x3F
char buf[3];

struct Data_Package {
  uint8_t LX;
  uint8_t LY;
  uint8_t RX;
  uint8_t RY;
  uint8_t XButton;
  uint8_t TButton;
  uint8_t SButton;
  uint8_t CButton;
  uint8_t UButton;
  uint8_t DButton;
  uint8_t LButton;
  uint8_t RButton;
  uint8_t L1;
  uint8_t L2;
  uint8_t L3;
  uint8_t R1;
  uint8_t R2;
  uint8_t R3;
};
Data_Package data; //Create a variable with the above structure

void setup() {
  Serial.begin(115200);

  ps2x.config_gamepad(2, 3, 4, 5, false, false); //GamePad(clock, command, attention, data, Pressures?, Rumble?)

  radio.begin();
  radio.openWritingPipe(address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);

    //lcd.begin();   // iInit the LCD for 16 chars 2 lines
  lcd.init();
  lcd.backlight();   // Turn on the backligt (try lcd.noBaklight() to turn it off)

  data.LX = 128;
  data.LY = 128;
  data.RX = 128;
  data.RY = 128;
  data.XButton = 0;
}

void loop() {
  ps2x.read_gamepad(false, 0);          //read controller and set large motor to spin at 'vibrate' speed
  data.LX = ps2x.Analog(PSS_LX);
  data.LY = ps2x.Analog(PSS_LY);
  data.RX = ps2x.Analog(PSS_RX);
  data.RY = ps2x.Analog(PSS_RY);

  if (data.LX != 128 || data.LY != 128 || data.RX != 128 || data.RY != 128) {
    Serial.print("ANALOG : ");
    Serial.print(data.LX);
    Serial.print(",");
    Serial.print(data.LY);
    Serial.print(",");
    Serial.print(data.RX);
    Serial.print(",");
    Serial.println(data.RY);
  }

  data.XButton = ps2x.ButtonPressed(PSB_BLUE);
  if (data.XButton) {
    Serial.println("X");
  }

  data.TButton = ps2x.ButtonPressed(PSB_GREEN);
  if (data.TButton) {
    Serial.println("Triangle");
  }

  data.SButton = ps2x.ButtonPressed(PSB_PINK);
  if (data.SButton) {
    Serial.println("Square");
  }

  data.CButton = ps2x.ButtonPressed(PSB_RED);
  if (data.CButton) {
    Serial.println("Circle");
  }

  data.UButton = ps2x.Button(PSB_PAD_UP);
  if (data.UButton) {
    Serial.println("Up");
  }

  data.DButton = ps2x.Button(PSB_PAD_DOWN);
  if (data.DButton) {
    Serial.println("Down");
  }

  data.LButton = ps2x.Button(PSB_PAD_LEFT);
  if (data.LButton) {
    Serial.println("Left");
  }

  data.RButton = ps2x.Button(PSB_PAD_RIGHT);
  if (data.RButton) {
    Serial.println("Right");
  }

  data.R1 = ps2x.Button(PSB_R1);
  if (data.R1) {
    Serial.println("R1");
  }
  
  data.R2 = ps2x.Button(PSB_R2);
  if (data.R2) {
    Serial.println("R2");
  }
  
  data.R3 = ps2x.Button(PSB_R3);
  if (data.R3) {
    Serial.println("R3");
  }
  
  data.L1 = ps2x.Button(PSB_L1);
  if (data.L1) {
    Serial.println("L1");
  }
  
  data.L2 = ps2x.Button(PSB_L2);
  if (data.L2) {
    Serial.println("L2");
  }
  
  data.L3 = ps2x.Button(PSB_L3);
  if (data.L3) {
    Serial.println("L3");
  }

  radio.write(&data, sizeof(Data_Package));

  lcd.setCursor(0, 0);
  lcd.print("STM32 JATAYU 2021");

  sprintf(printBawah, "%03d ", data.LX);
  lcd.setCursor(0, 1);
  lcd.print(printBawah);
  sprintf(printBawah, "%03d ", data.LY);
  lcd.setCursor(4, 1);
  lcd.print(printBawah);
  sprintf(printBawah, "%03d ", data.RX);
  lcd.setCursor(8, 1);
  lcd.print(printBawah);
  sprintf(printBawah, "%03d ", data.RY);
  lcd.setCursor(12, 1);
  lcd.print(printBawah);
}
