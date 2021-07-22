#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <PS2X_lib.h>
#include <LiquidCrystal_I2C.h>

//Remote Lama 0x27
RF24 radio(7, 8);   // nRF24L01 (CE, CSN)
const byte address[6] = "000111"; // Address

PS2X ps2x;
LiquidCrystal_I2C lcd(0x27, 16, 2); //0x27 atau 0x3F
char printBawah[15];
char printAtas[15];
char buf[3];

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

void setup() {
  Serial.begin(115200);

  ps2x.config_gamepad(2, 3, 4, 5, false, false); //GamePad(clock, command, attention, data, Pressures?, Rumble?)

  radio.begin();
  radio.openWritingPipe(address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);

  lcd.begin();   // iInit the LCD for 16 chars 2 lines
  lcd.backlight();   // Turn on the backligt (try lcd.noBaklight() to turn it off)

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
  data.Step = 1;
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

  data.DButton = ps2x.ButtonPressed(PSB_PAD_DOWN);
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

  data.R2 = ps2x.ButtonPressed(PSB_R2);
  if (data.R2) {
    Serial.println("R2");
    if (data.Step < 8) {
      data.Step = data.Step + 1;
    }
    else {
      data.Step = 1;
    }
  }

  data.R3 = ps2x.Button(PSB_R3);
  if (data.R3) {
    Serial.println("R3");
  }

  data.L1 = ps2x.Button(PSB_L1);
  if (data.L1) {
    Serial.println("L1");
  }

  data.L2 = ps2x.ButtonPressed(PSB_L2);
  if (data.L2) {
    Serial.println("L2");
    if (data.Step > 1) {
      data.Step = data.Step - 1;
    }
    else {
      data.Step = 8;
    }
  }

  data.L3 = ps2x.Button(PSB_L3);
  if (data.L3) {
    Serial.println("L3");
  }

  radio.write(&data, sizeof(Data_Package));

  sprintf(printBawah, "%03d %03d %03d %03d", data.LX, data.LY, data.RX, data.RY);
  lcd.setCursor(0, 1);
  lcd.print(printBawah);

  lcd.setCursor(0, 0);
  sprintf(printAtas, "JATAYU STEP = %d", data.Step);
  lcd.print(printAtas);


}
