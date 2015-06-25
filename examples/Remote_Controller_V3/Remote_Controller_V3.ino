// SainSmart Instabots Upright Rover rev. 2.0
// http://www.sainsmart.com

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned int Display_Counter, Button_Delay = 0;

struct Axis {
  uint16_t axis_1;
  uint16_t axis_2;
  uint16_t axis_3;
  uint16_t axis_4;
  uint16_t axis_5;
  uint16_t axis_6;
  uint16_t axis_7;
  uint16_t axis_8;
};
Axis axis_x;

struct Gesture {
  float angle;
  float omega;
  int speed;
  int P;
  int I;
  int D;
  uint16_t null_1;
  uint16_t null_2;
};
Gesture data;

void setup() {
  pinMode(2, INPUT);
  lcd.init();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("SainSmartProduct");
  lcd.setCursor(0, 1);
  lcd.print("UprightRover 3.0");

  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"clie1");
  Mirf.payload = 16;
  Mirf.config();

  delay(2000);
  Dummy_Display();
  delay(2000);
}

void loop() {
  
  /**********************************************************************************************************/
  sendData();
  receiveData();
  /**********************************************************************************************************/
  digitalWrite(2, HIGH);
  while (digitalRead(2) == LOW) {
    Button_Delay++;
    delay(1);
  }

  if (Button_Delay > 10) {
    lcd.clear();
    Display_Counter++;
  }
  Button_Delay = 0;

  while (digitalRead(2) == LOW) {
    delay(1);
  }

  switch (Display_Counter % 3){
  case 2:
    Dummy_Display();
    break;
  case 1:
    PID_Display();
    break;
  case 0:
  default:
    Gesture_Display();
    break;
  }
   
  /**********************************************************************************************************/
  /*Serial.print("Ping:");
  Serial.println((millis() - lastReceivedTime));*/
}

void sendData(){
  axis_x.axis_1 = analogRead(A0);
  axis_x.axis_2 = analogRead(A1);
  axis_x.axis_3 = analogRead(A2);
  axis_x.axis_4 = analogRead(A3);

  Mirf.setTADDR((byte *)"serv1");
  Mirf.send((byte *)&axis_x);
  while (Mirf.isSending()) {
  }
}

void receiveData(){
  unsigned long lastReceivedTime = millis();
  while (!Mirf.dataReady()) {
    if ((millis() - lastReceivedTime) > 2000) {
      lcd.setCursor(0, 0);
      lcd.print("   Waiting...   ");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      return;
    }
  }
  Mirf.getData((byte *)&data);
}



/******************* DISPLAYS *****************************************************************************/
/**********************************************************************************************************/
void PID_Display()
{
  lcd.setCursor(0, 0);
  lcd.print("Parameter:P=");
  lcdPrintNumberFixedWidth(data.P, 4, false);
  /**********************************************************************************************************/
  lcd.setCursor(0, 1);
  lcd.print(" I=");
  lcdPrintNumberFixedWidth(data.I, 4, false);
  /**********************************************************************************************************/
  lcd.print("  D=");
  lcdPrintNumberFixedWidth(data.D, 4, false);
}
/**********************************************************************************************************/
void Gesture_Display()
{
  
  lcd.setCursor(0, 0);
  lcd.print("Gesture:A=");
  lcdPrintNumberFixedWidth(data.angle, 2, true);
  /**********************************************************************************************************/
  lcd.setCursor(0, 1);
  lcd.print("O=");
  lcdPrintNumberFixedWidth(data.omega, 3, true);
  /**********************************************************************************************************/
  lcd.setCursor(9, 1);
  lcd.print(" S=");
  lcdPrintNumberFixedWidth(data.speed, 3, true);
}

void Dummy_Display(){
  lcd.setCursor(0, 0);
  lcd.print("Modified by:    ");
  lcd.setCursor(0,1);
  lcd.print("Brad Saund      ");
}

void lcdPrintNumberFixedWidth(int number, int numPlacesBeforeDecimal, boolean displayPlusMinus){
  printHelper((float)number, numPlacesBeforeDecimal, displayPlusMinus);
  lcd.print(abs(number));
}

void lcdPrintNumberFixedWidth(float number, int numPlacesBeforeDecimal, boolean displayPlusMinus){
  printHelper(number, numPlacesBeforeDecimal, displayPlusMinus);
  lcd.print(abs(number));
}

void printHelper(float number, int numPlacesBeforeDecimal, boolean displayPlusMinus){
  if (displayPlusMinus){
    number < 0 ? lcd.print("-") : lcd.print("+");
  }
  while (numPlacesBeforeDecimal > 1 && pow(10, numPlacesBeforeDecimal - 1) > abs(number)){
    lcd.print("0");
    numPlacesBeforeDecimal--;
  }
}















