// SainSmart Instabots Upright Rover rev. 2.0
// http://www.sainsmart.com

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);



uint16_t Display_Counter;
uint16_t NUM_MODES = 4;

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

  delay(1500);
  bradDisplay();
  delay(1500);
}

void loop() {
  
  /**********************************************************************************************************/
  sendData();
  if (!receiveData()){
    return;
  };
  /**********************************************************************************************************/
  if (buttonPressed()){
    lcd.clear();
    Display_Counter = (Display_Counter + 1) % NUM_MODES;
    modeDisplay(Display_Counter);
    delay(300);
    lcd.clear();
  }

  while (digitalRead(2) == LOW) {
    delay(1);
  }

  
  switch (Display_Counter){
  case 2:
    bradDisplay();
    break;
  case 1:
    PID_Display();
    break;
  case 0:
    Gesture_Display();
    break;
  default:
    modeDisplay(Display_Counter);
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
  axis_x.axis_8 = Display_Counter;

  Mirf.setTADDR((byte *)"serv1");
  Mirf.send((byte *)&axis_x);
  while (Mirf.isSending()) {
  }
}

boolean receiveData(){
  unsigned long lastReceivedTime = millis();
  while (!Mirf.dataReady()) {
    if ((millis() - lastReceivedTime) > 2000) {
      lcd.setCursor(0, 0);
      lcd.print("   Waiting...   ");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      return false;
    }
  }
  Mirf.getData((byte *)&data);
  return true;
}

boolean buttonPressed(){
  digitalWrite(2, HIGH);
  int buttonDelay = 0;

  while (digitalRead(2) == LOW) {
    buttonDelay++;
    delay(1);
  }

  return buttonDelay > 10;
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
  lcdPrintNumberFixedWidth((float)data.D/100, 2, false); //convention to pass D*100 over wireless
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

void bradDisplay(){
  lcd.setCursor(0, 0);
  lcd.print("Modified by:    ");
  lcd.setCursor(0,1);
  lcd.print("Brad Saund      ");
}

void modeDisplay(uint16_t mode){
  lcd.setCursor(0, 0);
  lcd.print("Mode: ");
  lcd.print(mode);
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















