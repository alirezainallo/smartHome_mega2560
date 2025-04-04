#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <TM1637Display.h>

#define bilink_pin 13 //PB7
void blink_loop(uint32_t ms);
LiquidCrystal_I2C lcd(0x27,  20, 4);
TM1637Display disp7seg(11 /*DIO*/, 12 /*CLK*/);
uint8_t data_7seg[] = {0xff, 0xff, 0xff, 0xff};
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting...");
  
  pinMode(bilink_pin, OUTPUT);
  Serial.print(bilink_pin);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Hello dear");
  delay(2000);
  lcd.clear();

  data_7seg[0]= disp7seg.encodeDigit(15);
  disp7seg.setSegments(data_7seg);
  disp7seg.setBrightness(7,true);
  // disp7seg.showNumberDec(1234);
}

void loop() {
  // put your main code here, to run repeatedly:
  blink_loop(500);
}

// put function definitions here:
void blink_loop(uint32_t ms){
  static uint32_t currTick = 0;
  static uint32_t nextTick = 0;
  static bool currStat = false;
  currTick = millis();
  if(nextTick < currTick){
    nextTick = currTick + ms;
    digitalWrite(bilink_pin, currStat);
    currStat = !currStat;
  }
}