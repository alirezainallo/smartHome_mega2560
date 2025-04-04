#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define bilink_pin 13 //PB7
void blink_loop(uint32_t ms);
LiquidCrystal_I2C lcd(0x27,  20, 4);

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