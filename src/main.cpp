#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <TM1637Display.h>
#include <Keypad.h>

const byte ROWS = 4; 
const byte COLS = 4; 

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {9, 8, 7, 6}; 
byte colPins[COLS] = {5, 4, 3, 2};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

#define bilink_pin 13 //PB7
void blink_loop(uint32_t ms);
LiquidCrystal_I2C lcd(0x27,  20, 4);
TM1637Display disp7seg(11 /*DIO*/, 12 /*CLK*/);
uint8_t data_7seg[] = {0xff, 0xff, 0xff, 0xff};
void sevSeg_printClock(uint8_t h, uint8_t m);

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

  disp7seg.setBrightness(7,true);
  // disp7seg.showNumberDecEx(1234);
  sevSeg_printClock(11, 57);
}

void loop() {
  // put your main code here, to run repeatedly:
  blink_loop(500);

  char customKey = customKeypad.getKey();
  if (customKey){
    Serial.println(customKey);
  }
}

// put function definitions here:
uint8_t sevSeg_buf[4] = {0};
void sevSeg_printClock(uint8_t h, uint8_t m){
  // sevSeg_num = h * 100 + m;
  // disp7seg.showNumberDecEx(sevSeg_num, true, true);

  sevSeg_buf[0] = h/10;
  sevSeg_buf[1] = h%10;
  sevSeg_buf[2] = m/10;
  sevSeg_buf[3] = m%10;
  bool isBefore = 1;
  for(uint8_t i = 0; i < 4; i++){
    if(sevSeg_buf[i] == 0 && isBefore){
      sevSeg_buf[i] = 0;
    }else{
      sevSeg_buf[i] = disp7seg.encodeDigit(sevSeg_buf[i]);
      isBefore = 0;
    }
  }
  disp7seg.setSegments(sevSeg_buf, 4, 0);
}
void blink_loop(uint32_t ms){
  static uint32_t currTick = 0;
  static uint32_t nextTick = 0;
  static bool currStat = false;
  currTick = millis();
  if(nextTick < currTick){
    nextTick = currTick + ms;
    digitalWrite(bilink_pin, currStat);
    currStat = !currStat;
    
    if(currStat){
      sevSeg_buf[1] &= ~(1<<7);
    }else{
      sevSeg_buf[1] |=  (1<<7);
    }
    disp7seg.setSegments(sevSeg_buf, 4, 0);
  }
}