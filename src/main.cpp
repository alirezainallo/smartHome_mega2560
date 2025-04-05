#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <TM1637Display.h>
#include <Keypad.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define MQ9_PIN 15
#define LDR_PIN 16
#define SHM_PIN 17 // soil humadity

#define DHTPIN  14
#define DHTTYPE DHT11
DHT_Unified dht(DHTPIN, DHTTYPE);

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
  // blink pin
  pinMode(bilink_pin, OUTPUT);
  // Serial.print(bilink_pin);
  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Hello dear");
  delay(2000);
  lcd.clear();
  // 7seg
  disp7seg.setBrightness(7,true);
  // disp7seg.showNumberDecEx(1234);
  sevSeg_printClock(11, 57);
  // DHT
  dht.begin();
  // MQ9
  pinMode(MQ9_PIN, INPUT_PULLUP);
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
    
    // blink
    digitalWrite(bilink_pin, currStat);
    currStat = !currStat;
    
    // MQ9
    static bool mq9_preState = false;
    bool mq9_curState = digitalRead(MQ9_PIN);
    if(mq9_curState != mq9_preState){
      mq9_preState = mq9_curState;
      if(mq9_curState){
        Serial.println("MQ9 Detected!!!");
      }else{
        Serial.println("MQ9 Solved");
      }
    }
    
    // LDR
    static bool ldr_preState = true;
    bool ldr_curState = digitalRead(LDR_PIN);
    if(ldr_curState != ldr_preState){
      ldr_preState = ldr_curState;
      if(ldr_curState){
        Serial.println("Light Low");
      }else{
        Serial.println("Light Ok");
      }
    }

    // SHM
    static bool shm_preState = true;
    bool shm_curState = digitalRead(SHM_PIN);
    if(shm_curState != shm_preState){
      shm_preState = shm_curState;
      if(shm_curState){
        Serial.println("Soil Humadity Low");
      }else{
        Serial.println("Soil Humadity High");
      }
    }

    // 7 seg dot
    if(currStat){
      sevSeg_buf[1] &= ~(1<<7);
    }else{
      sevSeg_buf[1] |=  (1<<7);
    }
    disp7seg.setSegments(sevSeg_buf, 4, 0);

    // dht11
    bool dht_debug_sw = false;
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      if(dht_debug_sw) Serial.println(F("Error reading temperature!"));
    }
    else {
      if(dht_debug_sw) Serial.print(F("Temperature: "));
      if(dht_debug_sw) Serial.print(event.temperature);
      if(dht_debug_sw) Serial.println(F("°C"));
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      if(dht_debug_sw) Serial.println(F("Error reading humidity!"));
    }
    else {
      if(dht_debug_sw) Serial.print(F("Humidity: "));
      if(dht_debug_sw) Serial.print(event.relative_humidity);
      if(dht_debug_sw) Serial.println(F("%"));
    }
  }
}
