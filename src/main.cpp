#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <TM1637Display.h>
#include <Keypad.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SoftwareSerial.h>
#include "DFRobotDFPlayerMini.h"
#include <Servo.h>
#include "RTClib.h"

RTC_DS1307 rtc;

#define FIRE_SENSOR_PIN     45

#define FAN_PIN             53
#define WATER_POMP_FIRE_PIN 51
#define WATER_POMP_SOIL_PIN 43
#define LIGHT_PIN           47
#define LOCK_PIN            49
bool needFan = false;

Servo doorServo;

SoftwareSerial softSerial(/*rx =*/19, /*tx =*/18, true);
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

#define MQ9_PIN 15
#define LDR_PIN 16
#define SHM_PIN 17 // soil humadity

#define DHTPIN  14
#define DHTTYPE DHT11
DHT_Unified dht(DHTPIN, DHTTYPE);

const byte ROWS = 4; 
const byte COLS = 4; 
void keypad_process(void);
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {9, 8, 7, 6}; 
byte colPins[COLS] = {5, 4, 3, 2};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

typedef enum menu_tag{
  menu_idle = 0,
  menu_open,
  menu_cannot_open,
  menu_locking,
  menu_rtc,
  menu_rtc_setting,
}menu_t;
menu_t gMenu = menu_idle;

uint32_t lcd_initPage_next_tick_to_idle = 0;
bool idleOrLocking = false;
#define bilink_pin 13 //PB7
void blink_loop(uint32_t ms);
void lcd_loop(uint32_t ms);
void lcd_initPage(menu_t m);
void rtc_loop(void);
LiquidCrystal_I2C lcd(0x27,  20, 4);
TM1637Display disp7seg(11 /*DIO*/, 12 /*CLK*/);
// TM1637Display disp7seg(12 /*DIO*/, 11 /*CLK*/);
uint8_t data_7seg[] = {0xff, 0xff, 0xff, 0xff};
void sevSeg_printClock(uint8_t h, uint8_t m);
typedef struct time_tag {
  uint8_t hour;
  uint8_t min;
}time_t;
time_t gTime = {.hour = 11, .min = 57};
time_t tmpTime = {.hour = 0, .min = 0};;

uint32_t gPassword = 6831;

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
  lcd.setCursor(4,1);
  lcd.print("Starting...");
  delay(2000);
  lcd.clear();
  lcd_initPage(menu_idle);
  // rtc
  if(!rtc.begin()){
    Serial.println("Couldn't find RTC");
  }
  // 7seg
  disp7seg.setBrightness(7,true);
  disp7seg.clear();
  disp7seg.showNumberDecEx(1234);
  delay(2000);
  if(rtc.isrunning()){
    DateTime rtcData = rtc.now();
    sevSeg_printClock(rtcData.hour(), rtcData.minute());
  }else{
    DateTime rtcData(__DATE__, __TIME__);
    rtc.adjust(rtcData);
    sevSeg_printClock(rtcData.hour(), rtcData.minute());
  }
  // DHT
  dht.begin();
  // MQ9
  pinMode(MQ9_PIN, INPUT_PULLUP);
  // // dfplayer
  // softSerial.begin(9600);
  // if (!myDFPlayer.begin(softSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
  //   Serial.println(F("Unable to begin:"));
  //   Serial.println(F("1.Please recheck the connection!"));
  //   Serial.println(F("2.Please insert the SD card!"));
  //   while(true){
  //     delay(0);
  //   }
  // }
  // Serial.println(F("DFPlayer Mini online."));
  // myDFPlayer.volume(15);  //Set volume value. From 0 to 30
  // myDFPlayer.play(1);  //Play the first mp3
  // if (myDFPlayer.available()) {
  //   printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  // }

  // pins
  pinMode(FIRE_SENSOR_PIN, INPUT_PULLUP);

  pinMode(FAN_PIN, OUTPUT);
  pinMode(WATER_POMP_FIRE_PIN, OUTPUT);
  pinMode(WATER_POMP_SOIL_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(LOCK_PIN, OUTPUT);

  digitalWrite(FAN_PIN, LOW);
  digitalWrite(WATER_POMP_FIRE_PIN, LOW);
  digitalWrite(WATER_POMP_SOIL_PIN, LOW);
  digitalWrite(LIGHT_PIN, LOW);
  digitalWrite(LOCK_PIN, LOW);

  // Servo
  doorServo.attach(A0);
  doorServo.write(0);
  // delay(100);
  // doorServo.write(0);
  // delay(100);
  // doorServo.write(45);
  // delay(100);
  // doorServo.write(90);
  // delay(100);
  // doorServo.write(135);
  // delay(100);
  // doorServo.write(180);
  // delay(100);
  // doorServo.write(135);
  // delay(100);
  // doorServo.write(90);
  // delay(100);
  // doorServo.write(45);
  // delay(100);
  // doorServo.write(0);
  // delay(100);

}

void loop() {
  // put your main code here, to run repeatedly:
  blink_loop(500);
  lcd_loop(5000);
  rtc_loop();
  keypad_process();
}

// put function definitions here:
uint8_t sevSeg_buf[4] = {0};
void sevSeg_printClock(uint8_t h, uint8_t m){
  // sevSeg_num = h * 100 + m;
  // disp7seg.showNumberDecEx(sevSeg_num, true, true);
  
  gTime.hour = h;
  gTime.min  = m;

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
void rtc_loop(void){
  static uint32_t currTick = 0;
  static uint32_t nextTick = 0;
  static bool currStat = false;
  currTick = millis();
  if(nextTick < currTick){
    nextTick = currTick + 500;

    DateTime rtcData = rtc.now();
    sevSeg_printClock(rtcData.hour(), rtcData.minute());
    currStat = !currStat;
    if(currStat){
      // Serial.printf("%02d:%02d\n", rtcData.hour(), rtcData.minute());
      Serial.print(rtcData.hour());
      Serial.print(":");
      Serial.println(rtcData.minute());
    }
  }
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
    
    needFan = false;
    
    // MQ9
    static bool mq9_preState = false;
    bool mq9_curState = digitalRead(MQ9_PIN);
    if(mq9_curState != mq9_preState){
      mq9_preState = mq9_curState;
      if(mq9_curState){
        Serial.println("MQ9 Detected!!!");
        // digitalWrite(FAN_PIN, HIGH);
        needFan = true;
      }else{
        Serial.println("MQ9 Solved");
        // digitalWrite(FAN_PIN, LOW);
      }
    }
    
    // fire
    static bool fire_preState = false;
    bool fire_curState = digitalRead(FIRE_SENSOR_PIN);
    if(fire_curState != fire_preState){
      fire_preState = fire_curState;
      if(fire_curState){
        Serial.println("Fire Detected!!!");
        digitalWrite(WATER_POMP_FIRE_PIN, HIGH);
      }else{
        Serial.println("Fire Solved");
        digitalWrite(WATER_POMP_FIRE_PIN, LOW);
      }
    }
    
    // LDR
    static bool ldr_preState = true;
    bool ldr_curState = digitalRead(LDR_PIN);
    if(ldr_curState != ldr_preState){
      ldr_preState = ldr_curState;
      if(ldr_curState){
        Serial.println("Light Low");
        digitalWrite(LIGHT_PIN, HIGH);
      }else{
        Serial.println("Light Ok");
        digitalWrite(LIGHT_PIN, LOW);
      }
    }

    // SHM
    static bool shm_preState = true;
    bool shm_curState = digitalRead(SHM_PIN);
    if(shm_curState != shm_preState){
      shm_preState = shm_curState;
      if(shm_curState){
        Serial.println("Soil Humadity Low");
        digitalWrite(WATER_POMP_SOIL_PIN, HIGH);
      }else{
        Serial.println("Soil Humadity High");
        digitalWrite(WATER_POMP_SOIL_PIN, LOW);
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

      if(event.temperature > 28){
        // digitalWrite(FAN_PIN, HIGH);
        needFan = true;
      }else{
        // digitalWrite(FAN_PIN, LOW);
      }
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


    // fan control
    if(needFan){
      digitalWrite(FAN_PIN, HIGH);
    }else{
      digitalWrite(FAN_PIN, LOW);
    }
  }
}
void lcd_loop(uint32_t ms){
  static uint32_t currTick = 0;
  static uint32_t nextTick = 0;
  static bool currStat = false;
  currTick = millis();
  if(nextTick < currTick){
    nextTick = currTick + ms;
    
    lcd.setCursor(0,3);
    lcd.print("                    ");
    
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

      lcd.setCursor(0,3);
      lcd.print(event.temperature);
      lcd.print(((char)178));
      lcd.print("C");
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

      lcd.setCursor(14,3);
      lcd.print(event.relative_humidity);
      lcd.print(F("%"));
    }
  }

  if(lcd_initPage_next_tick_to_idle != 0){
    if(lcd_initPage_next_tick_to_idle < currTick){
      lcd_initPage_next_tick_to_idle = 0;
      if(idleOrLocking){
        lcd_initPage(menu_locking);
      }else{
        lcd_initPage(menu_idle);
      }
    }
  }
}

void lcd_initPage(menu_t m){
  gMenu = m;
  switch (m)
  {
  case menu_idle:
    // lcd.blink_off();

    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("                    ");

    lcd.setCursor(2,0);
    lcd.print("Enter your pass");
    doorServo.write(0);
    break;
  case menu_open:
    lcd_initPage_next_tick_to_idle = millis() + 2000;
    idleOrLocking = true;
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(2,1);
    lcd.print("pass was correct");
    digitalWrite(LOCK_PIN, HIGH);
    delay(300);
    doorServo.write(90);
    break;
  case menu_cannot_open:
    lcd_initPage_next_tick_to_idle = millis() + 2000;
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(1,1);
    lcd.print("pass was incorrect");
    break;
  case menu_locking:
    lcd_initPage_next_tick_to_idle = millis() + 2000;
    idleOrLocking = false;
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(5,1);
    lcd.print("Locking...");
    digitalWrite(LOCK_PIN, LOW);
    delay(200);
    break;
  case menu_rtc:
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(6,0);
    lcd.print("Set time:");
    lcd.setCursor(8,1);
    tmpTime = gTime;
    lcd.print(tmpTime.hour);lcd.print(":");lcd.print(tmpTime.min);
    
    // lcd.setCursor(9,1);
    // lcd.blink_on();
    break;
  case menu_rtc_setting:
    lcd_initPage_next_tick_to_idle = millis() + 2000;
    lcd.setCursor(0,0);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(1,1);
    lcd.print("RTC set correctly.");
    break;
  default:
    break;
  }
}
typedef enum timeSetup_tag {
  timeSetup_hour = 0,
  timeSetup_min,
}timeSetup_t;
timeSetup_t timeSetupPos = timeSetup_hour;
void keypad_process(void){
  static uint32_t kNum = 0;
  uint8_t customKeyNum = 0;
  uint32_t tmpNum1 = 0;
  uint32_t tmpNum2 = 0;
  char customKey = customKeypad.getKey();
  if (customKey){
    switch (customKey)
    {
    case 'A':
      // clear
      if(gMenu == menu_idle){
        kNum = 0;
        lcd.setCursor(0,1);
        lcd.print("                    ");
      }else if(gMenu == menu_rtc){
        if(timeSetupPos == timeSetup_hour){
          kNum = 0;
          tmpTime.hour = 0;
          lcd.setCursor(8,1);
          lcd.print("  ");
          lcd.setCursor(8,1);
          lcd.print(tmpTime.hour);
        }else if(timeSetupPos == timeSetup_min){
          kNum = 0;
          tmpTime.min = 0;
          lcd.setCursor(11,1);
          lcd.print("  ");
          lcd.setCursor(11,1);
          lcd.print(tmpTime.min);
        }
      }
      break;
    case 'B':
      // backspace
      tmpNum1 = kNum / 10;
      if(gMenu == menu_idle){
        if(tmpNum1 >= 0){
          kNum = tmpNum1;
          lcd.setCursor(0,1);
          lcd.print("                    ");
          if(kNum == 0){
          }else if(kNum<10){
            lcd.setCursor(10,1);
            lcd.print(kNum);
          }else if(kNum<100){
            lcd.setCursor(9,1);
            lcd.print(kNum);
          }else if(kNum<1000){
            lcd.setCursor(8,1);
            lcd.print(kNum);
          }else if(kNum<10000){
            lcd.setCursor(7,1);
            lcd.print(kNum);
          }
        }
      }else if(gMenu == menu_rtc){
      }
      break;
    case 'C':
      // rtc setup
      if(gMenu == menu_idle){
        lcd_initPage(menu_rtc);
      }else if(gMenu == menu_rtc){
        lcd_initPage(menu_idle);
      }
      break;
    case 'D':
      // Check pass
      if(gMenu == menu_idle){
        if(kNum == gPassword){
          lcd_initPage(menu_open);
        }else{
          lcd_initPage(menu_cannot_open);
        }
        kNum = 0;
      }else if(gMenu == menu_rtc){
        gTime = tmpTime;
        DateTime rtcData(2025, 4, 6, gTime.hour, gTime.min, 0);
        rtc.adjust(rtcData);
        sevSeg_printClock(gTime.hour, gTime.min);
        lcd_initPage(menu_rtc_setting);
      }
      break;
    case '*':
      break;
    case '#':
      break;
    default:
      customKeyNum = customKey - '0';
      tmpNum1 = kNum * 10 + customKeyNum;
      if(gMenu == menu_idle){
        if(tmpNum1 < 10000){
          kNum = tmpNum1;
          lcd.setCursor(0,1);
          lcd.print("                    ");
          if(kNum<10){
            lcd.setCursor(10,1);
          }else if(kNum<100){
            lcd.setCursor(9,1);
          }else if(kNum<1000){
            lcd.setCursor(8,1);
          }else if(kNum<10000){
            lcd.setCursor(7,1);
          }
          lcd.print(kNum);
        }
      }else if(gMenu == menu_rtc){
        if(timeSetupPos == timeSetup_hour){
          if(tmpNum1 < 25){
            kNum = tmpNum1;
            tmpTime.hour = kNum;
            lcd.setCursor(8,1);
            lcd.print("  ");
            lcd.setCursor(8,1);
            lcd.print(tmpTime.hour);
          }else{
            timeSetupPos = timeSetup_min;
            kNum = 0;
          }
        }else if(timeSetupPos == timeSetup_min){
          if(tmpNum1 < 61){
            kNum = tmpNum1;
            tmpTime.min = kNum;
            lcd.setCursor(11,1);
            lcd.print("  ");
            lcd.setCursor(11,1);
            lcd.print(tmpTime.min);
          }else{
            timeSetupPos = timeSetup_hour;
            kNum = 0;
          }
        }
      }
      break;
    }
  }
}
void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("DFPlayer Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("DFPlayer Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("DFPlayer Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("DFPlayer Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("DFPlayer Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("DFPlayer USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("DFPlayer USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("DFPlayer Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
  
}
