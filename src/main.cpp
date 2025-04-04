#include <Arduino.h>

// #define bilink_pin 7 //PB7
uint32_t bilink_pin = 3;
void blink_loop(uint32_t ms);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting...");
  
  pinMode(bilink_pin, OUTPUT);
  Serial.print(bilink_pin);
}

void loop() {
  // put your main code here, to run repeatedly:
  blink_loop(500);
  if(Serial.available()){
    delay(100);
    while (Serial.available()){
      Serial.read();
    }
    
    bilink_pin++;
    Serial.print(bilink_pin);
    pinMode(bilink_pin, OUTPUT);
  }
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
    // Serial.println(".");
  }
}