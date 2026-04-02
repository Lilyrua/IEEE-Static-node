/* * HELTEC V3 LoRaWAN - MASTER DEPLOYMENT MODE
 * 1. ส่ง LoRaWAN ทุกๆ 5 นาที พร้อมเวลานับถอยหลังบนจอ
 * 2. GPS (UART Toggle)
 * 3. หน้าจอ OLED ปิดอัตโนมัติ 15 วิ (ปลุกด้วยปุ่ม PRG)
 * 4. ระดับน้ำ: Dummy (พร้อมเปิดใช้ RS485 Modbus)
 * 5. แบตเตอรี่: อ่านผ่าน ADC ขา 2 พร้อมระบบ Median + EMA Filter (นิ่งและเสถียรที่สุด)
 */

#include "LoRaWan_APP.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ModbusMaster.h>
#include <TinyGPSPlus.h>

// ================= LoRaWAN Config =================
uint8_t devEui[] = {0x70, 0xB3, 0xD5, 0x7E, 0xD8, 0x00, 0x50, 0xCE};
uint8_t appEui[] = {0x74, 0x64, 0x64, 0x2D, 0x73, 0x74, 0x61, 0x74};
uint8_t appKey[] = {0x0C, 0x0F, 0xD7, 0x7B, 0x46, 0x76, 0x1F, 0x79, 0x2A, 0xBB, 0xC3, 0xD7, 0x12, 0xDC, 0x8E, 0x60};

uint8_t nwkSKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appSKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint32_t devAddr = (uint32_t)0x007e6ae1;

uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = CLASS_A;
uint32_t appTxDutyCycle = 30000; // รอบส่ง 5 นาที (300,000 ms)

bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = false;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

// ================= ตัวแปรสำหรับจับเวลา =================
unsigned long lastSendTime = 0;
uint32_t currentTxWait = 30000;

// ================= Hardware Config =================
#define RELAY_PIN 26
#define RELAY_ON  HIGH    
#define RELAY_OFF LOW   

#define LED_RED_PIN 4     // LED สีแดง (พร้อมทำงาน)
#define LED_GREEN_PIN 5   // LED สีเขียว (กำลังส่งข้อมูล)

// หน้าจอ OLED และปุ่มกด
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    21 
#define VEXT_PIN      36  
#define BUTTON_PIN    0   

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool isScreenOn = true;
unsigned long screenTimer = 0;
const unsigned long SCREEN_TIMEOUT = 15000;

// เซนเซอร์
#define RS485_RX 48
#define RS485_TX 47
#define GPS_RX_PIN 20 
#define GPS_TX_PIN 19
#define CUSTOM_BAT_PIN 2  // ขาอ่านแบตเตอรี่

ModbusMaster node;
TinyGPSPlus gps;

// Global Variables
float currentLevel = 0.0;
float currentLat = 0.0;
float currentLng = 0.0;
int batPercentage = 0; 
float finalBatteryVoltage = 0.0; // เก็บค่าโวลต์
float smoothedVoltage = 0.0;     // สำหรับระบบ EMA Filter
float calibrationOffset = 0.056; // Calibrate แล้ว (เทียบมิเตอร์จริง)

String currentStatus = "Booting..."; 
int station_id = 1;
unsigned long lastDisplayUpdate = 0;

void VextON() {
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
}

void updateDisplay() {
  if (!isScreenOn) return; 

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  
  // 1. แถวบนสุด: โหมดการทำงาน
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(currentStatus);

  // 2. แถวบนสุด (ตรงกลาง): โชว์แบตเตอรี่และเปอร์เซ็นต์
  oled.setCursor(45, 0);
  oled.print(finalBatteryVoltage, 2); 
  oled.print("V ");
  oled.print(batPercentage);
  oled.print("%");

  // 3. แถวบนสุด (ขวา): เวลานับถอยหลัง
  oled.setCursor(95, 0); 
  if (lastSendTime > 0 && deviceState == DEVICE_STATE_SLEEP) {
    unsigned long elapsed = millis() - lastSendTime;
    unsigned long remain = (currentTxWait > elapsed) ? (currentTxWait - elapsed) / 1000 : 0;
    int m = remain / 60;
    int s = remain % 60;
    
    char timeStr[16];
    sprintf(timeStr, "%02d:%02d", m, s); 
    oled.print(timeStr);
  } else {
    oled.print("Wait");
  }

  oled.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  // 4. แถวกลาง: ค่าระดับน้ำ
  oled.setCursor(0, 15);
  oled.print("Lvl:");
  oled.setCursor(35, 15);
  oled.setTextSize(2);
  oled.print(currentLevel, 1); oled.setTextSize(1); oled.print("cm");

  // 5. แถวล่าง: พิกัด GPS
  oled.setCursor(0, 40);
  if (gps.location.isValid()) {
    oled.print("Lat:"); oled.println(gps.location.lat(), 4);
    oled.print("Lng:"); oled.println(gps.location.lng(), 4);
  } else {
    oled.print("GPS: Searching...");
    oled.setCursor(90, 40);
    oled.print("S:"); oled.print(gps.satellites.value());
  }
  
  oled.display();
}

void prepareTxFrame(uint8_t port) {
  uint16_t levelInt = (uint16_t)(currentLevel * 10);
  int32_t latInt = (int32_t)(currentLat * 1000000.0f);
  int32_t lngInt = (int32_t)(currentLng * 1000000.0f);
  uint32_t latU = (uint32_t)latInt;
  uint32_t lngU = (uint32_t)lngInt;
  uint16_t vBatInt = (uint16_t)(finalBatteryVoltage * 100); // 4.12V -> 412

  // *** อัปเดตขนาด Payload เป็น 14 Bytes ***
  appDataSize = 14; 

  appData[0] = (uint8_t)station_id;
  appData[1] = (levelInt >> 8) & 0xFF;
  appData[2] = levelInt & 0xFF;
  appData[3] = (latU >> 24) & 0xFF;
  appData[4] = (latU >> 16) & 0xFF;
  appData[5] = (latU >> 8) & 0xFF;
  appData[6] = latU & 0xFF;
  appData[7]  = (lngU >> 24) & 0xFF;
  appData[8]  = (lngU >> 16) & 0xFF;
  appData[9]  = (lngU >> 8) & 0xFF;
  appData[10] = lngU & 0xFF;
  
  // *** ส่ง finalBatteryVoltage และ Bat ***
  appData[11] = (vBatInt >> 8) & 0xFF;
  appData[12] = vBatInt & 0xFF;
  appData[13] = (uint8_t)batPercentage;
}

void readSensorsQuick() {
    // --- 1. อ่าน GPS (UART Toggle) ---
    Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); 
    unsigned long startWait = millis();
    while (millis() - startWait < 1500) { 
        while (Serial1.available()) {
            gps.encode(Serial1.read());
        }
    }
    if (gps.location.isValid()) {
        currentLat = gps.location.lat();
        currentLng = gps.location.lng();
        Serial.print("GPS Fix: ");
    } else {
        Serial.print("GPS No Fix: ");
    }
    Serial1.end(); 
    Serial.print(currentLat, 6); Serial.print(", "); Serial.println(currentLng, 6);

    // --- 2. อ่านแบตเตอรี่ผ่าน ADC (Median + EMA Filter) ---
    int samples[31];
    for (int i = 0; i < 31; i++) {
      samples[i] = analogReadMilliVolts(CUSTOM_BAT_PIN);
      delay(2);
    }
    for (int i = 0; i < 30; i++) {
      for (int j = i + 1; j < 31; j++) {
        if (samples[i] > samples[j]) {
          int temp = samples[i];
          samples[i] = samples[j];
          samples[j] = temp;
        }
      }
    }
    long sum = 0;
    for (int i = 10; i <= 20; i++) {
      sum += samples[i];
    }
    float medianAvgMv = sum / 11.0;
    float currentAvgVoltage = (medianAvgMv / 1000.0) * 6.615;
    
    if (smoothedVoltage == 0.0) {
      smoothedVoltage = currentAvgVoltage; 
    } else {
      smoothedVoltage = (currentAvgVoltage * 0.1) + (smoothedVoltage * 0.9);
    }
    
    finalBatteryVoltage = smoothedVoltage + calibrationOffset;
    batPercentage = map((int)(finalBatteryVoltage * 100), 250, 370, 0, 100);
    if (batPercentage > 100) batPercentage = 100;
    if (batPercentage < 0) batPercentage = 0;

    Serial.printf("Bat: %.2fV | %d%%\n", finalBatteryVoltage, batPercentage);

    // --- 3. อ่านค่าระดับน้ำ (ปัจจุบันใช้ Dummy) ---
    /* // หากต้องการใช้เซนเซอร์ RS485 จริง ให้ลบ /* และ */
    uint8_t result = node.readHoldingRegisters(0x0004, 1);
    if (result == node.ku8MBSuccess) {
      currentLevel = node.getResponseBuffer(0) / 10.0; 
    } else {
       Serial.println("Sensor Read Error");
    }
    /*
    currentLevel = random(100, 500) / 10.0; // บรรทัดนี้สำหรับทดสอบ
    Serial.print("Lvl: "); Serial.println(currentLevel);
    */
}

void setup() {
  Serial.begin(115200);
  
  VextON(); 
  delay(100);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, HIGH);  
  digitalWrite(LED_GREEN_PIN, LOW); 
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  screenTimer = millis();

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ON); 

  Wire.begin(17, 18);
  analogReadResolution(12); // ตั้งความละเอียด ADC

  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(10);
  digitalWrite(OLED_RESET, HIGH);
  delay(10);

  if(!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) { 
     Serial.println(F("SSD1306 failed"));
     digitalWrite(LED_RED_PIN, LOW); 
     for(;;); 
  }
  
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(10, 20);
  oled.println("SYSTEM OK!");
  oled.display();
  delay(1000);
  
  Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);      
  node.begin(1, Serial2);
}

void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT: {
      #if (LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
      #endif
      LoRaWAN.init(loraWanClass, loraWanRegion);
      break;
    }

    case DEVICE_STATE_JOIN: {
      currentStatus = "Joining...";
      updateDisplay();
      LoRaWAN.join();
      break;
    }

    case DEVICE_STATE_SEND: {
      currentStatus = "Sending...";
      updateDisplay();
      
      // --- เพิ่มจังหวะกระพริบ Pilot Lamp สีเขียว 3 ครั้ง ---
      for(int i=0; i<3; i++) {
         digitalWrite(LED_GREEN_PIN, HIGH); // สั่งไฟติด (ถ้าวงจรคุณเป็น Active LOW ให้แก้เป็น LOW)
         delay(150);
         digitalWrite(LED_GREEN_PIN, LOW);  // สั่งไฟดับ (ถ้าวงจรคุณเป็น Active LOW ให้แก้เป็น HIGH)
         delay(150);
      }
      
      // เปิดไฟค้างไว้ระหว่างอ่านเซนเซอร์และส่งข้อมูล
      digitalWrite(LED_GREEN_PIN, HIGH); 
      
      readSensorsQuick(); 
      prepareTxFrame(appPort);
      LoRaWAN.send();
      
      // หน่วงเวลาให้เห็นไฟค้างอีกนิดก่อนดับ
      delay(1000); 
      
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }

    case DEVICE_STATE_CYCLE: {
      digitalWrite(LED_GREEN_PIN, LOW); 
      currentTxWait = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
      txDutyCycleTime = currentTxWait;
      
      lastSendTime = millis(); 
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }

    case DEVICE_STATE_SLEEP: {
      // ให้วิทยุ LoRaWAN หลับและประมวลผลเครือข่าย
      LoRaWAN.sleep(loraWanClass); 
      
      // 1. กดปุ่ม PRG เพื่อเปิดจอ
      if (digitalRead(BUTTON_PIN) == LOW) {
        if (!isScreenOn) {
          oled.ssd1306_command(SSD1306_DISPLAYON); 
          isScreenOn = true;
          
          currentStatus = "Reading...";
          updateDisplay(); 
          readSensorsQuick(); 
          currentStatus = "Monitor...";
        }
        screenTimer = millis();
        delay(200); 
      }

      // 2. ปิดจออัตโนมัติ 15 วินาที
      if (isScreenOn && (millis() - screenTimer > SCREEN_TIMEOUT)) {
        oled.ssd1306_command(SSD1306_DISPLAYOFF); 
        isScreenOn = false;
      }

      // 3. วาดเวลานับถอยหลังใหม่ทุก 1 วิ (เมื่อจอเปิดอยู่)
      if (isScreenOn && (millis() - lastDisplayUpdate > 1000)) { 
        lastDisplayUpdate = millis();
        updateDisplay(); 
      }
      
      break;
    }

    default: {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}