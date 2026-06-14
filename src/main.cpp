// VERSION CODE : SHT latest Edit with v1 (14-06-2024 )

/* * HELTEC V3 LoRaWAN - MASTER DEPLOYMENT MODE (BEST PRACTICE)
 * - รองรับ SHT30 (I2C: SDA=6, SCL=7) แบบ Non-Blocking
 * - รองรับ GPS แบบ Background (ไม่บล็อกระบบ)
 * - รอบส่ง 5 นาที (300,000 ms) 
 */

#include "LoRaWan_APP.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ModbusMaster.h>
#include <TinyGPSPlus.h>
#include <Adafruit_SHT31.h> 

// ================= LoRaWAN Config for static 1 =================
uint8_t devEui[] = {0x63, 0x0C, 0x36, 0x21, 0x13, 0x33, 0x95, 0xC4};
uint8_t appEui[] = {0xD3, 0x93, 0xB6, 0xD3, 0xEE, 0xF5, 0xAE, 0xF1}; 
uint8_t appKey[] = {0x9B, 0xC4, 0x8B, 0x94, 0x1C, 0x48, 0x75, 0x5B, 0xA6, 0xEA, 0x79, 0xD1, 0xFA, 0xB8, 0x53, 0xEF};

uint8_t nwkSKey[] = {0x59, 0x9E, 0xB8, 0xCB, 0x47, 0x49, 0xE3, 0x40, 0x73, 0x41, 0x9C, 0x77, 0x53, 0x7B, 0x13, 0x1F};
uint8_t appSKey[] = {0xF7, 0xA8, 0x8F, 0x89, 0x6C, 0x6B, 0xAA, 0xFB, 0xA5, 0xF2, 0x31, 0x96, 0xCA, 0x7D, 0x05, 0xDC};

// ================= LoRaWAN Config for static 2 =================
// uint8_t devEui[] = {0x7F, 0x7D, 0x8C, 0xCF, 0x7A, 0xF2, 0x6A, 0x75};
// uint8_t appEui[] = {0x1C, 0x0A, 0x50, 0x5C, 0x96, 0x55, 0x21, 0xD7}; 
// uint8_t appKey[] = {0x20, 0xF5, 0xAE, 0x40, 0x61, 0x03, 0x4D, 0xAC, 0xC0, 0xE9, 0x91, 0x73, 0xDA, 0x5B, 0x2A, 0xAB};

// uint8_t nwkSKey[] = {0x59, 0x9E, 0xB8, 0xCB, 0x47, 0x49, 0xE3, 0x40, 0x73, 0x41, 0x9C, 0x77, 0x53, 0x7B, 0x13, 0x1F};
// uint8_t appSKey[] = {0xF7, 0xA8, 0x8F, 0x89, 0x6C, 0x6B, 0xAA, 0xFB, 0xA5, 0xF2, 0x31, 0x96, 0xCA, 0x7D, 0x05, 0xDC};

uint32_t devAddr = (uint32_t)0x01a88516; 

uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = CLASS_A;
uint32_t appTxDutyCycle = 30000; // รอบส่ง 5 นาที

bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = false;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

// ================= ตัวแปรสำหรับจับเวลา =================
unsigned long lastSendTime = 0;
uint32_t currentTxWait = 30000;
unsigned long lastBtnTime = 0; 

// ================= Hardware Config =================
#define RELAY_PIN 26
#define RELAY_ON  HIGH    
#define RELAY_OFF LOW   

#define LED_RED_PIN 5     
#define LED_GREEN_PIN 4   

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
#define GPS_RX_PIN 19 
#define GPS_TX_PIN 20
#define CUSTOM_BAT_PIN 2  

// SHT30 I2C Config
#define SHT30_SDA 6
#define SHT30_SCL 7
TwoWire I2CSHT = TwoWire(1); 
Adafruit_SHT31 sht31 = Adafruit_SHT31(&I2CSHT); 

ModbusMaster node;
TinyGPSPlus gps;

// Global Variables
float currentLevel = 0.0;
float currentLat = 0.0;
float currentLng = 0.0;
int batPercentage = 0; 
float finalBatteryVoltage = 0.0; 
float smoothedVoltage = 0.0;     
float calibrationOffset = 0; 

// ตัวแปรเก็บค่า SHT30
float currentTemp = NAN; // เปลี่ยนค่าเริ่มต้นเป็น NAN
float currentHum = NAN;

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
  
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(currentStatus);

  oled.setCursor(45, 0);
  oled.print(finalBatteryVoltage, 2); 
  oled.print("V ");
  oled.print(batPercentage);
  oled.print("%");

  oled.setCursor(95, 0); 
  if (deviceState == DEVICE_STATE_SLEEP) {
    oled.print("Sleep"); 
  } else if (lastSendTime > 0) {
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

  oled.setCursor(0, 13);
  oled.print("Lvl:");
  oled.setCursor(30, 13);
  oled.setTextSize(2);
  oled.print(currentLevel, 1); 
  oled.setTextSize(1); 
  oled.print("cm");

  // แถวอุณหภูมิและความชื้น (เพิ่มเช็คค่าก่อนแสดงผล)
  oled.setCursor(0, 32);
  if (!isnan(currentTemp) && !isnan(currentHum)) {
      oled.print("T: "); oled.print(currentTemp, 1); oled.print("C  ");
      oled.print("H: "); oled.print(currentHum, 1); oled.print("%");
  } else {
      oled.print("T: ---C  H: ---%");
  }

  oled.setCursor(0, 45);
  if (gps.location.isValid()) {
    oled.print("Lat:"); oled.println(gps.location.lat(), 4);
    oled.print("Lng:"); oled.println(gps.location.lng(), 4);
  } else {
    oled.print("GPS: Searching...");
    oled.setCursor(90, 45);
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
  uint16_t vBatInt = (uint16_t)(finalBatteryVoltage * 100); 
  
  // ป้องกันค่าขยะส่งเข้า LoRa หากอ่านเซนเซอร์ไม่ได้
  int16_t tempInt = isnan(currentTemp) ? 0 : (int16_t)(currentTemp * 100);
  uint16_t humInt = isnan(currentHum) ? 0 : (uint16_t)(currentHum * 100);

  appDataSize = 18; 
  appData[0]  = (uint8_t)station_id;
  appData[1]  = (levelInt >> 8) & 0xFF;
  appData[2]  = levelInt & 0xFF;
  appData[3]  = (latU >> 24) & 0xFF;
  appData[4]  = (latU >> 16) & 0xFF;
  appData[5]  = (latU >> 8) & 0xFF;
  appData[6]  = latU & 0xFF;
  appData[7]  = (lngU >> 24) & 0xFF;
  appData[8]  = (lngU >> 16) & 0xFF;
  appData[9]  = (lngU >> 8) & 0xFF;
  appData[10] = lngU & 0xFF;
  appData[11] = (vBatInt >> 8) & 0xFF;
  appData[12] = vBatInt & 0xFF;
  appData[13] = (uint8_t)batPercentage;
  appData[14] = (tempInt >> 8) & 0xFF;
  appData[15] = tempInt & 0xFF;
  appData[16] = (humInt >> 8) & 0xFF;
  appData[17] = humInt & 0xFF;
}

void readSensorsQuick() {
    // --- 1. อ่าน GPS ---
    if (gps.location.isValid()) {
        currentLat = gps.location.lat();
        currentLng = gps.location.lng();
    }

    // --- 2. อ่านแบตเตอรี่ ---
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
    for (int i = 10; i <= 20; i++) sum += samples[i];
    float medianAvgMv = sum / 11.0;
    float currentAvgVoltage = (medianAvgMv / 1000.0) * 5.96;
    
    if (smoothedVoltage == 0.0) smoothedVoltage = currentAvgVoltage; 
    else smoothedVoltage = (currentAvgVoltage * 0.1) + (smoothedVoltage * 0.9);
    
    finalBatteryVoltage = smoothedVoltage + calibrationOffset;
    batPercentage = map((int)(finalBatteryVoltage * 100), 330, 420, 0, 100);
    batPercentage = constrain(batPercentage, 0, 100);

    // --- 3. อ่านค่าระดับน้ำ (RS485) ---
    int clearCount = 0;
    while(Serial2.available() && clearCount < 100) { 
        Serial2.read(); 
        clearCount++;
    }
    
    bool rs485Success = false;
    for(int r = 0; r < 3; r++) { 
        uint8_t result = node.readHoldingRegisters(0x0004, 1);
        if (result == node.ku8MBSuccess) {
            currentLevel = node.getResponseBuffer(0) / 10.0; 
            rs485Success = true;
            break; 
        }
        delay(100); 
    }
    if(!rs485Success) Serial.println("RS485 Read Failed");

    // --- 4. อ่าน SHT30 (แก้ไขให้ถูกต้องแล้ว) ---
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
    
    // ตรวจสอบความถูกต้องของข้อมูลจาก Library ทันที
    if (!isnan(t) && !isnan(h) && t > -40.0 && t < 125.0) { 
        currentTemp = t;
        currentHum = h;
        Serial.printf("SHT30 Read OK: %.1f C | %.1f %%\n", currentTemp, currentHum);
    } else {
        Serial.println("SHT30 Read Error: No Signal or I2C Collision");
    }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); 
  
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
  I2CSHT.begin(SHT30_SDA, SHT30_SCL);
  
  delay(50); // เพิ่ม Delay ให้ SHT30 พร้อมทำงานบนบัส I2C

  if (!sht31.begin(0x44)) { 
    Serial.println("SHT30 not found!");
  } else {
    Serial.println("SHT30 Ready!");
  }

  analogReadResolution(12);

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
  while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
  }

  if (digitalRead(BUTTON_PIN) == LOW) {
    if (millis() - lastBtnTime > 500) { 
      lastBtnTime = millis();
      if (!isScreenOn) {
        oled.ssd1306_command(SSD1306_DISPLAYON); 
        isScreenOn = true;
        
        currentStatus = "Manual Read";
        updateDisplay(); 
        readSensorsQuick(); 
        currentStatus = "Monitor...";
      }
      screenTimer = millis();
    }
  }

  if (isScreenOn && (millis() - screenTimer > SCREEN_TIMEOUT)) {
    oled.ssd1306_command(SSD1306_DISPLAYOFF); 
    isScreenOn = false;
  }

  if (isScreenOn && (millis() - lastDisplayUpdate > 1000) && deviceState != DEVICE_STATE_SLEEP) { 
    lastDisplayUpdate = millis();
    updateDisplay(); 
  }

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
      
      for(int i=0; i<3; i++) {
         digitalWrite(LED_GREEN_PIN, HIGH);
         delay(150);
         digitalWrite(LED_GREEN_PIN, LOW);  
         delay(150);
      }
      digitalWrite(LED_GREEN_PIN, HIGH); 
      
      readSensorsQuick(); 
      prepareTxFrame(appPort);
      LoRaWAN.send();
      
      delay(1000); 
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }

    case DEVICE_STATE_CYCLE: {
      digitalWrite(LED_GREEN_PIN, LOW); 
      currentTxWait = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
      txDutyCycleTime = currentTxWait;
      
      lastSendTime = millis(); 
      currentStatus = "Sleep"; 
      updateDisplay();

      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }

    case DEVICE_STATE_SLEEP: {
      LoRaWAN.sleep(loraWanClass); 
      break;
    }

    default: {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
