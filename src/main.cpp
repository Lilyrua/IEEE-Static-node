// Version แค่สลับ GPS กับ code เก่า (สำรอง)

// Version [Pressure Sensor, GPS, Battery, SHT31] LTS Comfirm by Tus (14/06/69)
// + UPGRADED: OTAA/ABP Fallback, NVS FCnt Save, and 30s Active Phase Monitor
// >>> GPS MODE: V1-STYLE (Blocking 2 วิ + เคลียร์ buffer ตอนจะอ่าน, ไม่มี background encode)

/* * HELTEC V3 LoRaWAN - MASTER DEPLOYMENT MODE (FINAL VERSION)
 * - รองรับ SHT30 พร้อมระบบ Wake-up & Dummy Read
 * - รองรับ GPS แบบ Version 1 (Blocking 2 วิ ตอนจะอ่าน)
 * - รอบส่ง 5 นาที (300,000 ms) 
 * - เสริมระบบ OTAA with ABP Fallback และจำค่า Frame Counter ลง Flash
 * - เสริมระบบ Active Phase แสดงผลหน้าจอ 30 วินาทีก่อนเข้า Sleep
 */

#include "LoRaWan_APP.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ModbusMaster.h>
#include <TinyGPSPlus.h>
#include <Adafruit_SHT31.h> 
#include <Preferences.h>      // เพิ่มไลบรารีสำหรับบันทึกค่าลง Flash Memory

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

uint32_t devAddr = (uint32_t)0x00f208a1;
// uint32_t devAddr = (uint32_t)0x00734bbc;

uint16_t userChannelsMask[6] = {0x0002, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}; // บังคับ Channel 1 เหมือนโค้ด 2
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = CLASS_A;
uint32_t appTxDutyCycle = 300000; // รอบส่ง 5 นาที

bool overTheAirActivation = true;
bool loraWanAdr = false;
bool isTxConfirmed = false;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

// ================= OTAA/ABP Fallback Control =================
#define OTAA_JOIN_TIMEOUT_MS  60000UL    // รอ OTAA Join นาน 60 วิ
#define OTAA_MAX_RETRY        3          // ลอง OTAA สูงสุด 3 ครั้ง

Preferences loraPrefs;
bool     isUsingABP       = false;
uint8_t  otaaRetryCount   = 0;
unsigned long joinStartTime = 0;
bool     joinTimerStarted = false;

// ================= ตัวแปรสำหรับจับเวลา & Active Phase =================
unsigned long lastSendTime = 0;
uint32_t currentTxWait = 300000;

#define ACTIVE_DURATION 30000UL // หน่วงเวลาดูหน้าจอ 30 วิ
unsigned long activeStartTime = 0;
bool isActivePhase = false;

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
unsigned long lastDisplayUpdate = 0;
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
float currentTemp = NAN; 
float currentHum = NAN;

String currentStatus = "Booting..."; 
int station_id = 1;

void VextON() {
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
}

// ================= FCnt Flash Storage =================
uint32_t loadFCnt() {
    loraPrefs.begin("lora_fcnt", true);
    uint32_t val = loraPrefs.getUInt("fcnt", 0);
    loraPrefs.end();
    return val;
}

void saveFCnt(uint32_t fcnt) {
    loraPrefs.begin("lora_fcnt", false);
    loraPrefs.putUInt("fcnt", fcnt);
    loraPrefs.end();
}

// ⭐ มี updateDisplay() แค่ตรงนี้ที่เดียวพอครับ
void updateDisplay() {
  if (!isScreenOn) return; 

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  
  // ================= แถวที่ 1 (บนสุด) =================
  oled.setTextSize(1);
  
  oled.setCursor(0, 0);
  String shortStatus = currentStatus;
  if (shortStatus == "Sending...") shortStatus = "Send";
  else if (shortStatus == "Joining...") shortStatus = "Join";
  else if (shortStatus == "Warmup...") shortStatus = "Warm";
  else if (shortStatus == "Booting...") shortStatus = "Boot";
  else if (shortStatus == "Manual Read") shortStatus = "Read";
  else if (shortStatus == "Monitor...") shortStatus = "Mon.";
  oled.print(shortStatus);

  oled.setCursor(38, 0);
  oled.print(finalBatteryVoltage, 2); 
  oled.print("V ");
  oled.print(batPercentage);
  oled.print("%");

  oled.setCursor(96, 0); 
  if (isActivePhase) {
      unsigned long activeElapsed = millis() - activeStartTime;
      unsigned long remain = (ACTIVE_DURATION > activeElapsed) ? (ACTIVE_DURATION - activeElapsed) / 1000 : 0;
      char timeStr[8];
      sprintf(timeStr, "%02d:%02d", (int)(remain / 60), (int)(remain % 60));
      oled.print(timeStr);
  } else if (deviceState == DEVICE_STATE_SLEEP) {
      oled.print("Sleep"); 
  } else {
      oled.print("Wait");
  }

  oled.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  // ================= แถวที่ 2: ระดับน้ำ =================
  oled.setCursor(0, 13);
  oled.print("Lvl:");
  oled.setCursor(30, 13);
  oled.setTextSize(2);
  oled.print(currentLevel, 1); 
  oled.setTextSize(1); 
  oled.print("cm");

  // ================= แถวที่ 3: อุณหภูมิและความชื้น =================
  oled.setCursor(0, 32);
  if (!isnan(currentTemp) && !isnan(currentHum)) {
      oled.print("T:"); oled.print(currentTemp, 1); oled.print("C  ");
      oled.print("H:"); oled.print(currentHum, 1); oled.print("%");
  } else {
      oled.print("T:---C  H:---%");
  }

  // ================= แถวที่ 4: พิกัด GPS =================
  oled.setCursor(0, 45);
  if (gps.location.isValid()) {
    oled.print("Lat:"); oled.println(gps.location.lat(), 4);
    oled.print("Lng:"); oled.println(gps.location.lng(), 4);
  } else {
    oled.print("GPS: Searching...");
    oled.setCursor(105, 45);
    oled.print("S:"); oled.print(gps.satellites.value());
  }
  
  oled.display();
}

// ⭐ ตามด้วย switchToABP() 
void switchToABP() {
    isUsingABP = true;
    overTheAirActivation = false;

    loraPrefs.begin("lora_fcnt", false);
    loraPrefs.putBool("use_abp", true);
    loraPrefs.end();

    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_NET_ID;
    mibReq.Param.NetID = 0;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_DEV_ADDR;
    mibReq.Param.DevAddr = devAddr;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_NWK_SKEY;
    mibReq.Param.NwkSKey = nwkSKey;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_APP_SKEY;
    mibReq.Param.AppSKey = appSKey;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_NETWORK_JOINED;
    mibReq.Param.IsNetworkJoined = true;
    LoRaMacMibSetRequestConfirm(&mibReq);

    deviceState = DEVICE_STATE_SEND;
    currentStatus = "ABP Mode";
    updateDisplay(); 
}

void prepareTxFrame(uint8_t port) {
  uint16_t levelInt = (uint16_t)(currentLevel * 10);
  int32_t latInt = (int32_t)(currentLat * 1000000.0f);
  int32_t lngInt = (int32_t)(currentLng * 1000000.0f);
  uint32_t latU = (uint32_t)latInt;
  uint32_t lngU = (uint32_t)lngInt;
  uint16_t vBatInt = (uint16_t)(finalBatteryVoltage * 100); 
  
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
    // --- 1. อ่าน GPS (V1-STYLE: Blocking 2 วิ + เคลียร์ buffer เก่าทิ้งก่อน) ---
    while (Serial1.available()) {
        Serial1.read();                       // ทิ้งข้อมูลเก่าใน buffer
    }
    unsigned long startWait = millis();
    while (millis() - startWait < 2000) {     // หยุดรอเก็บข้อมูลสด 2 วิ
        while (Serial1.available()) {
            gps.encode(Serial1.read());
        }
    }
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

    // --- 4. อ่าน SHT30 (แก้บั๊ก -45 ด้วย Dummy Read) ---
    sht31.readTemperature(); // อ่านทิ้ง 1 รอบเพื่อกระตุ้น
    delay(50); // รอเซนเซอร์ทำงาน

    float t = sht31.readTemperature(); // อ่านค่าจริง
    float h = sht31.readHumidity();
    
    if (!isnan(t) && !isnan(h) && t > -40.0 && t < 125.0) { 
        currentTemp = t;
        currentHum = h;
    } else {
        currentTemp = NAN;
        currentHum = NAN;
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

  // ⭐ หากต้องการเคลียร์ NVS ให้เอาคอมเมนต์ 4 บรรทัดนี้ออก แฟลช 1 รอบ แล้วใส่คอมเมนต์กลับ
  // loraPrefs.begin("lora_fcnt", false);
  // loraPrefs.clear();
  // loraPrefs.end();
  // Serial.println("[NVS] Cleared");

  // โหลด Mode
  loraPrefs.begin("lora_fcnt", true);
  isUsingABP = loraPrefs.getBool("use_abp", false);
  loraPrefs.end();

  if (isUsingABP) overTheAirActivation = false;
  else overTheAirActivation = true;

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // ⭐ กู้คืน Frame Counter
  uint32_t savedFcnt = loadFCnt();
  MibRequestConfirm_t mibReq;
  mibReq.Type = MIB_UPLINK_COUNTER;
  mibReq.Param.UpLinkCounter = savedFcnt;
  LoRaMacMibSetRequestConfirm(&mibReq);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ON); 

  Wire.begin(17, 18);
  
  I2CSHT.begin(SHT30_SDA, SHT30_SCL);
  delay(50); 

  if (sht31.begin(0x44)) { 
    sht31.readTemperature(); // Dummy
    sht31.readHumidity();
    delay(50);
  }

  analogReadResolution(12);

  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(10);
  digitalWrite(OLED_RESET, HIGH);
  delay(10);

  if(!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) { 
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

      if (isUsingABP) {
          deviceState = DEVICE_STATE_SEND;
          break;
      }

      if (!joinTimerStarted) {
          joinStartTime = millis();
          joinTimerStarted = true;
          otaaRetryCount++;
          saveFCnt(0); // รีเซ็ต FCnt เมื่อเริ่ม Join ใหม่
          LoRaWAN.join();
      }

      if (millis() - joinStartTime >= OTAA_JOIN_TIMEOUT_MS) {
          joinTimerStarted = false;
          if (otaaRetryCount >= OTAA_MAX_RETRY) {
              currentStatus = "ABP Fallback";
              updateDisplay();
              switchToABP();
          }
      }
      break;
    }

    case DEVICE_STATE_SEND: {
      // เซฟค่า FCnt ก่อนส่ง
      MibRequestConfirm_t mibReq;
      mibReq.Type = MIB_UPLINK_COUNTER;
      if (LoRaMacMibGetRequestConfirm(&mibReq) == LORAMAC_STATUS_OK) {
          saveFCnt(mibReq.Param.UpLinkCounter + 1);
      }

      currentStatus = "Warmup...";
      isScreenOn = true;
      oled.ssd1306_command(SSD1306_DISPLAYON);
      updateDisplay();
      
      I2CSHT.begin(SHT30_SDA, SHT30_SCL);
      sht31.begin(0x44);
      delay(50);
      
      for(int i=0; i<3; i++) {
         digitalWrite(LED_GREEN_PIN, HIGH);
         delay(150);
         digitalWrite(LED_GREEN_PIN, LOW);  
         delay(150);
      }
      digitalWrite(LED_GREEN_PIN, HIGH); 
      
      readSensorsQuick(); 
      delay(500);         
      
      currentStatus = "Sending...";
      updateDisplay();
      
      readSensorsQuick(); 
      
      prepareTxFrame(appPort);
      LoRaWAN.send();
      
      delay(1000); 
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }

    case DEVICE_STATE_CYCLE: {
      digitalWrite(LED_GREEN_PIN, LOW); 
      currentTxWait = appTxDutyCycle;
      txDutyCycleTime = currentTxWait;
      
      lastSendTime = millis(); 
      activeStartTime = millis();
      isActivePhase = true;
      isScreenOn = true;
      screenTimer = millis();
      oled.ssd1306_command(SSD1306_DISPLAYON);
      
      currentStatus = "Monitor..."; 
      updateDisplay();

      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }

    case DEVICE_STATE_SLEEP: {
      if (isActivePhase) {
          if (millis() - activeStartTime >= ACTIVE_DURATION) {
              isActivePhase = false;
              isScreenOn = false;
              oled.ssd1306_command(SSD1306_DISPLAYOFF);
              currentStatus = "Sleep";
              screenTimer = millis();
          } else {
              if (millis() - lastDisplayUpdate > 1000) {
                  lastDisplayUpdate = millis();
                  updateDisplay();
              }
              
              static unsigned long lastSensorRead = 0;
              if (millis() - lastSensorRead > 5000) {
                  lastSensorRead = millis();
                  readSensorsQuick();
              }
          }
      } else {
          LoRaWAN.sleep(loraWanClass); 
      }

      // ระบบปลุกด้วยปุ่ม PRG
      if (digitalRead(BUTTON_PIN) == LOW) {
          isScreenOn = true;
          isActivePhase = true;
          activeStartTime = millis();
          screenTimer = millis();
          oled.ssd1306_command(SSD1306_DISPLAYON);
          currentStatus = "Monitor...";
          
          I2CSHT.begin(SHT30_SDA, SHT30_SCL);
          sht31.begin(0x44);
          delay(200);
      }

      if (isScreenOn && !isActivePhase && millis() - screenTimer > SCREEN_TIMEOUT) {
          oled.ssd1306_command(SSD1306_DISPLAYOFF);
          isScreenOn = false;
      }
      break;
    }

    default: {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
