#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// ========== إعدادات NRF24 ==========
RF24 radio(9, 10);  // CE, CSN
const uint64_t pipeOut = 0xF0F0F0F0A1LL;  

// ========== إعدادات GPS ==========
static const int RXPin = 3, TXPin = 4;
SoftwareSerial ss(RXPin, TXPin);
TinyGPSPlus gps;

// ========== إعدادات BME680 ==========
Adafruit_BME680 bme; 

// ========== إعدادات حساس الإشعاع ==========
const int gmPin = 2;             
volatile unsigned long counts = 0;       
unsigned long cpm = 0;         
unsigned long previousMillisGM = 0;
double uSv = 0;

// هيكل البيانات الخاص بالـ GPS (حزمة رقم 1)
struct __attribute__((packed)) GpsData {
  uint8_t packetId = 1;
  float latitude;
  float longitude;
  float speed;
  float altitude;
  float course;
  uint8_t gpsHour;
  uint8_t gpsMinute;
  uint8_t gpsSecond;
  uint16_t gpsYear;
  uint8_t gpsMonth;
  uint8_t gpsDay;
  bool fixValid;
};

// هيكل البيانات الخاص بالإشعاع (حزمة رقم 2)
struct __attribute__((packed)) RadiationData {
  uint8_t packetId = 2;
  unsigned long cpm;
  float uSv;
};

// هيكل البيانات الخاص بحساس الغاز BME680 (حزمة رقم 3)
struct __attribute__((packed)) GasData {
  uint8_t packetId = 3;
  float temperature;
  float pressure;
  float humidity;
  float pollutionLevel;
  float gasResistance;
};

GpsData gpsTelemetry;
RadiationData radTelemetry;
GasData gasTelemetry;

unsigned long lastSend = 0;
uint8_t sendStep = 0; // للتبديل بين الحزم الثلاث

void tube_impulse() {         
  counts++;
}

void setup() {
  Serial.begin(9600);
  ss.begin(9600);
  Wire.begin();

  pinMode(gmPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(gmPin), tube_impulse, RISING);

  if (!bme.begin(0x77)) { 
    if (!bme.begin(0x76)) {
      Serial.println(F("BME680 not found!"));
    }
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); 

  radio.begin();
  radio.setChannel(108);            
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(pipeOut);
  radio.stopListening();            

  Serial.println("Transmitter 3-Stage Ready!");
}

void loop() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  // تحديث قراءات حساس الإشعاع كل 15 ثانية
  if (millis() - previousMillisGM >= 15000) {
    previousMillisGM = millis();
    noInterrupts();
    unsigned long currentCounts = counts;
    counts = 0; 
    interrupts();
    cpm = currentCounts * 4; 
    uSv = ((float)cpm / 151.0);
  }

  // إرسال البيانات تباعاً كل 330 ملي ثانية (توزيع ذكي ومستقر)
  if (millis() - lastSend >= 330) {
    lastSend = millis();
    sendStep++;

    if (sendStep == 1) {
      // المرحلة الأولى: إرسال بيانات الـ GPS
      gpsTelemetry.latitude  = gps.location.lat();
      gpsTelemetry.longitude = gps.location.lng();
      gpsTelemetry.speed     = gps.speed.kmph();
      gpsTelemetry.altitude  = gps.altitude.meters();
      gpsTelemetry.course    = gps.course.deg();
      gpsTelemetry.fixValid  = gps.location.isValid();
      gpsTelemetry.gpsHour   = gps.time.hour();
      gpsTelemetry.gpsMinute = gps.time.minute();
      gpsTelemetry.gpsSecond = gps.time.second();
      gpsTelemetry.gpsYear   = gps.date.year();
      gpsTelemetry.gpsMonth  = gps.date.month();
      gpsTelemetry.gpsDay    = gps.date.day();


      radio.write(&gpsTelemetry, sizeof(GpsData));

    } else if (sendStep == 2) {
      // المرحلة الثانية: إرسال بيانات الإشعاع
      radTelemetry.cpm = cpm;
      radTelemetry.uSv = uSv;

      radio.write(&radTelemetry, sizeof(RadiationData));

    } else {
      // المرحلة الثالثة: إرسال بيانات الغاز BME680
      if (bme.performReading()) {
        float gasResK = bme.gas_resistance / 1000.0;
        float pollutionLevel = 0;
        if (gasResK < 50.0) { 
          pollutionLevel = (1.0 - (gasResK / 50.0)) * 100.0;
          if (pollutionLevel > 100.0) pollutionLevel = 100.0;
          if (pollutionLevel < 0.0) pollutionLevel = 0.0;
        }

        gasTelemetry.temperature    = bme.temperature;
        gasTelemetry.pressure       = bme.pressure / 100.0; 
        gasTelemetry.humidity       = bme.humidity;
        gasTelemetry.pollutionLevel = pollutionLevel;
        gasTelemetry.gasResistance  = gasResK;
      }

      radio.write(&gasTelemetry, sizeof(GasData));
      sendStep = 0; // العودة للمرحلة الأولى
    }
  }
}