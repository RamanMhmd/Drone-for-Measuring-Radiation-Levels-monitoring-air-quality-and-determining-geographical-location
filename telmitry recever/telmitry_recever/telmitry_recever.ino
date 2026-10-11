#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <U8g2lib.h>

// ========== إعدادات الشاشة ==========
U8G2_SSD1309_128X64_NONAME0_1_4W_SW_SPI u8g2(U8G2_R0, 2, 3, 4, 5, 6);

// ========== إعدادات NRF24 ==========
RF24 radio(9, 10);  // CE, CSN
const uint64_t pipeIn = 0xF0F0F0F0A1LL;

// تعريف الهياكل المطابقة للمرسل
struct __attribute__((packed)) GpsData {
  uint8_t packetId;
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

struct __attribute__((packed)) RadiationData {
  uint8_t packetId;
  unsigned long cpm;
  float uSv;
};

struct __attribute__((packed)) GasData {
  uint8_t packetId;
  float temperature;
  float pressure;
  float humidity;
  float pollutionLevel;
  float gasResistance;
};

// متغيرات تخزين القيم لعرضها على الشاشة
GpsData currentGps;
RadiationData currentRad;
GasData currentGas;

// ========== أطراف الجويستيك والإنذار ==========
#define JOY_THROTTLE A2
#define JOY_YAW      A3
#define BUZZER_PIN   A0   

const unsigned long RADIATION_THRESHOLD = 100;

unsigned long lastDisplayUpdate = 0;
unsigned long lastReceiveTime = 0;
bool connected = false;

int localThrottle = 0;
int localYaw = 0;

void setup() {
  Serial.begin(9600);
  u8g2.begin();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  radio.begin();
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(1, pipeIn);
  radio.startListening();

  Serial.println("Receiver 3-Stage Ready!");
}

void drawRadiationWarningIcon(int x, int y) {
  u8g2.drawLine(x, y + 8, x + 5, y);     
  u8g2.drawLine(x + 5, y, x + 10, y + 8); 
  u8g2.drawLine(x, y + 8, x + 10, y + 8); 
  u8g2.drawPixel(x + 5, y + 3);
  u8g2.drawPixel(x + 5, y + 6);
}

void loop() {
  // 1. استقبال البيانات وتصنيفها حسب الـ packetId
  if (radio.available()) {
    char buffer[32]; // أقصى حجم حزمة
    radio.read(&buffer, sizeof(buffer));
    
    uint8_t packetId = buffer[0];

    if (packetId == 1) {
      memcpy(&currentGps, buffer, sizeof(GpsData));
      lastReceiveTime = millis();
    } else if (packetId == 2) {
      memcpy(&currentRad, buffer, sizeof(RadiationData));
      lastReceiveTime = millis();
    } else if (packetId == 3) {
      memcpy(&currentGas, buffer, sizeof(GasData));
      lastReceiveTime = millis();
    }
  }

  connected = (millis() - lastReceiveTime < 3000);

  // 2. نظام الإنذار
  bool isDanger = (connected && currentRad.cpm >= RADIATION_THRESHOLD);
  if (isDanger) {
    digitalWrite(BUZZER_PIN, HIGH); 
  } else {
    digitalWrite(BUZZER_PIN, LOW);  
  }

  // 3. قراءة الجويستيك
  localThrottle = map(constrain(analogRead(JOY_THROTTLE), 10, 1015), 10, 1015, 0, 255);
  localYaw      = map(constrain(analogRead(JOY_YAW), 10, 1015), 10, 1015, 0, 255);

  // 4. تحديث الشاشة كل 200 ms
  if (millis() - lastDisplayUpdate >= 200) {
    lastDisplayUpdate = millis();
    
    int pageIndex = (millis() / 4000) % 4; 
    bool blinkState = (millis() / 400) % 2 == 0;

    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_profont11_tf); 
      u8g2.setCursor(2, 8);
      u8g2.print(connected ? "[LINK: OK]" : "[LINK: NO]");
      
      if (isDanger && blinkState) {
        drawRadiationWarningIcon(114, 0); 
      }
      
      u8g2.drawLine(0, 10, 128, 10); 

      u8g2.setFont(u8g2_font_ncenB08_tr); 

      if (pageIndex == 0) {
        // الصفحة 1: التحكم والوقت
        u8g2.setCursor(0, 26);
        u8g2.print("Throttle: "); u8g2.print(localThrottle);
        u8g2.setCursor(0, 42);
        u8g2.print("Yaw Pos:  "); u8g2.print(localYaw);
        
        u8g2.setCursor(0, 60);
        u8g2.print("Time: ");
        u8g2.print(currentGps.gpsHour); u8g2.print(":");
        u8g2.print(currentGps.gpsMinute); u8g2.print(":");
        u8g2.print(currentGps.gpsSecond);

      } else if (pageIndex == 1) {
        // الصفحة 2: الـ GPS والموقع
        u8g2.setCursor(0, 24);
        u8g2.print("Lat: "); u8g2.print(currentGps.fixValid ? String(currentGps.latitude, 4) : "---");
        u8g2.setCursor(0, 39);
        u8g2.print("Lon: "); u8g2.print(currentGps.fixValid ? String(currentGps.longitude, 4) : "---");
        u8g2.setCursor(0, 54);
        u8g2.print("Spd: "); u8g2.print(currentGps.fixValid ? String(currentGps.speed, 1) + "km/h" : "---");
        u8g2.setCursor(75, 54);
        u8g2.print("Alt:"); u8g2.print(currentGps.fixValid ? String(currentGps.altitude, 0) + "m" : "---");

      } else if (pageIndex == 2) {
        // الصفحة 3: الإشعاع واتجاه الحركة
        u8g2.setCursor(0, 26);
        u8g2.print("Radiation CPM: "); u8g2.print(currentRad.cpm);
        u8g2.setCursor(0, 42);
        u8g2.print("Dose uSv/h:    "); u8g2.print(currentRad.uSv, 3);
        u8g2.setCursor(0, 60);
        u8g2.print("Course:        "); u8g2.print(currentGps.course, 0); u8g2.print(" deg");

      } else {
        // الصفحة 4: البيئة وحساس الغاز BME680
        u8g2.setCursor(0, 24);
        u8g2.print("Temp: "); u8g2.print(currentGas.temperature, 1); u8g2.print(" C");
        u8g2.setCursor(65, 24);
        u8g2.print("Hum: "); u8g2.print(currentGas.humidity, 0); u8g2.print("%");
        
        u8g2.setCursor(0, 42);
        u8g2.print("Pres: "); u8g2.print(currentGas.pressure, 0); u8g2.print(" hPa");
        
        u8g2.setCursor(0, 60);
        u8g2.print("Air Poll: "); u8g2.print(currentGas.pollutionLevel, 1); u8g2.print(" %");
      }

    } while (u8g2.nextPage());
  }
}