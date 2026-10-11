#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <U8g2lib.h>



const uint64_t pipeOut = 0xC3C3A0A0B1LL;
RF24 radio(9, 10);

struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};
MyData data;

const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ;
float smoothRoll = 127.0;
float smoothPitch = 127.0;
unsigned long lastDisplayUpdate = 0;

void setup() {
  Serial.begin(9600);

  Wire.begin();
  Wire.beginTransmission(MPU_addr); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(76);
  radio.openWritingPipe(pipeOut);
  radio.stopListening();   // ⚠️ TX في وضع الإرسال فقط بشكل دائم
}

void loop() {
  // قراءة الذراعين
  data.throttle = map(constrain(analogRead(A2), 10, 1015), 10, 1015, 0, 255);
  data.yaw      = map(constrain(analogRead(A3), 10, 1015), 10, 1015, 0, 255);

  // قراءة MPU6050
  Wire.beginTransmission(MPU_addr); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 4, true);
  if (Wire.available() >= 4) {
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    smoothRoll = (smoothRoll * 0.8) + (map(constrain(AcX, -15000, 15000), -15000, 15000, 0, 255) * 0.2);
    smoothPitch = (smoothPitch * 0.8) + (map(constrain(AcY, -15000, 15000), -15000, 15000, 0, 255) * 0.2);
    data.roll = (byte)smoothRoll; data.pitch = (byte)smoothPitch;
  }
  // AUX1 و AUX2 سيبقيان 0 (لا يؤثران)

  // ===== إرسال القيم الحقيقية بشكل مستمر =====
  radio.write(&data, sizeof(MyData));

  // تحديث الشاشة (بدون GPS، بدون أرمينغ)
 
}