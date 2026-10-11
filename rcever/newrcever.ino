#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

Servo ch1, ch2, ch3, ch4, ch5;

struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};
MyData data;

const uint64_t pipeIn = 0xC3C3A0A0B1LL;
RF24 radio(9, 10);

unsigned long lastRecvTime = 0;

// دالة مخصصة لإعادة تعيين البيانات لقيم الأمان لمنع قفزة الـ 2000
void setToSafeValues() {
  data.throttle = 0;   // تحول بـ Map إلى 1000 (إيقاف)
  data.roll     = 127; // تحول بـ Map إلى ~1500 (المنتصف)
  data.pitch    = 127; // تحول بـ Map إلى ~1500 (المنتصف)
  data.yaw      = 127; // تحول بـ Map إلى ~1500 (المنتصف)
  data.AUX1     = 0;
  data.AUX2     = 0;
}

void setup() {
  Serial.begin(115200);
  ch1.attach(2); ch2.attach(3); ch3.attach(4); ch4.attach(5); ch5.attach(8);

  // إشارة البدء الآمنة
  ch1.writeMicroseconds(1500); 
  ch2.writeMicroseconds(1500); 
  ch3.writeMicroseconds(1000); 
  ch4.writeMicroseconds(1500); 
  ch5.writeMicroseconds(1000);
  
  setToSafeValues(); // تهيئة القيم آمنة من البداية
  delay(1000);

  radio.begin();
  radio.openReadingPipe(1, pipeIn);
  radio.setAutoAck(true);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(76);
  radio.startListening();   
}

void loop() {
  // فحص الاتصال وقراءة البيانات
  if (radio.available()) {
    radio.read(&data, sizeof(MyData));
    lastRecvTime = millis(); // تحديث الوقت عند الاستقبال الناجح فقط
  }

  // ===== نظام الـ Failsafe الذكي والمجرب =====
  // إذا مر أكثر من 300 مللي ثانية (أقل من نصف ثانية) بدون حزمة بيانات حقيقية
  if (millis() - lastRecvTime > 300) {
    setToSafeValues(); // اجبر الذاكرة على تنظيف قيم الـ 255 الوهمية فوراً
    
    ch1.writeMicroseconds(1500); // Roll في المنتصف
    ch2.writeMicroseconds(1500); // Pitch في المنتصف
    ch3.writeMicroseconds(1000); // Throttle صفر تماماً
    ch4.writeMicroseconds(1500); // Yaw في المنتصف
    ch5.writeMicroseconds(1000);
  } 
  else {
    // معالجة البيانات الطبيعية عند استقرار الاتصال
    int pwm_roll     = map(data.roll, 0, 255, 1000, 2000);
    int pwm_pitch    = map(data.pitch, 0, 255, 1000, 2000);
    int pwm_throttle = map(data.throttle, 0, 255, 1000, 2000);
    int pwm_yaw      = map(data.yaw, 0, 255, 1000, 2000);
    int pwm_aux1     = map(data.AUX1, 0, 255, 1000, 2000);

    // حماية إضافية: لو حدث تسريب لقيمة قصوى بالخطأ أثناء الفحص
    if(pwm_throttle > 2000 || pwm_throttle < 1000) pwm_throttle = 1000;

    ch1.writeMicroseconds(pwm_roll);
    ch2.writeMicroseconds(pwm_pitch);
    ch3.writeMicroseconds(pwm_throttle); // إرسال الثروتل الفعلي للطائرة
    ch4.writeMicroseconds(pwm_yaw);
    ch5.writeMicroseconds(pwm_aux1);
  }
}
