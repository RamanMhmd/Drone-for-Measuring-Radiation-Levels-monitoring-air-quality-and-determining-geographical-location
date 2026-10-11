#include <Servo.h>

// حدد دبابيس المحركات حسب توصيلك (مثلاً الأربع دبابيس المستخدمة في الدرون)
const int motorPins[] = {7, 4, 6, 5}; // قم بتعديل الأرقام حسب دبابيسك الفعلية
Servo motors[4];

void setup() {
  Serial.begin(9600);
  Serial.println("=== ESC Calibration Mode ===");
  Serial.println("1. Disconnect battery/power from ESCs.");
  Serial.println("2. Send MAX throttle (type 'max' or wait).");
  Serial.println("3. Connect battery. You will hear calibration beeps.");
  Serial.println("4. Send MIN throttle (type 'min').");
  
  // ربط الدبابيس
  for (int i = 0; i < 4; i++) {
    motors[i].attach(motorPins[i], 1000, 2000);
  }

  // الخطوة الأولى: إرسال أقصى سرعة (Max Throttle = 2000)
  Serial.println("Sending MAX Throttle (2000)...");
  for (int i = 0; i < 4; i++) {
    motors[i].writeMicroseconds(2000);
  }
  
  Serial.println("Now, please CONNECT power to your ESCs/Motors.");
  Serial.println("Wait for the special calibration tones, then press any key in Serial Monitor to set MIN throttle...");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // عند الضغط على أي مفتاح بعد توصيل البطارية، يتم إرسال أقل سرعة (Min Throttle = 1000)
    Serial.println("Sending MIN Throttle (1000)... Calibration complete!");
    for (int i = 0; i < 4; i++) {
      motors[i].writeMicroseconds(1000);
    }
    
    Serial.println("ESCs are now calibrated. You can upload your main drone code.");
    while(true); // إيقاف الحلقات بعد انتهاء المعايرة
  }
}