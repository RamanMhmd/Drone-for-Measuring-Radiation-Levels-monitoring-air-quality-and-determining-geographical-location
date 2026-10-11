#include <Wire.h>
#include <Servo.h>

// ========== أطراف المحركات ==========
#define MOTOR_FL  7
#define MOTOR_FR  4
#define MOTOR_RL  6 //7
#define MOTOR_RR  5

Servo escFL, escFR, escRL, escRR;

// ========== أطراف استقبال PWM من الريسيڤر ==========
#define RC_ROLL_PIN     8   // PB0
#define RC_PITCH_PIN    9   // PB1
#define RC_THROTTLE_PIN 10  // PB2
#define RC_YAW_PIN      11  // PB3

volatile uint16_t rc_values[4] = {1500, 1500, 1000, 1500};
volatile uint32_t rc_rise_time[4] = {0};
volatile bool rc_updated = false;

uint16_t rc_roll, rc_pitch, rc_throttle, rc_yaw;
unsigned long lastRxTime = 0;

// ========== إعدادات MPU6050 ==========
const int MPU_addr = 0x68;
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
float roll = 0, pitch = 0, yaw = 0;
float accRoll, accPitch;

// متغيرات حلقة التحكم الزمنية الثابتة (250Hz -> 4000us)
unsigned long loop_timer = 0;

// ----- إعدادات الإرسال للماتلاب -----
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000; // 5000 ميكروثانية = 5ms (200Hz)

float roll_P_term = 0, roll_I_term = 0, roll_D_term = 0; 

float roll_setpoint = 0, pitch_setpoint = 0, yaw_setpoint = 0;
float roll_error = 0, pitch_error = 0, yaw_error = 0;
float roll_integral = 0, pitch_integral = 0, yaw_integral = 0;
float roll_prev_error = 0, pitch_prev_error = 0, yaw_prev_error = 0;
float roll_prev_deriv = 0, pitch_prev_deriv = 0;
float roll_PID = 0, pitch_PID = 0, yaw_PID = 0.02;

float Kp_roll = 9, Ki_roll = 0.01, Kd_roll = 0.3;
float Kp_pitch = 9, Ki_pitch = 0.01, Kd_pitch = 0.3;
float Kp_yaw = 7, Ki_yaw = 0.00, Kd_yaw = 0.0;

int throttle_base = 1000;
const int MIN_PULSE = 1000;
const int MAX_PULSE = 2000;

// تحديث معاملات المرشح التكميلي بدقة حسب المطلوب
const float alpha = 0.9996;
const float beta_acc = 0.0004;

void stopMotors() {
  escFL.writeMicroseconds(1000); escFR.writeMicroseconds(1000);
  escRL.writeMicroseconds(1000); escRR.writeMicroseconds(1000);
}

void calibrateGyro() {
  const int samples = 200;
  for (int i = 0; i < samples; i++) {
    readMPU6050();
    gyroXoffset += GyroX; gyroYoffset += GyroY; gyroZoffset += GyroZ;
    delay(3);
  }
  gyroXoffset /= samples; gyroYoffset /= samples; gyroZoffset /= samples;
}

void readMPU6050() {
  Wire.beginTransmission(MPU_addr); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  int16_t ax = Wire.read() << 8 | Wire.read(); int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read(); Wire.read(); Wire.read();
  int16_t gx = Wire.read() << 8 | Wire.read(); int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();
  AccX = ax / 16384.0; AccY = ay / 16384.0; AccZ = az / 16384.0;
  GyroX = gx / 131.0; GyroY = gy / 131.0; GyroZ = gz / 131.0;
}

ISR(PCINT0_vect) {
  uint32_t now = micros(); uint8_t pb = PINB;
  for (uint8_t i = 0; i < 4; i++) {
    bool currentState = (pb & (1 << i)) != 0;
    if (currentState) rc_rise_time[i] = now;
    else if (rc_rise_time[i] != 0) {
      uint32_t width = now - rc_rise_time[i];
      if (width >= 900 && width <= 2100) {
        rc_values[i] = (uint16_t)width;
        rc_updated = true;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // إيقاف وضع السكون وتفعيل الحساس
  Wire.beginTransmission(MPU_addr); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  
  // تفعيل الفلتر الرقمي الداخلي (DLPF) على مسجل CONFIG (0x1A) بقيمة 0x03 (~43Hz) لامتصاص الاهتزازات
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  calibrateGyro();
  
  escFL.attach(MOTOR_FL); escFR.attach(MOTOR_FR);
  escRL.attach(MOTOR_RL); escRR.attach(MOTOR_RR);
  stopMotors();
  
  PCICR |= (1 << PCIE0);
  PCMSK0 |= 0x0F;       

  lastRxTime = millis();
  lastSendTime = micros(); 
  loop_timer = micros(); // تهيئة مؤقت الحلقة الزمنية
}

void loop() {
  if (rc_updated) {
    noInterrupts();
    uint16_t r0 = rc_values[0];
    uint16_t r1 = rc_values[1];
    uint16_t r2 = rc_values[2];
    uint16_t r3 = rc_values[3];
    rc_updated = false;
    interrupts();

    rc_roll = r0; rc_pitch = r1; rc_throttle = r2; rc_yaw = r3;
    if (rc_roll > 1490 && rc_roll < 1510) rc_roll = 1500;
    if (rc_pitch > 1490 && rc_pitch < 1510) rc_pitch = 1500;
    if (rc_yaw > 1490 && rc_yaw < 1510) rc_yaw = 1500;
    
    roll_setpoint = map_float(constrain(rc_roll, 1000, 2000), 1000, 2000, -30.0, 30.0);
    pitch_setpoint = map_float(constrain(rc_pitch, 1000, 2000), 1000, 2000, -30.0, 30.0);
    yaw_setpoint = map_float(constrain(rc_yaw, 1000, 2000), 1000, 2000, -200.0, 200.0);
    throttle_base = constrain(rc_throttle, 1000, 2000);
    lastRxTime = millis();
  }

  // ===== التعديل الهيكلي الحرج للأمان =====
  if ((millis() - lastRxTime > 1000) || (rc_throttle < 1020)) {
    stopMotors();
    roll_integral = 0; pitch_integral = 0; yaw_integral = 0;
    roll_P_term = roll_I_term = roll_D_term = 0;
    throttle_base = 1000; 
    
    // ضمان مزامنة الوقت حتى عند توقف المحركات
    while (micros() - loop_timer < 4000);
    loop_timer = micros();
    return; 
  }

  // تثبيت فترة الزمن (dt) بشكل دقيق وصارم على 4000 ميكروثانية (0.004 ثانية)
  float dt = 0.004; 

  readMPU6050();
  accRoll = atan2(AccY, AccZ) * 180.0 / PI;
  accPitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;

  // دمج قراءات الحساسات باستخدام المرشح التكميلي بالمعاملات الجديدة الدقيقة (0.9996 و 0.0004)
  roll = alpha * (roll + (GyroX - gyroXoffset) * dt) + beta_acc * accRoll;
  pitch = alpha * (pitch + (GyroY - gyroYoffset) * dt) + beta_acc * accPitch;

  int m1 = 1000, m2 = 1000, m3 = 1000, m4 = 1000;
  
  if (throttle_base > 1050 && (millis() - lastRxTime <= 1000)) {
    roll_error = roll_setpoint - roll;
    pitch_error = pitch_setpoint - pitch;
    yaw_error = yaw_setpoint - (GyroZ - gyroZoffset);

    roll_integral += roll_error * dt;
    pitch_integral += pitch_error * dt;
    yaw_integral += yaw_error * dt;

    roll_integral = constrain(roll_integral, -600.0, 600.0);
    pitch_integral = constrain(pitch_integral, -600.0, 600.0);
    yaw_integral = constrain(yaw_integral, -100.0, 100.0);

    float roll_deriv = (roll_error - roll_prev_error) / dt;
    float pitch_deriv = (pitch_error - pitch_prev_error) / dt;
    float yaw_deriv = (yaw_error - yaw_prev_error) / dt;

    roll_prev_deriv = roll_deriv;
    pitch_prev_deriv = pitch_deriv;

    roll_P_term = Kp_roll * roll_error;
    roll_I_term = Ki_roll * roll_integral;
    roll_D_term = Kd_roll * roll_deriv;

    roll_PID = roll_P_term + roll_I_term + roll_D_term;
    pitch_PID = Kp_pitch * pitch_error + Ki_pitch * pitch_integral + Kd_pitch * pitch_deriv;
    yaw_PID = Kp_yaw * yaw_error + Ki_yaw * yaw_integral + Kd_yaw * yaw_deriv;

    roll_PID = constrain(roll_PID, -600, 600);
    pitch_PID = constrain(pitch_PID, -600, 600);
    yaw_PID = constrain(yaw_PID, -300, 300);

    roll_prev_error = roll_error;
    pitch_prev_error = pitch_error;
    yaw_prev_error = yaw_error;

    m1 = throttle_base + roll_PID - pitch_PID + yaw_PID;
    m2 = throttle_base - roll_PID - pitch_PID - yaw_PID;
    m3 = throttle_base + roll_PID + pitch_PID - yaw_PID;
    m4 = throttle_base - roll_PID + pitch_PID + yaw_PID;

    escFL.writeMicroseconds(constrain(m1, MIN_PULSE, MAX_PULSE));
    escFR.writeMicroseconds(constrain(m2, MIN_PULSE, MAX_PULSE));
    escRL.writeMicroseconds(constrain(m3, MIN_PULSE, MAX_PULSE));
    escRR.writeMicroseconds(constrain(m4, MIN_PULSE, MAX_PULSE));
  } else {
    stopMotors();
    roll_integral = 0; pitch_integral = 0; yaw_integral = 0;
    roll_P_term = roll_I_term = roll_D_term = 0;
  }

  // ========== إرسال البيانات للماتلاب ==========
  if (micros() - lastSendTime >= sendInterval) {
    lastSendTime = micros();
    unsigned long now = micros();
    Serial.print(now);
    Serial.print(",");
    Serial.print(throttle_base);
    Serial.print(",");
    Serial.print(GyroX - gyroXoffset, 2); 
    Serial.print(",");
    Serial.print(roll, 2);
    Serial.print(",");
    Serial.print(roll_error, 2);
    Serial.print(",");
    Serial.print(roll_P_term, 2);
    Serial.print(",");
    Serial.print(roll_I_term, 2);
    Serial.print(",");
    Serial.println(roll_D_term, 2);
  }

  // حلقة الانتظار الصارمة لضمان اكتمال دورة الـ 4000 ميكروثانية בדיוק (250Hz)
  while (micros() - loop_timer < 4000);
  loop_timer = micros();
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}