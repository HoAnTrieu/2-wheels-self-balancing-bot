#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ===== CẤU HÌNH THAM SỐ HỆ THỐNG ===== //
#define Kp       15.0
#define Ki       0.0
#define Kd       1.0

#define PWM_MAX  255
#define PWM_MIN  70
#define DEADBAND 5

#define MOTOR_A_DIR  1
#define MOTOR_B_DIR  1
#define PITCH_SIGN   1    // +1 hoặc -1

#define SDA_PIN 21
#define SCL_PIN 22

// ===== CHÂN L298N (SỬA VỀ ĐÚNG CẤU HÌNH BẠN ĐÃ TEST) ===== //
#define IN1 26
#define IN2 27
#define ENA 25

#define IN3 16
#define IN4 17
#define ENB 33

#define SAFE_ANGLE 30.0

// ===== PWM LEDC (ESP32 core mới) ===== //
#define PWM_FREQ     20000   // 20kHz
#define PWM_RES_BITS 8       // 0..255

// ===== BIẾN TOÀN CỤC ===== //
Adafruit_MPU6050 mpu;
bool armed = false;
unsigned long lastMicros = 0;
unsigned long lastPrint  = 0;
float pitch = 0.0;
float errorSum = 0.0;
float lastError = 0.0;

// ===== HÀM PWM CHO ESP32 ===== //
static inline int dutyMax() { return (1 << PWM_RES_BITS) - 1; }

static inline void pwmWrite(int pin, int duty) {
  duty = constrain(duty, 0, dutyMax());
  ledcWrite(pin, duty);          // core mới: ledcWrite theo pin đã attach
}

// ===== THIẾT LẬP HỆ THỐNG ===== //
void setup() {
  Serial.begin(115200);
  delay(200);

  // I2C + MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found! Please check wiring.");
    while (1) delay(100);
  }

  // Cấu hình MPU (giữ như bạn, có thể đổi lại nếu cần)
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // PWM attach (ESP32 core mới)
  ledcAttach(ENA, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(ENB, PWM_FREQ, PWM_RES_BITS);

  // Motor off
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  pwmWrite(ENA, 0);
  pwmWrite(ENB, 0);

  armed = false;
  lastMicros = micros();

  Serial.println("Ready (1-loop PID).");
}

// ===== VÒNG LẶP CHÍNH ===== //
void loop() {
  // Đọc MPU6050
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // dt (s)
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0f;
  lastMicros = currentMicros;

  // kẹp dt tránh spike
  if (dt <= 0.0f) dt = 0.001f;
  if (dt > 0.05f) dt = 0.01f;

  // ===== TÍNH PITCH =====
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  // Góc từ accel (deg)
  float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958f;

  // Tốc độ góc từ gyro (deg/s) - bạn đang dùng gyro.y
  float gyroPitchRate = gyro.gyro.y * 57.2958f;

  // ÁP DỤNG SIGN ĐÚNG CÁCH (SỬA LỖI QUAN TRỌNG)
  pitchAcc      *= (float)PITCH_SIGN;
  gyroPitchRate *= (float)PITCH_SIGN;

  // Tích hợp gyro
  float pitchGyro = pitch + gyroPitchRate * dt;

  // Complementary filter: alpha là trọng số accel (nhỏ), (1-alpha) cho gyro
  const float alpha = 0.02f;
  pitch = alpha * pitchAcc + (1.0f - alpha) * pitchGyro;

  // ===== SAFETY ARM/DISARM =====
  if (armed) {
    if (fabs(pitch) > SAFE_ANGLE) {
      armed = false;
      pwmWrite(ENA, 0);
      pwmWrite(ENB, 0);
      digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
      errorSum = 0;
      lastError = 0;
    }
  } else {
    if (fabs(pitch) < SAFE_ANGLE) {
      armed = true;
      errorSum = 0;
      lastError = 0;
    }
  }

  // ===== PID =====
  float output = 0.0f;

  if (armed) {
    float error = 0.0f - pitch;

    errorSum += error * dt;
    float maxErrorSum = 100.0f;
    if (errorSum >  maxErrorSum) errorSum =  maxErrorSum;
    if (errorSum < -maxErrorSum) errorSum = -maxErrorSum;

    float dError = (error - lastError) / dt;
    lastError = error;

    output = Kp * error + Ki * errorSum + Kd * dError;
  } else {
    output = 0.0f;
  }

  // ===== LIMIT + DEADBAND + PWM_MIN =====
  if (output >  PWM_MAX) output =  PWM_MAX;
  if (output < -PWM_MAX) output = -PWM_MAX;

  if (fabs(output) < DEADBAND) output = 0.0f;

  if (output > 0 && output < PWM_MIN) output = PWM_MIN;
  else if (output < 0 && output > -PWM_MIN) output = -PWM_MIN;

  // ===== MOTOR DRIVE =====
  if (output == 0.0f || !armed) {
    pwmWrite(ENA, 0);
    pwmWrite(ENB, 0);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  } else {
    int pwmValue = (int)fabs(output);

    // Motor A
    int motorA_cmd = (int)(output * MOTOR_A_DIR);
    if (motorA_cmd > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
    else                { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }

    // Motor B
    int motorB_cmd = (int)(output * MOTOR_B_DIR);
    if (motorB_cmd > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
    else                { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }

    pwmWrite(ENA, pwmValue);
    pwmWrite(ENB, pwmValue);
  }

  // ===== SERIAL PRINT =====
  unsigned long now = millis();
  if (now - lastPrint >= 50) {
    lastPrint = now;
    Serial.print("pitch=");
    Serial.print(pitch, 2);
    Serial.print(", output=");
    Serial.print(output, 1);
    Serial.print(", armed=");
    Serial.println(armed ? 1 : 0);
  }
}
