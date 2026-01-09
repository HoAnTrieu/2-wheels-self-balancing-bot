#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ===================== PIN CONFIG =====================
static const int PIN_ENA = 25;
static const int PIN_IN1 = 26;
static const int PIN_IN2 = 27;

static const int PIN_IN3 = 16;
static const int PIN_IN4 = 17;
static const int PIN_ENB = 33;

static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

// ===================== PWM CONFIG =====================
static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;
static const int DUTY_MAX = (1 << PWM_RES) - 1;

static const int PWM_DEADZONE = 130;

// ===================== MPU =====================
Adafruit_MPU6050 mpu;

float rollDeg = 0.0f;
static const float ALPHA = 0.98f;
uint32_t lastUs = 0;

// ===================== PID =====================
float setpoint = 0.0f;
float Kp = 18.0f;
float Ki = 0.6f;
float Kd = 0.8f;

float pidI = 0.0f;
float lastErr = 0.0f;

// ===================== MOTOR =====================
int lastPWM = 0;
static const int PWM_SLEW = 20;

inline int clampPWM(int pwm) {
  return constrain(pwm, -DUTY_MAX, DUTY_MAX);
}

void pwmWritePin(int pin, int duty) {
  duty = constrain(duty, 0, DUTY_MAX);
  ledcWrite(pin, duty);
}

void setMotorA(int pwm) {
  pwm = clampPWM(pwm);
  int duty = abs(pwm);

  if (pwm > 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  } else if (pwm < 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }
  pwmWritePin(PIN_ENA, duty);
}

void setMotorB(int pwm) {
  pwm = clampPWM(pwm);
  int duty = abs(pwm);

  if (pwm > 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
  } else if (pwm < 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
  }
  pwmWritePin(PIN_ENB, duty);
}

void setMotors(int pwm) {
  // Giới hạn tốc độ thay đổi PWM
  if (pwm > lastPWM + PWM_SLEW) pwm = lastPWM + PWM_SLEW;
  if (pwm < lastPWM - PWM_SLEW) pwm = lastPWM - PWM_SLEW;

  lastPWM = pwm;
  setMotorA(pwm);
  setMotorB(pwm);
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  ledcAttach(PIN_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_ENB, PWM_FREQ, PWM_RES);

  setMotors(0);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  lastUs = micros();
  Serial.println("READY: SELF BALANCING ACTIVE");
}

// ===================== LOOP =====================
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;
  if (dt <= 0 || dt > 0.03f) dt = 0.01f;

  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  float gx = g.gyro.x * 180.0f / PI;

  float rollAcc = atan2f(ay, az) * 180.0f / PI;
  rollDeg = ALPHA * (rollDeg + gx * dt) + (1.0f - ALPHA) * rollAcc;

  // ===== NGẮT KHI XE ĐỔ =====
  if (abs(rollDeg) > 35.0f) {
    pidI = 0;
    lastErr = 0;
    setMotors(0);
    return;
  }

  // ===== PID (ĐẢO DẤU) =====
  float err = setpoint - rollDeg;

  // Deadband góc
  if (abs(err) < 0.5f) err = 0;

  pidI += err * dt;
  pidI = constrain(pidI, -30, 30);

  float dErr = (err - lastErr) / dt;
  lastErr = err;

  int pwm = Kp * err + Ki * pidI + Kd * dErr;

  // Bù ma sát
  if (pwm > 0) pwm += PWM_DEADZONE;
  if (pwm < 0) pwm -= PWM_DEADZONE;

  pwm = clampPWM(pwm);
  setMotors(pwm);

  // DEBUG
  static uint32_t tPrint = 0;
  if (millis() - tPrint > 50) {
    tPrint = millis();
    Serial.print("roll=");
    Serial.print(rollDeg, 2);
    Serial.print(" pwm=");
    Serial.println(pwm);
  }
}
