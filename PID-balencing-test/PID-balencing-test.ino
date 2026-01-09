#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ===================== PIN CONFIG =====================
static const int PIN_ENA = 25;  // PWM Motor A
static const int PIN_IN1 = 26;
static const int PIN_IN2 = 27;

static const int PIN_IN3 = 16;
static const int PIN_IN4 = 17;
static const int PIN_ENB = 33;  // PWM Motor B

static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

// ===================== PWM CONFIG =====================
// Core mới thường dùng ledcAttach(pin, freq, resolution)
static const int PWM_FREQ = 20000; // 20kHz
static const int PWM_RES  = 8;     // 8-bit: 0..255

// Nếu core bạn vẫn dùng channel, giữ biến này để code rõ ràng.
// Với core mới, ledcWrite(pin, duty) cũng hoạt động theo pin đã attach.
static const int DUTY_MAX = (1 << PWM_RES) - 1;

// ===================== MPU =====================
Adafruit_MPU6050 mpu;

// Complementary filter (góc đơn giản)
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
static const float ALPHA = 0.98f;
uint32_t lastUs = 0;

// ===================== MOTOR CONTROL =====================
inline int clampDuty(int duty) {
  return constrain(duty, 0, DUTY_MAX);
}

void pwmWritePin(int pin, int duty) {
  duty = clampDuty(duty);

  // Tùy core: ledcWrite(pin, duty) hoặc ledcWriteTone/ledcWrite(channel,...)
  // Với core mới: ledcWrite(pin, duty) là hợp lệ sau khi ledcAttach(pin,...)
  ledcWrite(pin, duty);
}

void setMotorA(int pwm) {
  pwm = constrain(pwm, -DUTY_MAX, DUTY_MAX);
  int duty = abs(pwm);

  if (pwm > 0) { digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW); }
  else if (pwm < 0) { digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, HIGH); }
  else { digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW); }

  pwmWritePin(PIN_ENA, duty);
}

void setMotorB(int pwm) {
  pwm = constrain(pwm, -DUTY_MAX, DUTY_MAX);
  int duty = abs(pwm);

  if (pwm > 0) { digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW); }
  else if (pwm < 0) { digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, HIGH); }
  else { digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW); }

  pwmWritePin(PIN_ENB, duty);
}

void setMotors(int pwmA, int pwmB) {
  setMotorA(pwmA);
  setMotorB(pwmB);
}

// ===================== AUTO TEST STEPS =====================
struct Step {
  int a; int b;
  uint32_t ms;
  const char* name;
};

Step steps[] = {
  {  0,   0, 1000, "STOP"        },
  { +160,+160, 2000, "FORWARD"   },
  {  0,   0,  800, "STOP"        },
  { -160,-160, 2000, "BACKWARD"  },
  {  0,   0,  800, "STOP"        },
  { +160,-160, 1500, "SPIN LEFT" },
  {  0,   0,  800, "STOP"        },
  { -160,+160, 1500, "SPIN RIGHT"},
  {  0,   0, 1500, "STOP"        },
};

const int NSTEP = sizeof(steps)/sizeof(steps[0]);
int stepIdx = 0;
uint32_t stepT0 = 0;

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Direction pins
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // PWM attach (core mới)
  // Nếu hàm này không tồn tại trên core bạn, báo tôi: tôi sẽ đưa bản “tương thích kép”.
  ledcAttach(PIN_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_ENB, PWM_FREQ, PWM_RES);

  setMotors(0, 0);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // MPU init
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Check 3V3/GND/SDA21/SCL22.");
    while (true) { delay(1000); }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("MPU6050 OK. Running motor+sensor test...");
  lastUs = micros();
  stepT0 = millis();
}

// ===================== LOOP =====================
void loop() {
  // Read MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float rollAcc  = atan2f(ay, az) * 180.0f / PI;
  float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;

  float gx = g.gyro.x * 180.0f / PI;
  float gy = g.gyro.y * 180.0f / PI;

  rollDeg  = ALPHA * (rollDeg  + gx * dt) + (1.0f - ALPHA) * rollAcc;
  pitchDeg = ALPHA * (pitchDeg + gy * dt) + (1.0f - ALPHA) * pitchAcc;

  // Auto motor test
  uint32_t nowMs = millis();
  Step &s = steps[stepIdx];
  setMotors(s.a, s.b);

  if (nowMs - stepT0 >= s.ms) {
    stepIdx = (stepIdx + 1) % NSTEP;
    stepT0 = nowMs;
  }

  // Print
  static uint32_t tPrint = 0;
  if (nowMs - tPrint >= 50) {
    tPrint = nowMs;
    Serial.print("STEP=");
    Serial.print(s.name);
    Serial.print("  roll=");
    Serial.print(rollDeg, 2);
    Serial.print("  pitch=");
    Serial.print(pitchDeg, 2);
    Serial.print("  temp=");
    Serial.println(temp.temperature, 1);
  }
}
