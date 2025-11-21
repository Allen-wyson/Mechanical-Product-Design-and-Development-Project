/***************************************************
  4x SHT41 (內部, via TCA9548A ch0~3)
  1x SHT41 (外部, via TCA9548A ch4)
  4x Servo on Adafruit 16-Channel Servo Shield (PCA9685 ch8~11)

  - 內部：每顆感測器決定各自 vent 的角度 (T+H 加權, 濾波, 死區)
  - 外部：環境溫度決定「最大開口比例」(外面越冷 → 全部 vent 上限越小)
****************************************************/

#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ========= I2C 裝置 =========
#define TCA_ADDR   0x70         // TCA9548A 預設位址
#define PWM_ADDR   0x40         // Servo Shield (PCA9685) 預設位址

Adafruit_SHT4x sht4;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_ADDR);

// ---- vent / sensor 設定 ----
const uint8_t NUM_VENTS = 4;

// 內部 4 顆 sensor 對應的 TCA channel (0~3)
const uint8_t ventTCAChannels[NUM_VENTS] = {0, 1, 2, 3};

// 外部環境 sensor 用 TCA channel 4
const uint8_t AMBIENT_TCA_CH = 4;

bool ventSensorOK[NUM_VENTS] = {false, false, false, false};
bool ambientSensorOK = false;

// TCA channel 選擇
void tcaSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);   // 每個 bit 對應一個 channel
  Wire.endTransmission();
}

// ========= Servo (PCA9685) 設定 =========

// Servo Shield 上使用哪幾個 channel
// 對應 vent 0~3
const uint8_t servoChannels[NUM_VENTS] = {8, 9, 10, 11};

// PCA9685 頻率（一般伺服 50 Hz）
const float SERVO_FREQ = 50.0f;

// 將「微秒」轉成 PCA9685 的 ticks (0~4095)
int pwmTicksFromMicros(int us) {
  float ticks = (float)us * SERVO_FREQ * 4096.0f / 1000000.0f;
  if (ticks < 0)    ticks = 0;
  if (ticks > 4095) ticks = 4095;
  return (int)(ticks + 0.5f);
}

// 一般伺服範圍約 500~2500 us
const int PULSE_MIN_US = 500;    // 對應 0°
const int PULSE_MAX_US = 2500;   // 對應 180°

int servoPulseTicks[NUM_VENTS];  // 每顆 servo 現在要輸出的 ticks

// ========= 內部控制參數（照你原本） =========

// Temperature control parameters
const float T_SET  = 23.0;
const float T_MAX  = 32.0;
const float T_DEAD = 0.2;

// Humidity control parameters
const float H_SET  = 50.0;
const float H_MAX  = 100.0;
const float H_DEAD = 3.0;

// Weights
float W_T = 0.6;
float W_H = 0.4;

// Servo angle limits
const int ANG_MIN = 0;
const int ANG_MAX = 180;

// Output smoothing (first-order filter)
const float ALPHA = 0.3;
float angleFilt[NUM_VENTS];
float lastAngleCmd[NUM_VENTS];

// Angle deadband to reduce servo jitter
const float ANGLE_DEADBAND = 2.0;  // ignore changes smaller than 2°

int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Map angle (0–180°) to pulse width (微秒)
int angleToMicros(int angle) {
  angle = clampi(angle, ANG_MIN, ANG_MAX);
  long range = (long)PULSE_MAX_US - PULSE_MIN_US;
  int us = PULSE_MIN_US + range * angle / (ANG_MAX - ANG_MIN);
  return us;
}

// Mapping: colder/drier -> 180°, hotter/more humid -> 0°
int mapTempToAngle(float tC) {
  if (isnan(tC)) return ANG_MAX;
  if (tC <= T_SET + T_DEAD) return ANG_MAX;
  if (tC >= T_MAX)          return ANG_MIN;
  float ratio = (tC - (T_SET + T_DEAD)) / (T_MAX - (T_SET + T_DEAD)); // 0..1
  int angle = (int)((1.0f - ratio) * (ANG_MAX - ANG_MIN) + ANG_MIN);
  return clampi(angle, ANG_MIN, ANG_MAX);
}

int mapHumToAngle(float rH) {
  if (isnan(rH)) return ANG_MAX;
  if (rH <= H_SET + H_DEAD) return ANG_MAX;
  if (rH >= H_MAX)          return ANG_MIN;
  float ratio = (rH - (H_SET + H_DEAD)) / (H_MAX - (H_SET + H_DEAD)); // 0..1
  int angle = (int)((1.0f - ratio) * (ANG_MAX - ANG_MIN) + ANG_MIN);
  return clampi(angle, ANG_MIN, ANG_MAX);
}

// ========= 外部環境縮放邏輯 =========
// 回傳一個 0.5 ~ 1.0 的比例：
// - T_env >= 25°C → 1.0 (完全不縮)
// - T_env <= 0°C  → 0.5 (最大只剩原本一半)
// - 中間線性插值
float computeAmbientScale(float T_env) {
  if (isnan(T_env)) return 1.0f;  // 如果外部 sensor 掛掉，先當作不縮

  if (T_env >= 25.0f) return 1.0f;
  if (T_env <= 0.0f)  return 0.5f;

  // 線性：0°C → 0.5, 25°C → 1.0
  float scale = 0.5f + 0.5f * (T_env / 25.0f);
  if (scale < 0.5f) scale = 0.5f;
  if (scale > 1.0f) scale = 1.0f;
  return scale;
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  Wire.begin();

  // 檢查 TCA 是否存在
  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("找不到 TCA9548A (0x70)，請檢查接線"));
    while (1) {}
  }

  // 初始化 PCA9685
  if (!pwm.begin()) {
    Serial.println(F("PCA9685 初始化失敗"));
    while (1) {}
  }
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // 初始化 4 顆內部 SHT41（ch0~3）
  Serial.println(F("初始化 4 顆『內部』SHT41 (ch0~3)..."));
  for (uint8_t i = 0; i < NUM_VENTS; i++) {
    uint8_t ch = ventTCAChannels[i];
    Serial.print(F("  Internal CH "));
    Serial.print(ch);
    Serial.print(F(": "));

    tcaSelect(ch);
    delay(5);

    if (sht4.begin()) {
      ventSensorOK[i] = true;
      Serial.println(F("找到 SHT4x"));
      sht4.setPrecision(SHT4X_HIGH_PRECISION);
      sht4.setHeater(SHT4X_NO_HEATER);
    } else {
      ventSensorOK[i] = false;
      Serial.println(F("找不到 SHT4x"));
    }
  }

  // 初始化外部環境 SHT41（ch4）
  Serial.println(F("\n初始化『外部環境』SHT41 (ch4)..."));
  tcaSelect(AMBIENT_TCA_CH);
  delay(5);
  if (sht4.begin()) {
    ambientSensorOK = true;
    Serial.println(F("外部 SHT4x OK"));
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
  } else {
    ambientSenso
