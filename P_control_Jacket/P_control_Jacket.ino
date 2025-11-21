/***************************************************
  4x SHT41 (via TCA9548A ch0~3)
  4x Servo on Adafruit 16-Channel Servo Shield (PCA9685 ch8~11)

  - Temperature + Humidity weighted control (same logic as your original code)
  - Uses PCA9685 instead of manual PWM
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

// 使用幾個 sensor / servo（用 ch0~3 對應 servo 8~11）
const uint8_t NUM_CH = 4;
bool sensor_ok[NUM_CH] = {false, false, false, false};

// TCA channel 選擇
void tcaSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);   // 每個 bit 對應一個 channel
  Wire.endTransmission();
}

// ========= Servo 相關設定（PCA9685） =========

// Servo Shield 上使用哪幾個 channel
const uint8_t servoChannels[NUM_CH] = {8, 9, 10, 11};

// PCA9685 頻率（一般伺服 50 Hz）
const float SERVO_FREQ = 50.0f;

// 將「微秒」轉成 PCA9685 的 ticks (0~4095)
// ticks = us * freq(Hz) * 4096 / 1e6
int pwmTicksFromMicros(int us) {
  float ticks = (float)us * SERVO_FREQ * 4096.0f / 1000000.0f;
  if (ticks < 0) ticks = 0;
  if (ticks > 4095) ticks = 4095;
  return (int)(ticks + 0.5f);
}

// 一般伺服常見範圍約 500~2500 us，你原本是 0~2500，我這裡改成比較安全一點
const int PULSE_MIN_US = 500;    // 對應 0°
const int PULSE_MAX_US = 2500;   // 對應 180°

int servoPulseTicks[NUM_CH];     // 每顆 servo 現在要輸出的 ticks

// ========= 控制參數（沿用你原本那支） =========

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
float angleFilt[NUM_CH];
float lastAngleCmd[NUM_CH];

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

  // 初始化 4 顆 SHT41（ch0~3）
  Serial.println(F("初始化 4 顆 SHT41（TCA ch0~3）..."));
  for (uint8_t ch = 0; ch < NUM_CH; ch++) {
    Serial.print(F("  Channel "));
    Serial.print(ch);
    Serial.print(F(": "));

    tcaSelect(ch);
    delay(5);

    if (sht4.begin()) {
      sensor_ok[ch] = true;
      Serial.println(F("找到 SHT4x"));
      sht4.setPrecision(SHT4X_HIGH_PRECISION);
      sht4.setHeater(SHT4X_NO_HEATER);
    } else {
      sensor_ok[ch] = false;
      Serial.println(F("找不到 SHT4x"));
    }
  }

  // 初始化每顆 servo 的狀態（先全部開到 180°）
  for (uint8_t i = 0; i < NUM_CH; i++) {
    angleFilt[i]      = ANG_MAX;
    lastAngleCmd[i]   = ANG_MAX;
    int us            = angleToMicros(ANG_MAX);
    servoPulseTicks[i] = pwmTicksFromMicros(us);
    pwm.setPWM(servoChannels[i], 0, servoPulseTicks[i]);
  }

  Serial.println(F("初始化完成，開始控制。"));
}

void loop() {
  static unsigned long lastSampleMillis = 0;
  unsigned long nowMillis = millis();

  if (nowMillis - lastSampleMillis >= 250) {
    lastSampleMillis += 250;

    for (uint8_t ch = 0; ch < NUM_CH; ch++) {
      if (!sensor_ok[ch]) {
        Serial.print(F("[CH"));
        Serial.print(ch);
        Serial.println(F("] 無感測器 / 初始化失敗"));
        continue;
      }

      // 選 TCA channel
      tcaSelect(ch);

      sensors_event_t humidity, temp;
      if (!sht4.getEvent(&humidity, &temp)) {
        Serial.print(F("[CH"));
        Serial.print(ch);
        Serial.println(F("] 讀取失敗"));
        continue;
      }

      float tC = temp.temperature;
      float rH = humidity.relative_humidity;

      int angT = mapTempToAngle(tC);
      int angH = mapHumToAngle(rH);

      float wsum = (W_T + W_H);
      if (wsum <= 0.0f) {
        W_T = 1.0f;
        W_H = 0.0f;
        wsum = 1.0f;
      }

      float angleCmd = (W_T * angT + W_H * angH) / wsum;

      // 死區：避免抖動
      if (fabs(angleCmd - lastAngleCmd[ch]) < ANGLE_DEADBAND) {
        angleCmd = lastAngleCmd[ch];
      } else {
        lastAngleCmd[ch] = angleCmd;
      }

      // 一階濾波
      angleFilt[ch] = ALPHA * angleCmd + (1.0f - ALPHA) * angleFilt[ch];
      int angleOut = clampi((int)(angleFilt[ch] + 0.5f), ANG_MIN, ANG_MAX);

      // 角度 → 微秒 → ticks → 寫到對應 Servo Shield channel
      int us   = angleToMicros(angleOut);
      int ticks = pwmTicksFromMicros(us);
      servoPulseTicks[ch] = ticks;
      pwm.setPWM(servoChannels[ch], 0, ticks);

      // Debug / Plot 輸出（沿用你原本格式）
      Serial.print("CH"); Serial.print(ch);
      Serial.print("_Temp:");  Serial.print(tC, 2);   Serial.print('\t');
      Serial.print("CH"); Serial.print(ch);
      Serial.print("_Hum:");   Serial.print(rH, 1);   Serial.print('\t');
      Serial.print("CH"); Serial.print(ch);
      Serial.print("_Angle:"); Serial.println(180 - angleOut);
    }

    Serial.println();
  }

  // 其餘時間交給 PCA9685 自己產生 PWM，不用手動維持 20ms frame
}
