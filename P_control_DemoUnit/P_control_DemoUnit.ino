/***************************************************
  SHT4x + Servo P-control (Temperature + Humidity)
  Manual PWM, no Servo.h
  Plot: Temperature (°C), Humidity (%), Servo angle (deg)
****************************************************/
#include <Adafruit_SHT4x.h>
#include <math.h>  // for fabs()

Adafruit_SHT4x sht4;

// ======== Servo manual PWM settings ========
const int SERVO_PIN = 10;

// Fixed 50 Hz → 20 ms period
const unsigned long SERVO_PERIOD = 20000UL;   // 20 ms

// Servo pulse range (you can fine-tune from measurements)
const int PULSE_MIN = 0;      // maps to 0°
const int PULSE_MAX = 2500;   // maps to 180°

int servoPulseWidth = 1500;   // current pulse width to output (µs)

// ======== Control parameters ========
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
float angleFilt   = ANG_MAX;

// Angle deadband to reduce servo jitter
const float ANGLE_DEADBAND = 2.0;  // ignore changes smaller than 2°
float lastAngleCmd = ANG_MAX;


// ======== Utility functions ========
int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Map angle (0–180°) to pulse width (PULSE_MIN–PULSE_MAX)
void setServoAngleHW(int angle) {
  angle = clampi(angle, ANG_MIN, ANG_MAX);
  long range = (long)PULSE_MAX - PULSE_MIN;
  servoPulseWidth = PULSE_MIN + range * angle / (ANG_MAX - ANG_MIN);
}

// Mapping: colder/drier -> 180°, hotter/more humid -> 0°
int mapTempToAngle(float tC){
  if (isnan(tC)) return ANG_MAX;
  if (tC <= T_SET + T_DEAD) return ANG_MAX;
  if (tC >= T_MAX)          return ANG_MIN;
  float ratio = (tC - (T_SET + T_DEAD)) / (T_MAX - (T_SET + T_DEAD)); // 0..1
  int angle = (int)((1.0 - ratio) * (ANG_MAX - ANG_MIN) + ANG_MIN);
  return clampi(angle, ANG_MIN, ANG_MAX);
}

int mapHumToAngle(float rH){
  if (isnan(rH)) return ANG_MAX;
  if (rH <= H_SET + H_DEAD) return ANG_MAX;
  if (rH >= H_MAX)          return ANG_MIN;
  float ratio = (rH - (H_SET + H_DEAD)) / (H_MAX - (H_SET + H_DEAD)); // 0..1
  int angle = (int)((1.0 - ratio) * (ANG_MAX - ANG_MIN) + ANG_MIN);
  return clampi(angle, ANG_MIN, ANG_MAX);
}


void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  // Wait up to 3 s for Serial Plotter to attach
  while (!Serial && millis() - t0 < 3000) {}

  if (!sht4.begin()) { while (1) {} }
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);

  pinMode(SERVO_PIN, OUTPUT);

  angleFilt    = ANG_MAX;
  lastAngleCmd = ANG_MAX;
  setServoAngleHW(ANG_MAX);    // start with vent fully open
}

// One 20 ms frame: generate one PWM pulse + use remaining time for sensing/control
void loop() {
  unsigned long frameStart = micros();

  // ===== 1) Generate one PWM pulse =====
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(servoPulseWidth);
  digitalWrite(SERVO_PIN, LOW);

  // ===== 2) Within the same 20 ms frame, do sensing and control (non-blocking) =====
  static unsigned long lastSampleMillis = 0;
  unsigned long nowMillis = millis();

  if (nowMillis - lastSampleMillis >= 250) {
    lastSampleMillis += 250;

    // Read SHT41
    sensors_event_t humidity, temp;
    sht4.getEvent(&humidity, &temp);
    float tC = temp.temperature;
    float rH = humidity.relative_humidity;

    // Compute angle commands from T and H
    int angT = mapTempToAngle(tC);
    int angH = mapHumToAngle(rH);

    float wsum = (W_T + W_H);
    if (wsum <= 0.0) {
      // fallback to temperature only if weights are invalid
      W_T = 1.0;
      W_H = 0.0;
      wsum = 1.0;
    }

    float angleCmd = (W_T * angT + W_H * angH) / wsum;

    // Angle deadband: ignore small changes to reduce jitter
    if (fabs(angleCmd - lastAngleCmd) < ANGLE_DEADBAND) {
      angleCmd = lastAngleCmd;
    } else {
      lastAngleCmd = angleCmd;
    }

    // Exponential smoothing
    angleFilt = ALPHA * angleCmd + (1.0 - ALPHA) * angleFilt;
    int angleOut = clampi((int)(angleFilt + 0.5f), ANG_MIN, ANG_MAX);

    // Update PWM target for next frames
    setServoAngleHW(angleOut);

    // Serial Plotter output
    Serial.print("Temp:");  Serial.print(tC, 2);   Serial.print('\t');
    Serial.print("Hum:");   Serial.print(rH, 1);   Serial.print('\t');
    Serial.print("Angle:"); Serial.println(180 - angleOut);
  }

  // ===== 3) Wait until the 20 ms frame is complete to keep period stable =====
  unsigned long elapsed = micros() - frameStart;
  if (elapsed < SERVO_PERIOD) {
    delayMicroseconds(SERVO_PERIOD - elapsed);
  }
}
