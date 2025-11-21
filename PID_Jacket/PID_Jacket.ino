/***************************************************
  4x SHT41 (內部, via TCA9548A ch0~3)
  1x SHT41 (外部, via TCA9548A ch4)

  4x Servo on Adafruit 16-Channel Servo Shield
     (PCA9685 channels 8~11)

  PID 控制 vent 開度 (0°=全開, 180°=全關)
  外部溫度會限制最大開口 (外面越冷 -> vent 越關)
****************************************************/

#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ================= I2C =================
#define TCA_ADDR 0x70
#define PWM_ADDR 0x40

Adafruit_SHT4x sht4;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_ADDR);

// ================= Vent/Sensor config =================
const uint8_t NUM_VENTS = 4;

const uint8_t ventTCAChannels[NUM_VENTS] = {0,1,2,3};
const uint8_t AMBIENT_TCA_CH = 4;

bool ventSensorOK[NUM_VENTS] = {false,false,false,false};
bool ambientSensorOK = false;

void tcaSelect(uint8_t ch){
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

// ================= Servo (PCA9685) =================
const uint8_t servoChannels[NUM_VENTS] = {8,9,10,11};
const float SERVO_FREQ = 50.0;

int pwmTicksFromMicros(int us){
  float ticks = us * SERVO_FREQ * 4096.0 / 1e6;
  if(ticks < 0) ticks = 0;
  if(ticks > 4095) ticks = 4095;
  return (int)(ticks + 0.5);
}

const int PULSE_MIN_US = 500;    // 0° = fully open
const int PULSE_MAX_US = 2500;   // 180° = fully closed

int angleToMicros(int angle){
  angle = constrain(angle,0,180);
  return PULSE_MIN_US + (long)(PULSE_MAX_US - PULSE_MIN_US) * angle / 180;
}

// ================= Comfort parameters =================
const float T_SET = 23.0;
const float H_SET = 50.0;

float W_T = 0.6;
float W_H = 0.4;

// ================= PID parameters =================
const float Kp = 6.0;
const float Ki = 0.5;
const float Kd = 0.0;

const float DT = 0.25;
const float INTEG_LIMIT = 150.0;

float err[NUM_VENTS]={0};
float prevErr[NUM_VENTS]={0};
float integ[NUM_VENTS]={0};
float angleFilt[NUM_VENTS]={0};
float lastAngleCmd[NUM_VENTS]={0};

const float ALPHA = 0.25;
const float ANGLE_DEADBAND = 2.0;

// ================= 外部環境縮放 =================
// T_env >= 25°C → scale = 1
// T_env <= 0°C  → scale = 0.5
float computeAmbientScale(float Tenv){
  if(isnan(Tenv)) return 1.0;
  if(Tenv >= 25.0) return 1.0;
  if(Tenv <= 0.0)  return 0.5;

  float scale = 0.5 + 0.5 * (Tenv / 25.0);
  return constrain(scale,0.5,1.0);
}


void setup(){
  Serial.begin(115200);
  Wire.begin();

  // TCA check
  Wire.beginTransmission(TCA_ADDR);
  if(Wire.endTransmission()!=0){
    Serial.println("TCA not found!");
    while(1);
  }

  // PCA9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Initialize internal sensors
  Serial.println("Init 4x internal SHT41...");
  for(uint8_t i=0;i<NUM_VENTS;i++){
    uint8_t ch = ventTCAChannels[i];
    tcaSelect(ch);
    delay(5);

    if(sht4.begin()){
      ventSensorOK[i]=true;
      sht4.setPrecision(SHT4X_HIGH_PRECISION);
      sht4.setHeater(SHT4X_NO_HEATER);
      Serial.print("  CH");Serial.print(ch);Serial.println(" OK");
    }else{
      ventSensorOK[i]=false;
      Serial.print("  CH");Serial.print(ch);Serial.println(" FAIL");
    }

    // 初始角度：180° = 完全關閉
    angleFilt[i] = 180;
    lastAngleCmd[i] = 180;

    int us = angleToMicros(180);
    pwm.setPWM(servoChannels[i],0,pwmTicksFromMicros(us));
  }

  // Initialize external sensor
  Serial.println("Init external SHT41...");
  tcaSelect(AMBIENT_TCA_CH);
  delay(5);
  if(sht4.begin()){
    ambientSensorOK = true;
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    Serial.println("  External OK");
  }else{
    Serial.println("  External FAIL");
    ambientSensorOK = false;
  }

  Serial.println("\nSetup complete.");
}


void loop(){
  static unsigned long lastMs = 0;
  unsigned long now = millis();

  if(now - lastMs < 250) return;
  lastMs = now;

  // ================= 外部環境 =================
  float Tenv = NAN;
  float scale = 1.0;

  if(ambientSensorOK){
    tcaSelect(AMBIENT_TCA_CH);
    sensors_event_t he, te;
    if(sht4.getEvent(&he,&te)){
      Tenv = te.temperature;
      scale = computeAmbientScale(Tenv);
    }
  }

  Serial.print("[ENV] T_env=");
  if(isnan(Tenv)) Serial.print("NaN");
  else            Serial.print(Tenv,2);
  Serial.print("  scale=");
  Serial.println(scale,2);


  // ================= 4 片 vent 的 PID 控制 =================
  for(uint8_t i=0;i<NUM_VENTS;i++){
    if(!ventSensorOK[i]) continue;

    uint8_t ch = ventTCAChannels[i];
    tcaSelect(ch);

    sensors_event_t hum, tmp;
    if(!sht4.getEvent(&hum,&tmp)) continue;

    float T = tmp.temperature;
    float H = hum.relative_humidity;

    // 舒適誤差：越熱/越潮 → e 大
    float eT = (T - T_SET);
    float eH = (H - H_SET);

    float e = W_T*eT + W_H*eH;
    err[i] = e;

    // PID
    integ[i] += e * DT;
    integ[i] = constrain(integ[i], -INTEG_LIMIT, INTEG_LIMIT);
    float deriv = (e - prevErr[i]) / DT;
    prevErr[i] = e;

    // PID 輸出：計算需要「打開」的程度
    float u = Kp*e + Ki*integ[i] + Kd*deriv;

    // u > 0 → 越熱越打開
    // u < 0 → 越冷越關，但 vent 最多只能 180°（完全關）
    float angleCmd = 180.0 - u;   // 因為 180 = 關, 0 = 開

    angleCmd = constrain(angleCmd, 0, 180);

    // 死區
    if(fabs(angleCmd - lastAngleCmd[i]) < ANGLE_DEADBAND){
      angleCmd = lastAngleCmd[i];
    }else{
      lastAngleCmd[i]=angleCmd;
    }

    // 濾波
    angleFilt[i] = ALPHA * angleCmd + (1.0 - ALPHA)*angleFilt[i];
    int angleInternal = constrain((int)(angleFilt[i]+0.5),0,180);

    // 外部縮放
    int angleScaled = angleInternal;
    angleScaled = (int)(angleInternal * scale + 0.5);
    angleScaled = constrain(angleScaled, 0, 180);

    // 寫到伺服
    int us = angleToMicros(angleScaled);
    pwm.setPWM(servoChannels[i],0,pwmTicksFromMicros(us));

    // Debug Print
    Serial.print("CH");Serial.print(ch);
    Serial.print("  T=");Serial.print(T,2);
    Serial.print("  H=");Serial.print(H,1);
    Serial.print("  e=");Serial.print(e,2);
    Serial.print("  AngInt=");Serial.print(angleInternal);
    Serial.print("  AngOut=");Serial.println(angleScaled);
  }

  Serial.println();
}
