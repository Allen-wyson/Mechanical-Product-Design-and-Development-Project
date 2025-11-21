/***************************************************
  Jacket valves – sequential open test
  - 4x Servo on PCA9685 (ch 8~11)
  - Opens valves ONE BY ONE
  - No sensors
****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PWM_ADDR 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_ADDR);

const uint8_t NUM_VALVES = 4;
const uint8_t servoChannels[NUM_VALVES] = {8, 9, 10, 11};

const float SERVO_FREQ = 50.0f;

const int PULSE_MIN_US = 500;   
const int PULSE_MAX_US = 2500;  

const int ANG_MIN = 0;
const int ANG_MAX = 180;

// Direction for your jacket valves (flip if needed)
const int VALVE_OPEN_ANGLE  = 0;  
const int VALVE_CLOSE_ANGLE = 180;    

// How long each valve stays open before moving to next
const unsigned long STEP_TIME_MS = 2000;

uint8_t currentValve = 0;
unsigned long lastSwitch = 0;

// ========= helper functions =========

int clampi(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

int pwmTicksFromMicros(int us) {
  float ticks = (float)us * SERVO_FREQ * 4096.0f / 1000000.0f;
  if (ticks < 0) ticks = 0;
  if (ticks > 4095) ticks = 4095;
  return (int)(ticks + 0.5f);
}

int angleToMicros(int angle) {
  angle = clampi(angle, ANG_MIN, ANG_MAX);
  long range = (long)PULSE_MAX_US - PULSE_MIN_US;
  return PULSE_MIN_US + range * angle / (ANG_MAX - ANG_MIN);
}

void setValveAngle(uint8_t index, int angleDeg) {
  if (index >= NUM_VALVES) return;
  int us    = angleToMicros(angleDeg);
  int ticks = pwmTicksFromMicros(us);
  pwm.setPWM(servoChannels[index], 0, ticks);
}

void closeAllValves() {
  for (uint8_t i = 0; i < NUM_VALVES; i++) {
    setValveAngle(i, VALVE_CLOSE_ANGLE);
  }
  Serial.println("All valves CLOSED");
}

// ========= setup =========
void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!pwm.begin()) {
    Serial.println("PCA9685 init error");
    while (1) {}
  }

  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Start with all closed
  closeAllValves();
  lastSwitch = millis();

  Serial.println("Sequential valve test ready");
}

// ========= loop =========
void loop() {
  unsigned long now = millis();

  if (now - lastSwitch >= STEP_TIME_MS) {
    lastSwitch = now;

    // 1. Close all valves first
    closeAllValves();

    // 2. Open the next valve
    setValveAngle(currentValve, VALVE_OPEN_ANGLE);

    Serial.print("Valve ");
    Serial.print(currentValve);
    Serial.println(" OPEN");

    // Move to next valve
    currentValve++;
    if (currentValve >= NUM_VALVES) {
      currentValve = 0;
    }
  }

  // PCA9685 handles PWM automatically
}
