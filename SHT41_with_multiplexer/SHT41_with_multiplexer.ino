/***************************************************
  TCA9548A + 2x SHT41 (channels 1 and 2)
****************************************************/

#include <Wire.h>
#include <Adafruit_SHT4x.h>

#define TCA_ADDR 0x70

Adafruit_SHT4x sht1;
Adafruit_SHT4x sht2;

void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);  // enable that channel
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("TCA9548A + SHT41 test");

  // ===== Sensor 1 (Channel 1) =====
  tcaSelect(1);
  delay(10);
  if (!sht1.begin()) {
    Serial.println("SHT41 NOT found on channel 1");
  } else {
    Serial.println("SHT41 found on channel 1");
  }

  // ===== Sensor 2 (Channel 2) =====
  tcaSelect(2);
  delay(10);
  if (!sht2.begin()) {
    Serial.println("SHT41 NOT found on channel 2");
  } else {
    Serial.println("SHT41 found on channel 2");
  }
}

void loop() {
  sensors_event_t humidity, temp;

  // ===== Read Sensor 1 =====
  tcaSelect(1);
  delay(2);
  sht1.getEvent(&humidity, &temp);

  Serial.print("CH1 -> T: ");
  Serial.print(temp.temperature);
  Serial.print(" C, H: ");
  Serial.println(humidity.relative_humidity);

  // ===== Read Sensor 2 =====
  tcaSelect(2);
  delay(2);
  sht2.getEvent(&humidity, &temp);

  Serial.print("CH2 -> T: ");
  Serial.print(temp.temperature);
  Serial.print(" C, H: ");
  Serial.println(humidity.relative_humidity);

  Serial.println("---------------");
  delay(1000);
}
