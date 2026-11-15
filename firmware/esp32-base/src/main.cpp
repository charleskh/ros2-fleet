// Bench smoke test: BNO055 on I2C (SDA=GPIO41, SCL=GPIO42).
// Scans the bus, then prints fused orientation + calibration at 5 Hz.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

constexpr int PIN_SDA = 41;
constexpr int PIN_SCL = 42;

Adafruit_BNO055 bno(55, 0x28, &Wire);

void scanI2C() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  device at 0x%02X\n", addr);
      found++;
    }
  }
  if (!found) Serial.println("  nothing found - check wiring (SDA/SCL swapped? Vin/GND?)");
}

void setup() {
  Serial.begin(115200);
  delay(2000);  // give the USB console time to attach
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setTimeOut(100);  // BNO055 clock-stretches; be patient
  scanI2C();
  if (!bno.begin()) {
    Serial.println("BNO055 not detected at 0x28. Halting.");
    while (true) delay(1000);
  }
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 OK. heading/roll/pitch (deg) | calib sys/gyro/accel/mag (0-3)");
}

void loop() {
  sensors_event_t e;
  bno.getEvent(&e);
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.printf("H %7.2f  R %7.2f  P %7.2f  |  cal %u/%u/%u/%u\n",
                e.orientation.x, e.orientation.y, e.orientation.z, sys, gyro, accel, mag);
  delay(200);
}
