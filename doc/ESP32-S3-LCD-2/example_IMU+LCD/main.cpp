#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>

namespace {

constexpr uint8_t kImuAddress = 0x6B;
constexpr uint8_t kQmiWhoAmIReg = 0x00;
constexpr uint8_t kQmiWhoAmIValue = 0x05;
constexpr uint8_t kQmiCtrl1Reg = 0x02;
constexpr uint8_t kQmiCtrl2Reg = 0x03;
constexpr uint8_t kQmiCtrl3Reg = 0x04;
constexpr uint8_t kQmiCtrl5Reg = 0x06;
constexpr uint8_t kQmiCtrl7Reg = 0x08;
constexpr uint8_t kQmiTempLowReg = 0x33;
constexpr uint8_t kQmiAccelLowReg = 0x35;
constexpr uint8_t kQmiStatus0Reg = 0x2E;
constexpr uint8_t kQmiResetReg = 0x60;

constexpr int kLcdSclk = 39;
constexpr int kLcdMosi = 38;
constexpr int kLcdMiso = 40;
constexpr int kLcdDc = 42;
constexpr int kLcdReset = -1;
constexpr int kLcdCs = 45;
constexpr int kLcdBacklight = 1;

constexpr int kImuSda = 48;
constexpr int kImuScl = 47;

constexpr uint16_t kScreenWidth = 240;
constexpr uint16_t kScreenHeight = 320;
constexpr uint8_t kScreenRotation = 1;

constexpr uint32_t kImuReadPeriodUs = 5000;      // 200 Hz host update rate
constexpr uint32_t kSerialPrintPeriodMs = 100;
constexpr uint32_t kScreenRefreshPeriodMs = 100;

// QMI8658 hardware limits:
// - 200 Hz ODR is not supported by the sensor.
// - 500 dps full scale is not supported; closest available range is 512 dps.
// To keep LPF near 100 Hz, the sensor runs at 2000 Hz with LPF mode 5.32% -> ~106 Hz cutoff.
constexpr uint8_t kCtrl2Accel8gOdr2000Hz = 0x22;
constexpr uint8_t kCtrl3Gyro512dpsOdr2000Hz = 0x52;
constexpr uint8_t kCtrl5Lpf106Hz = 0x55;
constexpr float kAccelScale = 8.0f / 32768.0f;
constexpr float kGyroScale = 512.0f / 32768.0f;

struct ImuSample {
  float accelX = 0.0f;
  float accelY = 0.0f;
  float accelZ = 0.0f;
  float gyroX = 0.0f;
  float gyroY = 0.0f;
  float gyroZ = 0.0f;
  float temperatureC = 0.0f;
  bool valid = false;
};

Arduino_DataBus *bus = new Arduino_ESP32SPI(
    kLcdDc, kLcdCs, kLcdSclk, kLcdMosi, kLcdMiso);
Arduino_GFX *gfx = new Arduino_ST7789(
    bus, kLcdReset, kScreenRotation, true, kScreenWidth, kScreenHeight);

ImuSample latestSample;
uint32_t lastImuReadUs = 0;
uint32_t lastSerialPrintMs = 0;
uint32_t lastScreenRefreshMs = 0;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kImuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t reg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(kImuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received = Wire.requestFrom(static_cast<int>(kImuAddress), static_cast<int>(length));
  if (received != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool qmiDataAvailable() {
  uint8_t status = 0;
  return readRegisters(kQmiStatus0Reg, &status, 1) && ((status & 0x03U) != 0U);
}

bool initDisplay() {
  if (!gfx->begin()) {
    return false;
  }

  gfx->fillScreen(BLACK);
  pinMode(kLcdBacklight, OUTPUT);
  digitalWrite(kLcdBacklight, HIGH);
  return true;
}

bool initImu() {
  uint8_t whoAmI = 0;
  if (!readRegisters(kQmiWhoAmIReg, &whoAmI, 1) || whoAmI != kQmiWhoAmIValue) {
    return false;
  }

  if (!writeRegister(kQmiResetReg, 0xFF)) {
    return false;
  }
  delay(100);

  return writeRegister(kQmiCtrl1Reg, 0x40) &&
         writeRegister(kQmiCtrl2Reg, kCtrl2Accel8gOdr2000Hz) &&
         writeRegister(kQmiCtrl3Reg, kCtrl3Gyro512dpsOdr2000Hz) &&
         writeRegister(kQmiCtrl5Reg, kCtrl5Lpf106Hz) &&
         writeRegister(kQmiCtrl7Reg, 0x03);
}

bool readImuSample(ImuSample &sample) {
  uint8_t raw[14] = {};
  if (!readRegisters(kQmiTempLowReg, raw, sizeof(raw))) {
    return false;
  }

  auto toInt16 = [](uint8_t low, uint8_t high) -> int16_t {
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
  };

  const float temperature = static_cast<float>(raw[1]) + static_cast<float>(raw[0]) / 256.0f;
  const int16_t ax = toInt16(raw[2], raw[3]);
  const int16_t ay = toInt16(raw[4], raw[5]);
  const int16_t az = toInt16(raw[6], raw[7]);
  const int16_t gx = toInt16(raw[8], raw[9]);
  const int16_t gy = toInt16(raw[10], raw[11]);
  const int16_t gz = toInt16(raw[12], raw[13]);

  sample.accelX = ax * kAccelScale;
  sample.accelY = ay * kAccelScale;
  sample.accelZ = az * kAccelScale;
  sample.gyroX = gx * kGyroScale;
  sample.gyroY = gy * kGyroScale;
  sample.gyroZ = gz * kGyroScale;
  sample.temperatureC = temperature;
  sample.valid = true;
  return true;
}

void printImuConfig() {
  Serial.println("QMI8658 config:");
  Serial.println("  Requested by user: 200 Hz, LPF around 100 Hz, 8g, 500 dps");
  Serial.println("  Applied on hardware: sensor ODR 2000 Hz, LPF ~106 Hz, 8g, 512 dps");
  Serial.println("  Host read loop: 200 Hz");
}

void drawScreen(const ImuSample &sample) {
  gfx->fillScreen(BLACK);
  gfx->setTextSize(1);
  gfx->setTextColor(CYAN);
  gfx->setCursor(8, 8);
  gfx->println("ESP32-S3-LCD-2 Test");

  gfx->setTextColor(WHITE);
  gfx->setCursor(8, 28);
  gfx->println("LCD: OK");
  gfx->println("IMU: QMI8658");
  gfx->println("Sensor ODR: 2000 Hz");
  gfx->println("Host loop: 200 Hz");
  gfx->println("LPF: ~106 Hz");
  gfx->println("Accel: +/-8g");
  gfx->println("Gyro: +/-512 dps");

  gfx->setTextColor(YELLOW);
  gfx->setCursor(8, 110);
  gfx->printf("AX: %7.3f g\n", sample.accelX);
  gfx->printf("AY: %7.3f g\n", sample.accelY);
  gfx->printf("AZ: %7.3f g\n", sample.accelZ);

  gfx->setTextColor(GREEN);
  gfx->printf("GX: %7.2f dps\n", sample.gyroX);
  gfx->printf("GY: %7.2f dps\n", sample.gyroY);
  gfx->printf("GZ: %7.2f dps\n", sample.gyroZ);

  gfx->setTextColor(MAGENTA);
  gfx->printf("Temp: %6.2f C\n", sample.temperatureC);
}

void printSample(const ImuSample &sample) {
  Serial.printf(
      "AX=%0.3f AY=%0.3f AZ=%0.3f g | GX=%0.2f GY=%0.2f GZ=%0.2f dps | T=%0.2f C\n",
      sample.accelX, sample.accelY, sample.accelZ,
      sample.gyroX, sample.gyroY, sample.gyroZ,
      sample.temperatureC);
}

[[noreturn]] void haltWithMessage(const __FlashStringHelper *message) {
  Serial.println(message);
  if (gfx != nullptr) {
    gfx->fillScreen(BLACK);
    gfx->setTextColor(RED);
    gfx->setTextSize(2);
    gfx->setCursor(8, 20);
    gfx->println(message);
  }
  while (true) {
    delay(1000);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Booting Waveshare ESP32-S3-LCD-2 hardware test");

  if (!initDisplay()) {
    haltWithMessage(F("LCD init failed"));
  }

  Wire.begin(kImuSda, kImuScl);
  Wire.setClock(400000);

  if (!initImu()) {
    haltWithMessage(F("IMU init failed"));
  }

  printImuConfig();
  drawScreen(latestSample);
}

void loop() {
  const uint32_t nowUs = micros();
  if (static_cast<uint32_t>(nowUs - lastImuReadUs) >= kImuReadPeriodUs) {
    lastImuReadUs = nowUs;
    if (qmiDataAvailable()) {
      readImuSample(latestSample);
    }
  }

  const uint32_t nowMs = millis();
  if (latestSample.valid && static_cast<uint32_t>(nowMs - lastSerialPrintMs) >= kSerialPrintPeriodMs) {
    lastSerialPrintMs = nowMs;
    printSample(latestSample);
  }

  if (latestSample.valid && static_cast<uint32_t>(nowMs - lastScreenRefreshMs) >= kScreenRefreshPeriodMs) {
    lastScreenRefreshMs = nowMs;
    drawScreen(latestSample);
  }
}
