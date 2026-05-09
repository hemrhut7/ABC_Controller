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
constexpr uint8_t kTouchAddress = 0x15;  // Common on CST816/CTS816S family

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
constexpr uint32_t kSerialPrintPeriodMs = 200;
constexpr uint32_t kScreenRefreshPeriodMs = 200;
constexpr uint32_t kTouchReadPeriodMs = 20;
constexpr uint16_t kValueColumnX = 56;
constexpr uint16_t kValueFieldWidth = 170;

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

struct TouchSample {
  int16_t x = 0;
  int16_t y = 0;
  bool touched = false;
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
uint32_t lastTouchReadMs = 0;
uint32_t lastTouchPrintMs = 0;
uint32_t lastRateReportMs = 0;
bool screenLayoutDrawn = false;
TouchSample latestTouch;
bool touchControllerDetected = false;
uint32_t imuReadCount = 0;
uint32_t touchPollCount = 0;
uint32_t touchActiveCount = 0;
float imuRateHz = 0.0f;
float touchPollRateHz = 0.0f;
float touchActiveRateHz = 0.0f;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kImuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegistersFrom(uint8_t address, uint8_t reg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received = Wire.requestFrom(static_cast<int>(address), static_cast<int>(length));
  if (received != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool isI2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool readTouchCst816Like(TouchSample &sample) {
  uint8_t raw[5] = {};
  if (!readRegistersFrom(kTouchAddress, 0x02, raw, sizeof(raw))) {
    return false;
  }

  const uint8_t points = raw[0] & 0x0F;
  if (points == 0) {
    sample.touched = false;
    sample.valid = true;
    return true;
  }

  // 0x03/0x04: X high/low, 0x05/0x06: Y high/low
  const int16_t x = static_cast<int16_t>(((raw[1] & 0x0F) << 8) | raw[2]);
  const int16_t y = static_cast<int16_t>(((raw[3] & 0x0F) << 8) | raw[4]);

  sample.x = x;
  sample.y = y;
  sample.touched = true;
  sample.valid = true;
  return true;
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
  if (!readRegistersFrom(kImuAddress, kQmiWhoAmIReg, &whoAmI, 1) || whoAmI != kQmiWhoAmIValue) {
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
  if (!readRegistersFrom(kImuAddress, kQmiTempLowReg, raw, sizeof(raw))) {
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

void printTouchConfig() {
  Serial.println("Touch config:");
  Serial.println("  Interface: I2C shared with IMU");
  Serial.println("  Pins: SDA=48, SCL=47");
  Serial.println("  Polled controller address: 0x15");
}

void clearValueLine(uint16_t y) {
  gfx->fillRect(kValueColumnX, y, kValueFieldWidth, 10, BLACK);
}

void drawStaticScreen() {
  gfx->fillScreen(BLACK);
  gfx->setTextSize(1);
  gfx->setTextColor(CYAN);
  gfx->setCursor(8, 8);
  gfx->println("ESP32-S3-LCD-2 Test");

  gfx->setTextColor(WHITE);
  gfx->setCursor(8, 28);
  gfx->println("LCD: OK");
  gfx->println("IMU: QMI8658");
  gfx->println(touchControllerDetected ? "Touch: detected" : "Touch: not found");
  gfx->println("Sensor ODR: 2000 Hz");
  gfx->println("Host loop: 200 Hz");
  gfx->println("LPF: ~106 Hz");
  gfx->println("Accel: +/-8g");
  gfx->println("Gyro: +/-512 dps");

  gfx->setTextColor(YELLOW);
  gfx->setCursor(8, 110);
  gfx->println("AX:");
  gfx->setCursor(8, 122);
  gfx->println("AY:");
  gfx->setCursor(8, 134);
  gfx->println("AZ:");

  gfx->setTextColor(GREEN);
  gfx->setCursor(8, 146);
  gfx->println("GX:");
  gfx->setCursor(8, 158);
  gfx->println("GY:");
  gfx->setCursor(8, 170);
  gfx->println("GZ:");

  gfx->setTextColor(MAGENTA);
  gfx->setCursor(8, 182);
  gfx->println("Temp:");

  gfx->setTextColor(CYAN);
  gfx->setCursor(8, 206);
  gfx->println("Touch:");
  gfx->setCursor(8, 218);
  gfx->println("Rate:");

  screenLayoutDrawn = true;
}

void drawDynamicScreen(const ImuSample &sample) {
  if (!screenLayoutDrawn) {
    drawStaticScreen();
  }

  gfx->setTextColor(YELLOW);
  clearValueLine(110);
  gfx->setCursor(kValueColumnX, 110);
  gfx->printf("%7.3f g", sample.accelX);
  clearValueLine(122);
  gfx->setCursor(kValueColumnX, 122);
  gfx->printf("%7.3f g", sample.accelY);
  clearValueLine(134);
  gfx->setCursor(kValueColumnX, 134);
  gfx->printf("%7.3f g", sample.accelZ);

  gfx->setTextColor(GREEN);
  clearValueLine(146);
  gfx->setCursor(kValueColumnX, 146);
  gfx->printf("%7.2f dps", sample.gyroX);
  clearValueLine(158);
  gfx->setCursor(kValueColumnX, 158);
  gfx->printf("%7.2f dps", sample.gyroY);
  clearValueLine(170);
  gfx->setCursor(kValueColumnX, 170);
  gfx->printf("%7.2f dps", sample.gyroZ);

  gfx->setTextColor(MAGENTA);
  clearValueLine(182);
  gfx->setCursor(kValueColumnX, 182);
  gfx->printf("%6.2f C", sample.temperatureC);

  clearValueLine(206);
  gfx->setCursor(kValueColumnX, 206);
  if (!touchControllerDetected) {
    gfx->setTextColor(RED);
    gfx->print("N/A");
  } else if (!latestTouch.valid) {
    gfx->setTextColor(CYAN);
    gfx->print("init...");
  } else if (!latestTouch.touched) {
    gfx->setTextColor(CYAN);
    gfx->print("released");
  } else {
    gfx->setTextColor(CYAN);
    gfx->printf("x=%3d y=%3d", latestTouch.x, latestTouch.y);
  }

  clearValueLine(218);
  gfx->setCursor(kValueColumnX, 218);
  gfx->setTextColor(WHITE);
  gfx->printf("I:%4.1f T:%4.1f", imuRateHz, touchPollRateHz);
}

void printSample(const ImuSample &sample) {
  Serial.printf(
      "AX=%0.3f AY=%0.3f AZ=%0.3f g | GX=%0.2f GY=%0.2f GZ=%0.2f dps | T=%0.2f C\n",
      sample.accelX, sample.accelY, sample.accelZ,
      sample.gyroX, sample.gyroY, sample.gyroZ,
      sample.temperatureC);
}

void printTouchSample(const TouchSample &sample) {
  if (!touchControllerDetected || !sample.valid) {
    return;
  }
  if (sample.touched) {
    Serial.printf("TOUCH x=%d y=%d\n", sample.x, sample.y);
  }
}

void printRateReport() {
  Serial.printf("RATE imu=%0.1fHz touch_poll=%0.1fHz touch_active=%0.1fHz\n",
                imuRateHz, touchPollRateHz, touchActiveRateHz);
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
  Serial.begin(921600);
  delay(1000);
  Serial.println("Booting Waveshare ESP32-S3-LCD-2 hardware test");

  if (!initDisplay()) {
    haltWithMessage(F("LCD init failed"));
  }

  Wire.begin(kImuSda, kImuScl);
  Wire.setClock(400000);
  touchControllerDetected = isI2cDevicePresent(kTouchAddress);

  if (!initImu()) {
    haltWithMessage(F("IMU init failed"));
  }

  printImuConfig();
  printTouchConfig();
  drawStaticScreen();
  drawDynamicScreen(latestSample);
}

void loop() {
  uint32_t nowUs = micros();
  uint8_t imuCatchup = 0;
  while (static_cast<uint32_t>(nowUs - lastImuReadUs) >= kImuReadPeriodUs && imuCatchup < 4) {
    lastImuReadUs += kImuReadPeriodUs;
    if (readImuSample(latestSample)) {
      imuReadCount++;
    }
    imuCatchup++;
    nowUs = micros();
  }

  const uint32_t nowMs = millis();
  if (latestSample.valid && static_cast<uint32_t>(nowMs - lastSerialPrintMs) >= kSerialPrintPeriodMs) {
    lastSerialPrintMs = nowMs;
    printSample(latestSample);
  }

  if (touchControllerDetected && static_cast<uint32_t>(nowMs - lastTouchReadMs) >= kTouchReadPeriodMs) {
    lastTouchReadMs = nowMs;
    if (readTouchCst816Like(latestTouch)) {
      touchPollCount++;
      if (latestTouch.touched) {
        touchActiveCount++;
      }
    }
  }

  if (static_cast<uint32_t>(nowMs - lastTouchPrintMs) >= kSerialPrintPeriodMs) {
    lastTouchPrintMs = nowMs;
    printTouchSample(latestTouch);
  }

  if (latestSample.valid && static_cast<uint32_t>(nowMs - lastScreenRefreshMs) >= kScreenRefreshPeriodMs) {
    lastScreenRefreshMs = nowMs;
    drawDynamicScreen(latestSample);
  }

  if (static_cast<uint32_t>(nowMs - lastRateReportMs) >= 1000U) {
    const uint32_t dtMs = nowMs - lastRateReportMs;
    const float dt = dtMs / 1000.0f;
    lastRateReportMs = nowMs;

    if (dt > 0.0f) {
      imuRateHz = imuReadCount / dt;
      touchPollRateHz = touchPollCount / dt;
      touchActiveRateHz = touchActiveCount / dt;
    }
    imuReadCount = 0;
    touchPollCount = 0;
    touchActiveCount = 0;

    printRateReport();
  }
}
