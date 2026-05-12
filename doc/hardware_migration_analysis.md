# 硬體遷移分析報告：切換至 ESP32-S3-LCD-2

本文件分析了將現有專案從通用 ESP32 開發板遷移至 **Waveshare ESP32-S3-LCD-2** 的可行性、接腳規劃及軟體調整建議。

## 1. 現有系統接腳分析 (ESP32)

目前系統總共使用了約 21 個外部 GPIO 接腳，分佈如下：

| 功能模組 | 接腳數量 | 關鍵接腳 (目前) |
| :--- | :--- | :--- |
| **馬達控制 (L/R)** | 11 | PWM(2), DIR(4), Encoder(4), Standby(1) |
| **IMU (MPU6050)** | 2 | I2C (SDA: 23, SCL: 22) |
| **顯示器 (SSD1306)** | 4 | Software SPI (MOSI: 13, SCLK: 14, DC: 27, RST: 12) |
| **電池偵測** | 1 | ADC (GPIO 35) |
| **序列遙測 (UART1)** | 2 | TX: 1, RX: 3 (或自定義) |
| **系統指示燈** | 1 | LED_BUILTIN |

---

## 2. ESP32-S3-LCD-2 遷移優勢

1.  **高度整合**：內建 1.47 吋彩色 LCD (ST7789) 與六軸 IMU (QMI8658)，不佔用外部排針接腳。
2.  **接腳節省**：遷移後可釋放原本用於 I2C IMU 和 SPI OLED 的 **6 個 GPIO**。
3.  **性能提升**：ESP32-S3 具有更強的運算能力 (Xtensa® 32-bit LX7) 與更多的硬體 PWM (LEDC) 及脈衝計數 (PCNT) 單元。
4.  **電路簡化**：內建鋰電池充電電路與多種感測器，可大幅減少杜邦線連接，提高機器人運動時的硬體穩定性。

---

## 3. 建議接腳規劃 (Migrated Pin Map)

ESP32-S3-LCD-2 引出了 22 個可用 GPIO，以下為建議的分配方案，優先考慮配線的緊湊性：

### 外部排針分配

| GPIO | 功能定義 | 建議連接對象 | 備註 |
| :--- | :--- | :--- | :--- |
| **GPIO 2** | Motor_L_PWM | 左馬達速度 | 支援硬體 PWM |
| **GPIO 4** | Motor_L_DIR1 | 左馬達方向 1 | |
| **GPIO 6** | Motor_L_DIR2 | 左馬達方向 2 | |
| **GPIO 16** | Motor_L_EncA | 左馬達編碼器 A | 支援硬體 PCNT |
| **GPIO 17** | Motor_L_EncB | 左馬達編碼器 B | 支援硬體 PCNT |
| **GPIO 18** | Motor_R_PWM | 右馬達速度 | 支援硬體 PWM |
| **GPIO 8** | Motor_R_DIR1 | 右馬達方向 1 | |
| **GPIO 7** | Motor_R_DIR2 | 右馬達方向 2 | |
| **GPIO 10** | Motor_R_EncA | 右馬達編碼器 A | 支援硬體 PCNT |
| **GPIO 21** | Motor_R_EncB | 右馬達編碼器 B | 支援硬體 PCNT |
| **GPIO 15** | Motor_STBY | 馬達驅動器致能 | 全局 Standby |
| **GPIO 9** | Battery_ADC | 電池電壓監測 | ADC1_CH8 |
| **GPIO 43** | UART1_TX | 外部通訊/除錯 | Serial1 |
| **GPIO 44** | UART1_RX | 外部通訊/除錯 | Serial1 |

### 剩餘可用接腳 (可供未來擴充)
*   **GPIO 11, 12, 13, 14**：可用於超音波模組、蜂鳴器、自定義按鈕等。
*   **GPIO 47, 48**：可用於第二組 I2C 或 UART 設備。
*   **GPIO 19, 20**：USB 專用腳位 (若不使用 USB 通訊可轉作 GPIO)。

---

## 4. 軟體修改清單

為了適應新硬體，需對 `src/hal/` 下的驅動進行以下更新：

1.  **`hal_imu.cpp`**：
    *   更換庫文件：從 `Adafruit_MPU6050` 切換至 `QMI8658` 驅動。
    *   校準參數：需重新針對 QMI8658 進行加速度與陀螺儀偏置校準。
2.  **`hal_display.cpp`**：
    *   更換庫文件：從 `Adafruit_SSD1306` 切換至 `TFT_eSPI` 或 `Arduino_GFX`。
    *   調整 UI：解析度從 128x32 提升至 320x240，可顯示更多圖像化資訊。
3.  **`hal_motor.h`**：
    *   更新 `#define` 中的腳位編號至上述規劃。
4.  **`platformio.ini`**：
    *   更新 `board` 型號為 `esp32-s3-devkitc-1` (或對應的 Waveshare 設定)。
    *   更新引腳庫依賴。

---

## 5. 總結

**結論：遷移方案完全可行。**
遷移至 ESP32-S3-LCD-2 不僅能解決目前接腳緊張的問題，還能利用彩色螢幕提升人機互動體驗，並透過內建 IMU 減少系統噪訊，建議作為下一代控制器的硬體基準。
