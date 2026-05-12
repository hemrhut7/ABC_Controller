# 硬體原理圖解析與腳位定義文件 (ESP32-S3-Touch-LCD-2)

## 1. 設備基本資訊 (Basic Device Information)
* **設備名稱 / 型號**: ESP32-S3-Touch-LCD-2
* **主控晶片 (MCU)**: ESP32-S3R8
* **文件類型**: 電路原理圖解析與腳位定義

---

## 2. 核心功能模塊 (Core Functional Modules)

* **微處理器 (MCU Area)**: 搭載 ESP32-S3R8 核心電路。
* **影像與顯示 (Imaging & Display)**:
    * **Camera (攝像頭)**: 具備專用的攝像頭感測器接口。
    * **LCD (液晶顯示器)**: 具備觸控顯示螢幕控制接口。
* **儲存與感測 (Storage & Sensing)**:
    * **SD Card (記憶卡)**: 支援外部 SD 卡讀寫擴充。
    * **IMU (慣性測量單元)**: 內建運動與姿態感測器。
* **電源與通訊介面 (Power & Interfaces)**:
    * **TYPE C**: USB Type-C 接口，提供供電與數據傳輸。
    * **Power (電源管理)**: 系統電壓轉換與穩壓電路。
    * **BAT (電池)**: 提供外部電池供電與管理接口。
* **擴充引腳 (Expansion IO)**:
    * **PinOut**: 實體排針擴充接口，引出多組 GPIO 供外部使用。

---

## 3. 實體排針位置對照表 (Physical Pin Layout)
根據電路圖中的「PinOut」定義，擴充介面分為左右兩排（P1 與 P2），共 28 個引腳，其對應的實際排列位置如下：

| 腳位編號 (Pin) | P1 排針 (左排 / Left Row) | P2 排針 (右排 / Right Row) |
| :---: | :--- | :--- |
| **1** | IO2 | 3V3 |
| **2** | IO4 | GND |
| **3** | IO6 | IO43 |
| **4** | IO16 | IO44 |
| **5** | IO17 | IO47 |
| **6** | IO18 | IO48 |
| **7** | IO21 | IO15 |
| **8** | IO8 | IO13 |
| **9** | IO7 | IO11 |
| **10** | IO10 | IO12 |
| **11** | IO20 | IO14 |
| **12** | IO19 | IO9 |
| **13** | GND | GND |
| **14** | 5V | VBAT |

---

## 4. 完整腳位功能對照表 (Complete PinOut Table)

| GPIO | Camera (攝像頭) | LCD (螢幕與觸控) | SD_Card (記憶卡) | Other (其他) | PinOut (擴充引腳) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **IO0** | | LCD_RST | | BOOT | |
| **IO1** | | LCD_BL | | | |
| **IO2** | CAM_D7 | | | | IO2 |
| **IO3** | | | | IMU_INT1 | |
| **IO4** | CAM_HREF | | | | IO4 |
| **IO5** | | | | BAT_ADC | |
| **IO6** | CAM_VSYNC | | | | IO6 |
| **IO7** | CAM_D6 | | | | IO7 |
| **IO8** | CAM_XCLK | | | | IO8 |
| **IO9** | CAM_PCLK | | | | IO9 |
| **IO10** | CAM_D5 | | | | IO10 |
| **IO11** | CAM_D3 | | | | IO11 |
| **IO12** | CAM_D0 | | | | IO12 |
| **IO13** | CAM_D1 | | | | IO13 |
| **IO14** | CAM_D4 | | | | IO14 |
| **IO15** | CAM_D2 | | | | IO15 |
| **IO16** | TWI_CLK | | | | IO16 |
| **IO17** | CAM_PWDN | | | | IO17 |
| **IO18** | | | | | IO18 |
| **IO19** | | | | USB_N | IO19 |
| **IO20** | | | | USB_P | IO20 |
| **IO21** | TWI_SDA | | | | IO21 |
| **IO38** | | LCD_MOSI | SD_MOSI | | |
| **IO39** | | LCD_SCLK | SD_SCLK | | |
| **IO40** | | | SD_MISO | | |
| **IO41** | | | SD_CS | | |
| **IO42** | | LCD_DC | | | |
| **IO43** | | | | U0_TXD | IO43 |
| **IO44** | | | | U0_RXD | IO44 |
| **IO45** | | LCD_CS | | | |
| **IO46** | | TP_INT | | | |
| **IO47** | | TP_SCL | | IMU_SCL | IO47 |
| **IO48** | | TP_SDA | | IMU_SDA | IO48 |

---

## 5. 依功能模組分類 (Module-Based Pin Mapping)

### 📷 Camera (攝像頭模組)
* **資料傳輸 (Data)**: IO12 (D0), IO13 (D1), IO15 (D2), IO11 (D3), IO14 (D4), IO10 (D5), IO7 (D6), IO2 (D7)
* **時鐘與同步 (Clock & Sync)**: IO8 (XCLK), IO9 (PCLK), IO6 (VSYNC), IO4 (HREF)
* **控制通訊 (I2C/Control)**: IO16 (TWI_CLK), IO21 (TWI_SDA), IO17 (PWDN)

### 🖥️ LCD (螢幕顯示與觸控模組)
* **顯示控制 (Display)**: IO0 (RST), IO1 (BL/背光), IO42 (DC), IO45 (CS)
* **顯示通訊 (SPI共用)**: IO38 (MOSI), IO39 (SCLK)
* **觸控通訊 (Touch I2C)**: IO46 (TP_INT), IO47 (TP_SCL), IO48 (TP_SDA)

### 💾 SD Card (記憶卡模組)
* **SPI 通訊**: IO41 (CS), IO40 (MISO)
* **與 LCD 共用 SPI 腳位**: IO38 (MOSI), IO39 (SCLK)

### ⚙️ Other (其他周邊控制)
* **慣性測量單元 (IMU)**: IO3 (IMU_INT1), IO47 (IMU_SCL), IO48 (IMU_SDA) *(註：SCL/SDA 與觸控面板共用)*
* **序列埠 (UART)**: IO43 (U0_TXD), IO44 (U0_RXD)
* **USB 通訊 (Type-C)**: IO19 (USB_N), IO20 (USB_P)
* **系統與電源**: IO0 (BOOT), IO5 (BAT_ADC)