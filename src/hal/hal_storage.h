#pragma once

#include <Arduino.h>
#include <EEPROM.h>

// 定義 PID 結構體，方便管理
struct PID_Params {
    float p;
    float i;
    float d;
};

// 定義所有需要儲存的設定
struct SystemConfig {
    uint32_t magic_number; // 用來檢查 EEPROM 是否已初始化 (例如 0xAABBCCDD)
    
    PID_Params pitch;
    PID_Params rate;
    PID_Params yaw;
    PID_Params velocity;
};

class ConfigStore {
public:
    ConfigStore();
    
    // 初始化 EEPROM
    void begin();

    // 載入設定，如果無效則載入預設值
    void load_config();

    // 儲存目前設定到 EEPROM
    void save_config();

    // 檢查並執行延遲寫入 (需在 Loop 中定期呼叫)
    void update();

    // 重置為預設值
    void reset_defaults();

    // 提供給外部存取的設定實例
    SystemConfig data;

private:
    const uint32_t CONFIG_MAGIC = 0xCAFEBABE; // 識別碼
    const int EEPROM_ADDR = 0;                // 起始位址
    uint32_t last_save_time = 0;
    bool _dirty = false;
};