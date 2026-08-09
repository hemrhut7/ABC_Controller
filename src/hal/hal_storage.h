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
    
    PID_Params motor;
    PID_Params pitch;
    PID_Params rate;
    PID_Params steer;
    PID_Params velocity;

    // 低通濾波器截止頻率 (Hz)
    float lpf_freq[4]; // 4 = LPF_ID_COUNT

    // 靜止狀態 Velocity PID 倍率
    PID_Params static_velocity_scale;
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

    // 重置為預設值
    void reset_defaults();

    // 提供給外部存取的設定實例
    SystemConfig data;

private:
    const uint32_t CONFIG_MAGIC = 0xCAFEBAAC; // 識別碼 (已更新以強制重置舊設定)
    const int EEPROM_ADDR = 0;                // 起始位址
    bool _dirty = false;
};
