#include "hal_storage.h"
#include <math.h>

ConfigStore::ConfigStore() {
    // 建構子不執行硬體操作
}

void ConfigStore::begin() {
    // 如果是 ESP32/ESP8266，需要指定大小 (例如 512 bytes)
    #if defined(ESP32) || defined(ESP8266)
        EEPROM.begin(512); 
    #endif
}

void ConfigStore::reset_defaults() {
    data.magic_number = CONFIG_MAGIC;

    // 設定預設 PID 參數
    data.motor = {1.0f, 5.0f, 0.0f};
    data.pitch = {12.0f, 0.0f, 0.5f};
    data.rate  = {1.5f, 8.0f, 0.04f};
    data.steer   = {2.0f, 0.0f, 0.0f};
    data.velocity = {0.1f, 0.01f, 0.0f};

    save_config(); // 寫入預設值
}

void ConfigStore::load_config() {
    EEPROM.get(EEPROM_ADDR, data);

    bool needs_reset = false;

    // 檢查 Magic Number，如果不符合代表是新晶片或資料損毀
    if (data.magic_number != CONFIG_MAGIC) {
        Serial.println("Config invalid (magic number mismatch), resetting to defaults...");
        needs_reset = true;
    } else if (isnan(data.motor.p) || isnan(data.motor.i) || isnan(data.motor.d) ||
               isnan(data.pitch.p) || isnan(data.pitch.i) || isnan(data.pitch.d) ||
               isnan(data.rate.p) || isnan(data.rate.i) || isnan(data.rate.d) ||
               isnan(data.steer.p) || isnan(data.steer.i) || isnan(data.steer.d) ||
               isnan(data.velocity.p) || isnan(data.velocity.i) || isnan(data.velocity.d)) {
        Serial.println("Config invalid (contains NaN), resetting to defaults...");
        needs_reset = true;
    }

    if (needs_reset) {
        reset_defaults();
    } else {
        Serial.println("Config loaded from EEPROM.");
    }
}

void ConfigStore::save_config() {
    // 為了避免過度寫入 Flash，只有在資料改變時才寫入
    SystemConfig old_data;
    EEPROM.get(EEPROM_ADDR, old_data);
    if (memcmp(&data, &old_data, sizeof(SystemConfig)) == 0) return;

    EEPROM.put(EEPROM_ADDR, data);

    // ESP 系列需要 commit 才會真正寫入 Flash
    #if defined(ESP32) || defined(ESP8266)
        EEPROM.commit();
    #endif
}
