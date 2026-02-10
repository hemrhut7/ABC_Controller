#include "hal_storage.h"

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
    data.pitch = {12.0f, 0.0f, 0.5f};
    data.rate  = {1.5f, 8.0f, 0.04f};
    data.yaw   = {2.0f, 0.0f, 0.0f};
    data.velocity = {0.1f, 0.01f, 0.0f};

    save_config(); // 寫入預設值
}

void ConfigStore::load_config() {
    EEPROM.get(EEPROM_ADDR, data);

    // 檢查 Magic Number，如果不符合代表是新晶片或資料損毀
    if (data.magic_number != CONFIG_MAGIC) {
        Serial.println("Config invalid, resetting to defaults...");
        reset_defaults();
    } else {
        Serial.println("Config loaded from EEPROM.");
    }
}

void ConfigStore::save_config() {
    _dirty = true;
}

void ConfigStore::update() {
    if (!_dirty) return;

    uint32_t now = millis();
    if (now - last_save_time > 1000) {
        last_save_time = now;
        _dirty = false;
        EEPROM.put(EEPROM_ADDR, data);
        
        // ESP 系列需要 commit 才會真正寫入 Flash
        #if defined(ESP32) || defined(ESP8266)
            EEPROM.commit();
        #endif
    }
}