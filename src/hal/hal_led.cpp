#include "hal_led.h"

static TimerHandle_t xLedTimer = NULL;

void init_led() {
    pinMode(LED_BUILTIN, OUTPUT);
    xLedTimer = xTimerCreate(
        "LED_Timer",
        pdMS_TO_TICKS(100), 
        pdTRUE,                      
        (void*)0,                    
        vLedTimerCallback            
    );

    if (xLedTimer != NULL) {
        Serial.println("LED Timer created successfully.");
        xTimerStart(xLedTimer, 0); 
    } else {
        Serial.println("LED Timer creation failed!");
    }
}


void vLedTimerCallback(TimerHandle_t xTimerHandle){
    led_toggle();
}

void led_toggle(){
    int current_state = digitalRead(LED_BUILTIN);
    digitalWrite(LED_BUILTIN, !current_state);
}


void led_blink_1sec_block(){
    pinMode(LED_BUILTIN, OUTPUT);
    uint32_t last_time = millis();
    while (millis() - last_time <= 1000) {
        led_toggle();
        delay(100);
    }
}

void set_led_state(SYSTEM_STATE state){
    if (xLedTimer == NULL) {
        return;
    }

    switch (state)
    {
    case INITIALIZING:
        xTimerStart(xLedTimer, 0);
        xTimerChangePeriod(xLedTimer, pdMS_TO_TICKS(100), 0);        
        break;
    case IMU_MEASURING:
        xTimerStart(xLedTimer, 0);
        xTimerChangePeriod(xLedTimer, pdMS_TO_TICKS(1000), 0);        
        break;
    case CONFIGURING:
        xTimerStop(xLedTimer, 0);
        digitalWrite(LED_BUILTIN, HIGH);        
    default:
        break;
    }
}

