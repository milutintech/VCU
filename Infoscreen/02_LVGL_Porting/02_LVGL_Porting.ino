#include <Arduino.h>
#include <ESP_Panel_Library.h>
#include <lvgl.h>
#include "lvgl_port_v8.h"
#include "ui.h"
#include "screens.h"
#include "actions.h"
#include <ESP_IOExpander_Library.h>
#include "HWCDC.h"
#include <WiFi.h>
#include <esp_now.h>

// HWCDC for USB Serial
HWCDC USBSerial;

// Global event variables
lv_event_t g_eez_event;
bool g_eez_event_is_available = false;

// IO expander configuration
#define EXAMPLE_CHIP_NAME TCA95xx_8bit
#define EXAMPLE_I2C_NUM (1)
#define EXAMPLE_I2C_SDA_PIN (8)
#define EXAMPLE_I2C_SCL_PIN (9)
#define _EXAMPLE_CHIP_CLASS(name, ...) ESP_IOExpander_##name(__VA_ARGS__)
#define EXAMPLE_CHIP_CLASS(name, ...) _EXAMPLE_CHIP_CLASS(name, ##__VA_ARGS__)

// Message types
#define MSG_TYPE_BMS 0x01
#define MSG_TYPE_DMC_TEMP 0x02

ESP_IOExpander *expander = NULL;

// Test values
uint8_t test_soc = 0;
float test_voltage = 48.0;
int16_t test_current = 0;
float test_temp = 25.0;

// ESP-NOW stats
uint32_t message_count = 0;
bool is_test_mode = false;  // false = ESP-NOW mode, true = test mode

// Custom indicator for SOC gauge
lv_meter_indicator_t *custom_soc_indicator = NULL;
lv_meter_scale_t *custom_soc_scale = NULL;
bool custom_indicator_created = false;

// Required function for action handling
void action_set_global_eez_event(lv_event_t* event) {
    g_eez_event = *event;
    g_eez_event_is_available = true;
}

// ESP-NOW callback - SIMPLIFIED VERSION USING INTEGERS
void OnDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    if (data_len < 1) return;
    
    message_count++;
    uint8_t msg_type = data[0];
    
    switch (msg_type) {
        case MSG_TYPE_BMS:
            if (data_len >= 6) {
                // Extract data using integer representation
                uint16_t voltageInt = (data[1] << 8) | data[2];
                test_voltage = voltageInt / 10.0;  // Convert back to float with 1 decimal
                
                // Extract signed current
                test_current = (int16_t)((data[3] << 8) | data[4]);
                
                // Direct SOC value
                test_soc = data[5];
                
                USBSerial.printf("ESP-NOW: BMS data - SOC:%d%% V:%.1fV I:%dA\n", 
                                 test_soc, test_voltage, test_current);
            }
            break;
            
        case MSG_TYPE_DMC_TEMP:
            if (data_len >= 5) {
                // Extract temperature data using integer representation
                uint16_t invTempInt = (data[1] << 8) | data[2];
                float temp_inverter = invTempInt / 10.0;  // Convert back to float
                
                uint16_t motorTempInt = (data[3] << 8) | data[4];
                float temp_motor = motorTempInt / 10.0;  // Convert back to float
                
                test_temp = max(temp_inverter, temp_motor);
                
                USBSerial.printf("ESP-NOW: DMC temp: Inverter=%.1f°C, Motor=%.1f°C, Max=%.1f°C\n", 
                                 temp_inverter, temp_motor, test_temp);
            }
            break;
    }
    
    // Update UI immediately when data is received
    updateUI();
}

// Create a custom indicator for the SOC gauge
void createCustomIndicator() {
    if (objects.main_gauge_soc != NULL && !custom_indicator_created) {
        USBSerial.println("Creating custom SOC indicator");
        
        // First get the scale
        custom_soc_scale = lv_meter_add_scale(objects.main_gauge_soc);
        
        // Set scale properties to match the original scale
        lv_meter_set_scale_ticks(objects.main_gauge_soc, custom_soc_scale, 41, 1, 5, lv_color_hex(0xffa8c685));
        lv_meter_set_scale_major_ticks(objects.main_gauge_soc, custom_soc_scale, 8, 3, 10, lv_color_hex(0xffa8c685), 10);
        lv_meter_set_scale_range(objects.main_gauge_soc, custom_soc_scale, 0, 100, 300, 120);
        
        // Create a custom needle indicator
        custom_soc_indicator = lv_meter_add_needle_line(objects.main_gauge_soc, custom_soc_scale, 5, lv_color_hex(0xffa8c686), -28);
        
        // Set initial value
        lv_meter_set_indicator_value(objects.main_gauge_soc, custom_soc_indicator, test_soc);
        
        custom_indicator_created = true;
        USBSerial.println("Custom SOC indicator created successfully");
    }
}

// Safe UI update function with custom indicator
void updateUI() {
    // Update voltage label - most important info
    if (objects.main_lable_volt != NULL) {
        char voltText[16];
        snprintf(voltText, sizeof(voltText), "%.1fV", test_voltage);
        lv_label_set_text(objects.main_lable_volt, voltText);
    } else {
        USBSerial.println("Warning: main_lable_volt is NULL");
    }
    
    // Update power bar
    if (objects.main_bar_pwr != NULL) {
        int16_t absCurrentValue = abs(test_current);
        uint8_t currentPercent = map(absCurrentValue, 0, 500, 0, 100);
        lv_bar_set_value(objects.main_bar_pwr, currentPercent, LV_ANIM_OFF);
    }
    
    // Update temperature bar
    if (objects.main_bar_temp != NULL) {
        uint8_t tempPercent = map(test_temp, 0, 110, 0, 100);
        lv_bar_set_value(objects.main_bar_temp, tempPercent, LV_ANIM_OFF);
    }
    
    // Update SOC gauge with custom indicator
    if (custom_indicator_created && objects.main_gauge_soc != NULL && custom_soc_indicator != NULL) {
        uint8_t safeSoc = constrain(test_soc, 0, 100);
        try {
            lv_meter_set_indicator_value(objects.main_gauge_soc, custom_soc_indicator, safeSoc);
        } catch (...) {
            USBSerial.println("Error: Exception updating custom indicator");
        }
    }
}

void setup() {
    USBSerial.begin(115200);
    delay(500);
    
    USBSerial.println("ESP32 + LVGL + ESP-NOW");
    
    // Initialize IO expander
    expander = new EXAMPLE_CHIP_CLASS(EXAMPLE_CHIP_NAME,
                                    (i2c_port_t)EXAMPLE_I2C_NUM, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
                                    EXAMPLE_I2C_SCL_PIN, EXAMPLE_I2C_SDA_PIN);
    expander->init();
    esp_err_t initStatus = expander->begin();

    if (initStatus != ESP_OK) {
        expander = new EXAMPLE_CHIP_CLASS(EXAMPLE_CHIP_NAME,
                                        (i2c_port_t)1, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
                                        7, 15);
        expander->init();
        expander->begin();
    }

    // Set up IO pins
    pinMode(16, OUTPUT);
    digitalWrite(16, LOW);
    expander->pinMode(5, OUTPUT);
    expander->digitalWrite(5, HIGH);
    expander->pinMode(0, OUTPUT);
    expander->digitalWrite(0, LOW);
    expander->pinMode(2, OUTPUT);
    expander->digitalWrite(2, LOW);
    delay(200);
    expander->digitalWrite(5, LOW);
    expander->digitalWrite(2, HIGH);
    expander->digitalWrite(0, HIGH);

    // Initialize display panel
    ESP_Panel *panel = new ESP_Panel();
    panel->init();
    #if LVGL_PORT_AVOID_TEAR
        ESP_PanelBus_RGB *rgb_bus = static_cast<ESP_PanelBus_RGB *>(panel->getLcd()->getBus());
        rgb_bus->configRgbFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
        rgb_bus->configRgbBounceBufferSize(LVGL_PORT_RGB_BOUNCE_BUFFER_SIZE);
    #endif
    panel->begin();

    // Initialize LVGL
    lvgl_port_init(panel->getLcd(), panel->getTouch());
    lvgl_port_lock(-1);
    lv_init();
    lvgl_port_unlock();
    
    // Initialize UI
    ui_init();
    
    // Wait a bit for UI to initialize before creating custom indicator
    delay(1000);
    
    // Create custom indicator for the gauge
    createCustomIndicator();
    
    // Initialize WiFi and ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    USBSerial.print("MAC: ");
    USBSerial.println(WiFi.macAddress());
    
    if (esp_now_init() != ESP_OK) {
        USBSerial.println("ESP-NOW init failed! Using test mode.");
        is_test_mode = true;
    } else {
        esp_now_register_recv_cb(OnDataReceived);
        USBSerial.println("ESP-NOW initialized");
    }
    
    USBSerial.println("READY");
}

void loop() {
    // Run LVGL tasks
    lv_task_handler();
    
    // LVGL UI tick
    ui_tick();
    
    // Process UI events
    if (g_eez_event_is_available) {
        g_eez_event_is_available = false;
    }
    
    // Test mode - cycle values if no ESP-NOW data
    if (is_test_mode) {
        static unsigned long last_update = 0;
        if (millis() - last_update > 2000) {
            test_soc = (test_soc + 5) % 101;
            test_voltage = 45.0 + (test_soc / 10.0);
            test_current = -250 + (test_soc * 5);
            test_temp = 25.0 + (test_soc * 0.5);
            last_update = millis();
            updateUI();  // Update UI in test mode
        }
    }
    
    // Heartbeat
    static unsigned long last_heartbeat = 0;
    if (millis() - last_heartbeat > 10000) {
        USBSerial.printf("Running: msgs=%lu, mode=%s\n", message_count, is_test_mode ? "TEST" : "ESP-NOW");
        last_heartbeat = millis();
    }
    
    delay(10);
}