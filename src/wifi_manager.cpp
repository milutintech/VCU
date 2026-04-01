#include "wifi_manager.h"

WiFiManager::WiFiManager()
    : connected(false)
    , apMode(false)
    , stationEnabled(true)
    , apEnabled(true)
    , lastConnectionAttempt(0)
    , reconnectInterval(10000)  // 10 seconds
{
    // Default AP credentials
    uint64_t chipid = ESP.getEfuseMac();
    apSSID = "VCU_" + String((uint32_t)(chipid >> 32), HEX);
    apPassword = "vcu12345";
}

void WiFiManager::begin() {
    loadConfig();

    // Try station mode first if enabled and credentials exist
    if (stationEnabled && stationSSID.length() > 0) {
        Serial.println("[WiFi] Attempting Station mode...");
        connectStation();

        // Wait up to 10 seconds for connection
        unsigned long startTime = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < 10000) {
            delay(100);
        }
    }

    // Fall back to AP mode if station failed or not enabled
    if (WiFi.status() != WL_CONNECTED && apEnabled) {
        Serial.println("[WiFi] Station failed, starting AP mode...");
        startAP();
    }
}

void WiFiManager::update() {
    checkConnection();
}

void WiFiManager::connectStation() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(stationSSID.c_str(), stationPassword.c_str());
    apMode = false;
}

void WiFiManager::startAP() {
    WiFi.mode(WIFI_AP);

    // Configure AP with explicit IP settings for reliability
    IPAddress local_ip(192, 168, 4, 3);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);

    WiFi.softAPConfig(local_ip, gateway, subnet);

    // Start AP: channel 1, hidden=false, max_connections=4
    bool success = WiFi.softAP(apSSID.c_str(), apPassword.c_str(), 1, 0, 4);

    if (success) {
        apMode = true;
        connected = true;

        Serial.println("\n========================================");
        Serial.println("         WiFi AP Mode Started!");
        Serial.println("========================================");
        Serial.print("[WiFi] AP SSID: ");
        Serial.println(apSSID);
        Serial.print("[WiFi] AP Password: ");
        Serial.println(apPassword);
        Serial.print("[WiFi] AP IP Address: ");
        Serial.println(WiFi.softAPIP());
        Serial.print("[WiFi] AP MAC Address: ");
        Serial.println(WiFi.softAPmacAddress());
        Serial.println("========================================");
        Serial.println("Connect to the AP and navigate to:");
        Serial.println("http://192.168.4.1");
        Serial.println("========================================\n");
    } else {
        Serial.println("[WiFi] ERROR: Failed to start AP mode!");
        apMode = false;
        connected = false;
    }
}

void WiFiManager::checkConnection() {
    unsigned long now = millis();

    if (apMode) {
        // In AP mode, always consider connected
        connected = true;
        return;
    }

    // In station mode, check actual connection
    if (WiFi.status() == WL_CONNECTED) {
        if (!connected) {
            connected = true;
            Serial.print("[WiFi] Connected to: ");
            Serial.println(stationSSID);
            Serial.print("[WiFi] IP: ");
            Serial.println(WiFi.localIP());
        }
    } else {
        if (connected) {
            Serial.println("[WiFi] Connection lost!");
            connected = false;
        }

        // Attempt reconnection
        if (now - lastConnectionAttempt > reconnectInterval) {
            lastConnectionAttempt = now;
            Serial.println("[WiFi] Attempting reconnection...");
            WiFi.reconnect();
        }
    }
}

void WiFiManager::setStationCredentials(const char* ssid, const char* password) {
    stationSSID = ssid;
    stationPassword = password;
}

void WiFiManager::setAPCredentials(const char* ssid, const char* password) {
    apSSID = ssid;
    apPassword = password;
}

void WiFiManager::enableAP(bool enable) {
    apEnabled = enable;
}

void WiFiManager::enableStation(bool enable) {
    stationEnabled = enable;
}

String WiFiManager::getIPAddress() const {
    if (apMode) {
        return WiFi.softAPIP().toString();
    } else {
        return WiFi.localIP().toString();
    }
}

String WiFiManager::getSSID() const {
    if (apMode) {
        return apSSID;
    } else {
        return stationSSID;
    }
}

int WiFiManager::getRSSI() const {
    if (apMode) {
        return 0;
    } else {
        return WiFi.RSSI();
    }
}

void WiFiManager::saveConfig() {
    preferences.begin("wifi", false);
    preferences.putString("sta_ssid", stationSSID);
    preferences.putString("sta_pass", stationPassword);
    preferences.putString("ap_ssid", apSSID);
    preferences.putString("ap_pass", apPassword);
    preferences.putBool("sta_en", stationEnabled);
    preferences.putBool("ap_en", apEnabled);
    preferences.end();

    Serial.println("[WiFi] Configuration saved");
}

void WiFiManager::loadConfig() {
    preferences.begin("wifi", true);  // Read-only
    stationSSID = preferences.getString("sta_ssid", "");
    stationPassword = preferences.getString("sta_pass", "");

    // Only load AP credentials if they were customized
    String savedAPSSID = preferences.getString("ap_ssid", "");
    if (savedAPSSID.length() > 0) {
        apSSID = savedAPSSID;
    }
    String savedAPPassword = preferences.getString("ap_pass", "");
    if (savedAPPassword.length() > 0) {
        apPassword = savedAPPassword;
    }

    stationEnabled = preferences.getBool("sta_en", true);
    apEnabled = preferences.getBool("ap_en", true);
    preferences.end();

    Serial.println("[WiFi] Configuration loaded");
    if (stationSSID.length() > 0) {
        Serial.print("[WiFi] Station SSID: ");
        Serial.println(stationSSID);
    }
}

void WiFiManager::clearConfig() {
    preferences.begin("wifi", false);
    preferences.clear();  // Clear all WiFi preferences
    preferences.end();

    // Reset to defaults
    stationSSID = "";
    stationPassword = "";
    stationEnabled = true;
    apEnabled = true;

    // Keep default AP credentials (generated from chip ID)
    uint64_t chipid = ESP.getEfuseMac();
    apSSID = "VCU_" + String((uint32_t)(chipid >> 32), HEX);
    apPassword = "vcu12345";

    Serial.println("[WiFi] All WiFi configuration cleared!");
    Serial.println("[WiFi] Station credentials removed - will start in AP mode");
}

void WiFiManager::clearStationCredentials() {
    preferences.begin("wifi", false);
    preferences.remove("sta_ssid");
    preferences.remove("sta_pass");
    preferences.end();

    stationSSID = "";
    stationPassword = "";

    Serial.println("[WiFi] Station credentials cleared!");
    Serial.println("[WiFi] AP mode will be used on next boot");
}
