#include "wifi_manager.h"
#include <ESPmDNS.h>

static constexpr const char* MDNS_HOSTNAME = "vcu";

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

    // Reachable as vcu.local in either mode, so LIVI does not depend on the DHCP address.
    startMdns();
}

void WiFiManager::startMdns() {
    MDNS.end();   // after a network change the old announcement points nowhere
    if (MDNS.begin(MDNS_HOSTNAME)) {
        MDNS.addService("http", "tcp", 80);
        Serial.println("[WiFi] mDNS: vcu.local");
    } else {
        Serial.println("[WiFi] mDNS failed to start");
    }
}

/** From the fallback AP: try the saved network without taking the AP down first. */
void WiFiManager::tryStationFromAP() {
    Serial.printf("[WiFi] Trying to join \"%s\" from the AP...\n", stationSSID.c_str());
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(stationSSID.c_str(), stationPassword.c_str());
    stationTrying = true;
    stationTryStarted = millis();
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

    const bool haveStation = stationEnabled && stationSSID.length() > 0;

    if (apMode) {
        // In AP mode, always consider connected
        connected = true;

        if (stationTrying) {
            if (WiFi.status() == WL_CONNECTED) {
                // Joined: the AP is no longer needed, so it stops broadcasting.
                stationTrying = false;
                WiFi.softAPdisconnect(true);
                WiFi.mode(WIFI_STA);
                apMode = false;
                stationLostSince = 0;
                Serial.print("[WiFi] Joined ");
                Serial.print(stationSSID);
                Serial.print(", AP off. IP: ");
                Serial.println(WiFi.localIP());
                startMdns();
            } else if (now - stationTryStarted > STATION_TRY_MS) {
                // Not there (yet): stay an AP only and try again later.
                stationTrying = false;
                WiFi.disconnect(false);
                WiFi.mode(WIFI_AP);
                Serial.println("[WiFi] Join failed, staying in AP mode");
            }
        } else if (haveStation && now - lastConnectionAttempt > STATION_RETRY_MS &&
                   WiFi.softAPgetStationNum() == 0) {
            // Someone configuring the VCU through the AP is not disturbed by a scan.
            lastConnectionAttempt = now;
            tryStationFromAP();
        }
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
        stationLostSince = 0;
    } else {
        if (connected) {
            Serial.println("[WiFi] Connection lost!");
            connected = false;
        }
        if (stationLostSince == 0) stationLostSince = now;

        // Gone for good: bring the AP back so the VCU can still be reached.
        if (apEnabled && now - stationLostSince > AP_AFTER_LOSS_MS) {
            Serial.println("[WiFi] Station gone, starting the AP again");
            startAP();
            startMdns();
            stationLostSince = 0;
            lastConnectionAttempt = now;
            return;
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

String WiFiManager::statusJSON() const {
    JsonDocument doc;
    doc["mode"] = apMode ? "AP" : "STA";
    doc["connected"] = connected;
    doc["ip"] = getIPAddress();
    doc["rssi"] = getRSSI();
    doc["hostname"] = String(MDNS_HOSTNAME) + ".local";
    doc["staSsid"] = stationSSID;
    doc["hasStaPassword"] = stationPassword.length() > 0;
    doc["staEnabled"] = stationEnabled;
    doc["apSsid"] = apSSID;
    doc["apEnabled"] = apEnabled;
    String out;
    serializeJson(doc, out);
    return out;
}

bool WiFiManager::applyJSON(JsonObjectConst obj) {
    // Validate everything first so a bad request changes nothing.
    String newStaSsid = stationSSID;
    String newStaPass = stationPassword;
    String newApSsid = apSSID;
    String newApPass = apPassword;
    bool newStaEn = stationEnabled;

    if (!obj["staSsid"].isNull()) {
        if (!obj["staSsid"].is<const char*>()) return false;
        newStaSsid = obj["staSsid"].as<String>();
        if (newStaSsid.length() > 32) return false;
    }
    if (!obj["staPassword"].isNull()) {
        if (!obj["staPassword"].is<const char*>()) return false;
        newStaPass = obj["staPassword"].as<String>();
        // Empty means an open network, otherwise WPA2 length rules.
        if (newStaPass.length() != 0 && (newStaPass.length() < 8 || newStaPass.length() > 63)) return false;
    }
    if (!obj["staEnabled"].isNull()) {
        if (!obj["staEnabled"].is<bool>()) return false;
        newStaEn = obj["staEnabled"].as<bool>();
    }
    if (!obj["apSsid"].isNull()) {
        if (!obj["apSsid"].is<const char*>()) return false;
        newApSsid = obj["apSsid"].as<String>();
        if (newApSsid.length() < 1 || newApSsid.length() > 32) return false;
    }
    if (!obj["apPassword"].isNull()) {
        if (!obj["apPassword"].is<const char*>()) return false;
        newApPass = obj["apPassword"].as<String>();
        // The AP must stay protected: it is the way back in when joining fails.
        if (newApPass.length() < 8 || newApPass.length() > 63) return false;
    }
    // Joining is only meaningful with a network name.
    if (newStaEn && newStaSsid.length() == 0) newStaEn = false;

    stationSSID = newStaSsid;
    stationPassword = newStaPass;
    apSSID = newApSsid;
    apPassword = newApPass;
    stationEnabled = newStaEn;
    // The AP is the fallback and stays on; LIVI offers no way to switch it off.
    apEnabled = true;
    saveConfig();
    return true;
}
