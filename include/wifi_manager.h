#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <Preferences.h>
#include <ArduinoJson.h>

class WiFiManager {
public:
    WiFiManager();

    // Service mode: WiFi is off unless started from the serial console (Ethernet is the
    // normal link). begin() tries the saved station first, falls back to the AP.
    void begin();
    // Service mode off: radio off until the next begin().
    void stop();
    bool isStarted() const { return started; }

    // Update connection status (call in loop)
    void update();

    // Configuration
    void setStationCredentials(const char* ssid, const char* password);
    void setAPCredentials(const char* ssid, const char* password);
    void enableAP(bool enable);
    void enableStation(bool enable);

    // Status
    bool isConnected() const { return connected; }
    bool isAPMode() const { return apMode; }
    String getIPAddress() const;
    String getSSID() const;
    int getRSSI() const;

    // JSON for LIVI. Passwords are never reported, only whether a station password is stored.
    String statusJSON() const;
    // Validates, stores and persists. Takes effect on the next boot, never mid-drive.
    bool applyJSON(JsonObjectConst obj);

    // Save/Load configuration
    void saveConfig();
    void loadConfig();
    void clearConfig();  // Clear all saved WiFi credentials
    void clearStationCredentials();  // Clear only station credentials (keeps AP settings)

private:
    bool started = false;
    bool connected;
    bool apMode;
    bool stationEnabled;
    bool apEnabled;

    String stationSSID;
    String stationPassword;
    String apSSID;
    String apPassword;

    unsigned long lastConnectionAttempt;
    unsigned long reconnectInterval;

    // Joining the infotainment's network from the fallback AP, and the AP coming back.
    bool stationTrying = false;
    bool initialJoin = false;           ///< first join after boot, running in update()
    unsigned long initialJoinStarted = 0;
    static constexpr unsigned long INITIAL_JOIN_MS = 10000;
    unsigned long stationTryStarted = 0;
    unsigned long stationLostSince = 0;
    static constexpr unsigned long STATION_RETRY_MS = 30000;   ///< between tries from the AP
    static constexpr unsigned long STATION_TRY_MS = 12000;     ///< how long one try may take
    static constexpr unsigned long AP_AFTER_LOSS_MS = 60000;   ///< station gone this long: AP back

    Preferences preferences;

    void connectStation();
    void startMdns();
    void tryStationFromAP();
    void startAP();
    void checkConnection();
};

#endif // WIFI_MANAGER_H
