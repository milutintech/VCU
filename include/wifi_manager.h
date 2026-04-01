#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <Preferences.h>

class WiFiManager {
public:
    WiFiManager();

    // Initialize WiFi (tries Station mode first, falls back to AP)
    void begin();

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

    // Save/Load configuration
    void saveConfig();
    void loadConfig();
    void clearConfig();  // Clear all saved WiFi credentials
    void clearStationCredentials();  // Clear only station credentials (keeps AP settings)

private:
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

    Preferences preferences;

    void connectStation();
    void startAP();
    void checkConnection();
};

#endif // WIFI_MANAGER_H
