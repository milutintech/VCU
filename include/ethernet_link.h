/**
 * @file ethernet_link.h
 * @brief W5500 Ethernet: the cable to the infotainment Pi.
 *
 * Fixed address, no DHCP (a direct cable has no server): VCU 192.168.5.1/24, Pi 192.168.5.2.
 * The W5500 shares SCK/MOSI/MISO with the MCP2515 (CS IO15, INT IO3, RST IO20).
 */

#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <ArduinoJson.h>

class EthernetLink {
public:
    /** Starts the W5500 and the network stack. Call before the web server starts listening. */
    bool begin(SPIClass& spi);

    bool started() const { return ok; }
    bool linkUp() const;
    String ip() const;

    /** `eth` block for /api/system/info. */
    void toJSON(JsonObject obj) const;

    /** Holds the W5500 in reset through deep sleep. Only to be called right before it. */
    static void holdForSleep();

    static constexpr uint8_t SPI_MHZ = 20;   ///< MCP2515 runs at 4 MHz on the same bus

private:
    bool ok = false;
};
