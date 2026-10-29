#include "ethernet_link.h"
#include <ETH.h>
#include <Network.h>
#include <ESPmDNS.h>
#include <driver/gpio.h>
#include "config.h"

static const IPAddress ETH_IP(192, 168, 5, 1);
static const IPAddress ETH_GATEWAY(0, 0, 0, 0);   // nothing behind the Pi is routed
static const IPAddress ETH_SUBNET(255, 255, 255, 0);

// The W5500 receive task. IDF creates it at priority 15 on either core, above the CAN task
// (5): while it reads a frame it would hold the shared bus with the MCP2515 (two receive
// buffers) waiting. Below the CAN task it only gets the bus when CAN has nothing to do.
static constexpr const char* W5500_TASK = "w5500_tsk";
static constexpr UBaseType_t W5500_TASK_PRIO = 4;

static void onNetworkEvent(arduino_event_id_t event) {
    switch (event) {
        case ARDUINO_EVENT_ETH_CONNECTED:
            Serial.printf("[ETH] Link up, %u Mbit/s %s duplex\n", ETH.linkSpeed(),
                          ETH.fullDuplex() ? "full" : "half");
            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            Serial.println("[ETH] Link down");
            break;
        default:
            break;
    }
}

bool EthernetLink::begin(SPIClass& spi) {
    gpio_hold_dis(static_cast<gpio_num_t>(Pins::ETH_RST));   // held low through deep sleep

    // Brings up the network stack even if the W5500 is missing, so the web server (and the
    // WiFi of service mode) still work.
    Network.begin();
    Network.onEvent(onNetworkEvent);

    if (!ETH.begin(ETH_PHY_W5500, 1, Pins::ETH_CS, Pins::ETH_INT, Pins::ETH_RST, spi, SPI_MHZ)) {
        Serial.println("[ETH] W5500 not found");
        return false;
    }
    ETH.config(ETH_IP, ETH_GATEWAY, ETH_SUBNET);

    TaskHandle_t rx = xTaskGetHandle(W5500_TASK);
    if (rx) {
        vTaskPrioritySet(rx, W5500_TASK_PRIO);
    } else {
        Serial.println("[ETH] W5500 task not found, priority left at 15");
    }

    // vcu.local, which the Pi's shutdown watcher uses. Also answers on WiFi in service mode.
    if (MDNS.begin("vcu")) MDNS.addService("http", "tcp", 80);

    ok = true;
    Serial.print("[ETH] W5500 up, IP ");
    Serial.print(ETH_IP);
    Serial.print(", MAC ");
    Serial.println(ETH.macAddress());
    return true;
}

bool EthernetLink::linkUp() const {
    return ok && ETH.linkUp();
}

String EthernetLink::ip() const {
    return ETH_IP.toString();
}

void EthernetLink::toJSON(JsonObject obj) const {
    obj["present"] = ok;
    obj["link"] = linkUp();
    obj["ip"] = ip();
    if (ok) {
        obj["mac"] = ETH.macAddress();
        obj["speedMbps"] = linkUp() ? ETH.linkSpeed() : 0;
    }
}

void EthernetLink::holdForSleep() {
    const gpio_num_t rst = static_cast<gpio_num_t>(Pins::ETH_RST);
    // gpio_* directly: the pin belongs to the ETH driver, digitalWrite would refuse it.
    gpio_set_direction(rst, GPIO_MODE_OUTPUT);
    gpio_set_level(rst, 0);
    gpio_hold_en(rst);
    gpio_deep_sleep_hold_en();
}
