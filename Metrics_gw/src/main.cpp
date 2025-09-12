#include <Arduino.h>
#include "LoraMesher.h"

// 0 = deshabilitat (no WiFi/MQTT), 1 = habilitat
#define GW_ENABLE_MQTT 0

// Optional WiFi/MQTT (gated by GW_ENABLE_MQTT)
#if GW_ENABLE_MQTT
#include <WiFi.h>
#include <PubSubClient.h>
#endif

// ==== Node identity (descomenta un) ====
// #define SELF_NODE 0x0E5C
// #define SELF_NODE 0x4D88
#define SELF_NODE 0x5758
//#define SELF_NODE 0x7250

// ==== Enviament reliable del "hello counter" (0/1) ====
#define HELLO_RELIABLE 0

// Llista completa de nodes
static const uint16_t ALL_NODES[] = { 0x0E5C, 0x4D88, 0x5758, 0x7250 };
static const size_t   NUM_NODES   = sizeof(ALL_NODES)/sizeof(ALL_NODES[0]);

//Using LILYGO TTGO T-BEAM v1.1 
#define BOARD_LED   4
#define LED_ON      LOW
#define LED_OFF     HIGH

#ifndef BROADCAST_ADDR
#define BROADCAST_ADDR 0xFFFF
#endif

LoraMesher& radio = LoraMesher::getInstance();

struct dataPacket {
    
    uint32_t counter = 0;
};

// ===== MQTT (gated) =====
#if GW_ENABLE_MQTT

#ifndef WIFI_SSID
#define WIFI_SSID    "MIWIFI_2G_X9sC"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS    "4GCUCdmu"
#endif

#ifndef MQTT_BROKER
#define MQTT_BROKER  "192.168.1.140"
#endif
#ifndef MQTT_PORT
#define MQTT_PORT    1883
#endif
#ifndef MQTT_USER
#define MQTT_USER    ""
#endif
#ifndef MQTT_PASSWD
#define MQTT_PASSWD  ""
#endif

#ifndef MQTT_RETAINED
#define MQTT_RETAINED 0
#endif

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
TaskHandle_t mqttTaskHandle = NULL;

static void wifiConnect() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; ++i) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] OK %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("[WiFi] FAILED");
    }
}

static bool mqttConnect() {
    if (!WiFi.isConnected()) wifiConnect();
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    String cid = String("gw-") + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.printf("[MQTT] Connecting to %s as %s ... ", MQTT_BROKER, cid.c_str());
    bool ok = MQTT_USER[0] ? mqtt.connect(cid.c_str(), MQTT_USER, MQTT_PASSWD)
                           : mqtt.connect(cid.c_str());
    Serial.println(ok ? "OK" : "FAIL");
    return ok;
}

static void mqttTask(void*) {
    for(;;) {
        if (!mqtt.connected()) mqttConnect();
        mqtt.loop();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void mqtt_pub_dades(uint16_t src, const char* payload) {
    if (!mqtt.connected()) return;
    char topic[64];
    snprintf(topic, sizeof(topic), "lora/node/%04X/data", src);
    mqtt.publish(topic, payload, MQTT_RETAINED);
    Serial.printf("[MQTT] -> %s : %s\n", topic, payload);
}

static void mqtt_pub_routes(uint16_t src, const char* payload) {
    if (!mqtt.connected()) return;
    char topic[64];
    snprintf(topic, sizeof(topic), "lora/node/%04X/routes", src);
    mqtt.publish(topic, payload, MQTT_RETAINED);
    Serial.printf("[MQTT] -> %s : %s\n", topic, payload);
}

#endif // GW_ENABLE_MQTT

//Led flash
void led_Flash(uint16_t flashes, uint16_t delaymS) {
    uint16_t index;
    for (index = 1; index <= flashes; index++) {
        digitalWrite(BOARD_LED, LED_OFF);
        delay(delaymS);
        digitalWrite(BOARD_LED, LED_ON);
        delay(delaymS);
    }
}

static void printRawOrJson(AppPacket<uint8_t>* packet) {
    uint8_t* raw = packet->payload;
    size_t len = packet->getPayloadLength();

    if (len > 0 && raw[0] == '{' && len < 512) {
        char* buf = (char*)malloc(len + 1);
        if (!buf) return;
        memcpy(buf, raw, len);
        buf[len] = '\0';
        Serial.printf("[GW] JSON from %04X: %s\n", packet->src, buf);

        // minimal whois_gw responder
        if (strstr(buf, "\"type\":\"whois_gw\"") != NULL) {
            char iam[128];
            snprintf(iam, sizeof(iam), "{\"type\":\"iam_gw\",\"gw\":\"0x%04X\"}", radio.getLocalAddress());
            size_t iam_len = strlen(iam) + 1;
            Serial.printf("[GW] -> iam_gw to %04X: %s\n", packet->src, iam);
            radio.createPacketAndSend(packet->src, (uint8_t*)iam, (uint8_t)iam_len);
        }

        // Print hello_counter if provided in JSON
        if (strstr(buf, "\"type\":\"hello_counter\"") != NULL) {
            uint32_t ctr = 0;
            const char* ck = strstr(buf, "\"counter\"");
            if (ck) {
                const char* c = strchr(ck, ':');
                if (c) {
                    unsigned long v = strtoul(c + 1, NULL, 10);
                    ctr = (uint32_t)v;
                }
            }
            Serial.printf("[GW] hello_counter from %04X: %u\n", packet->src, (unsigned)ctr);
        }

        // If MQTT enabled, forward route-related JSON and any JSON payload as data
#if GW_ENABLE_MQTT
        if (strstr(buf, "\"type\":\"route\"") ||
            strstr(buf, "\"type\":\"routes\"") ||
            strstr(buf, "\"type\":\"routes_begin\"") ||
            strstr(buf, "\"type\":\"routes_end\"")) {
            mqtt_pub_routes(packet->src, buf);
        } else {
            // generic JSON to data channel
            mqtt_pub_dades(packet->src, buf);
        }
#endif

        free(buf);
    } else {
        Serial.printf("[GW] BIN len=%u from %04X\n", (unsigned)len, packet->src);
        // Try parse as hello counter (binary dataPacket)
        if (len >= sizeof(dataPacket)) {
            dataPacket* dp = (dataPacket*)raw;
            uint32_t ctr = dp[0].counter;
            Serial.printf("[GW] hello_counter from %04X: %u\n", packet->src, (unsigned)ctr);
#if GW_ENABLE_MQTT
            char json[96];
            snprintf(json, sizeof(json), "{\"type\":\"hello_counter\",\"counter\":%u}", (unsigned)ctr);
            mqtt_pub_dades(packet->src, json);
#endif
        }
    }
}

/**
 * @brief Function that process the received packets
 *
 */
void processReceivedPackets(void*) {
    for (;;) {
        /* Wait for the notification of processReceivedPackets and enter blocking */
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);
        led_Flash(1, 100); //one quick LED flashes to indicate a packet has arrived

        //Iterate through all the packets inside the Received User Packets Queue
        while (radio.getReceivedQueueSize() > 0) {
            Serial.println("ReceivedUserData_TaskHandle notify received");
            Serial.printf("Queue receiveUserData size: %d\n", radio.getReceivedQueueSize());

            //Get the first element inside the Received User Packets Queue
            AppPacket<uint8_t>* packet = radio.getNextAppPacket<uint8_t>();

            //Print/Handle the packet
            if (packet) printRawOrJson(packet);

            //Delete the packet when used. It is very important to call this function to release the memory of the packet.
            if (packet) radio.deletePacket(packet);
        }
    }
}

TaskHandle_t receiveLoRaMessage_Handle = NULL;

/**
 * @brief Create a Receive Messages Task and add it to the LoRaMesher
 *
 */
void createReceiveMessages() {
    int res = xTaskCreate(
        processReceivedPackets,
        "Receive App Task",
        4096,
        (void*) 1,
        2,
        &receiveLoRaMessage_Handle);
    if (res != pdPASS) {
        Serial.printf("Error: Receive App Task creation gave error: %d\n", res);
    }
}


/**
 * @brief Initialize LoRaMesher
 *
 */
void setupLoraMesher() {
    //Get the configuration of the LoRaMesher
    LoraMesher::LoraMesherConfig config = LoraMesher::LoraMesherConfig();

    //Set the configuration of the LoRaMesher (HELTEC WIFI LORA V3.2 )
    config.loraCs = 8;
    config.loraRst = 12;
    config.loraIrq = 14;
    config.loraIo1 = 13;
  
    config.freq           = 868.3;    // MHz
    config.bw             = 125.0;    // kHz
    config.sf             = 9;        // SF9
    config.cr             = 5;        // 4/5 (RadioLib usa 5..8)
    config.syncWord       = 0x34;     // públic/LoRaWAN-like
    config.power          = 14;       // dBm
    config.preambleLength = 8;        // símbols

    config.module = LoraMesher::LoraModules::SX1262_MOD;

    //Init the loramesher with a configuration
    radio.begin(config);

    //Create the receive task and add it to the LoRaMesher
    createReceiveMessages();

    //Set the task handle to the LoRaMesher
    radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);

    //Start LoRaMesher
    radio.start();

    // Set Gateway role so other nodes can route to this node as GW
    radio.addGatewayRole();

    Serial.println("Lora initialized (GW)");
}


void setup() {
    Serial.begin(115200);

    Serial.println("initBoard");
    pinMode(BOARD_LED, OUTPUT); //setup pin as output for indicator LED
    led_Flash(2, 125);          //two quick LED flashes to indicate program start
    setupLoraMesher();

#if GW_ENABLE_MQTT
    // Start MQTT background if enabled
    wifiConnect();
    mqttConnect();
    xTaskCreatePinnedToCore(mqttTask, "mqtt", 4096, nullptr, 1, &mqttTaskHandle, 0);
#endif

    // Minimal periodic unicast iam_gw to help multi-hop route refresh
    auto gwUnicastTask = [](void*) {
        for(;;) {
            uint16_t self = radio.getLocalAddress();
            char iam[64];
            int n = snprintf(iam, sizeof(iam), "{\"type\":\"iam_gw\",\"gw\":\"0x%04X\"}", self);
            if (n > 0) {
                for (size_t i = 0; i < NUM_NODES; ++i) {
                    uint16_t dst = ALL_NODES[i];
                    if (dst == self) continue;
                    radio.createPacketAndSend(dst, (uint8_t*)iam, (uint8_t)(strlen(iam) + 1));
                }
            }
            vTaskDelay(60000 / portTICK_PERIOD_MS); // every 60s
        }
    };
    xTaskCreate(gwUnicastTask, "gw_unicast_iam", 3072, nullptr, 1, nullptr);
}


void loop() {
	for (;;) {
		// GW only receives and prints
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}