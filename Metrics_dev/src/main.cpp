#include <Arduino.h>
#include "LoraMesher.h"
#include <ctype.h>

// ==== Node identity (descomenta un) ====
//#define SELF_NODE 0x0E5C
#define SELF_NODE 0x4D88
//#define SELF_NODE 0x5758
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

LoraMesher& radio = LoraMesher::getInstance();

#ifndef BROADCAST_ADDR
#define BROADCAST_ADDR 0xFFFF
#endif
#define LORA_MAX_PAYLOAD 91

uint32_t dataCounter = 0;
struct dataPacket {
    
    uint32_t counter = 0;
};

dataPacket* helloPacket = new dataPacket;

// ==== TFS20/TFmini sensor (UART + enable pin) ====
#define ENABLE_TFS20 1
#define TFS20_POLL_MS 60000
#define TFS20_BAUD    9600
#define TFS20_RX_PIN  40     // LIDAR TX -> MCU RX
#define TFS20_TX_PIN  -1     // no TX
#define TFS20_EN_PIN  5      // GPIO per alimentar/enable (configurable segons maquinari)

#if ENABLE_TFS20
#include <HardwareSerial.h>
HardwareSerial TFS20Serial(2);
#endif

// (Sense cache): lectura sota demanda per evitar enviar valors antics

//  discovery flags/state
static volatile bool gw_ready = false;
static uint16_t known_gw_addr = 0;
static unsigned long nextDataAt = 0;
static unsigned long nextRoutesAt = 0;
static unsigned long nextWhoisAt = 0;

static inline bool isSelf(uint16_t a) { return a == SELF_NODE; }

// JSON helper and routing sender
static inline bool lora_send_json(uint16_t dst, const char* json) {
    if (!json || !json[0]) return false;
    size_t n = strlen(json) + 1; // include NUL
    if (n > LORA_MAX_PAYLOAD) {
        Serial.printf("[drop] payload %uB > %uB, ignored: %s\n", (unsigned)n, (unsigned)LORA_MAX_PAYLOAD, json);
        return false;
    }
    radio.createPacketAndSend(dst, (uint8_t*)json, (uint8_t)n);
    return true;
}

//

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

/**
 * @brief Print the counter of the packet
 *
 * @param data
 */
void printPacket(dataPacket data) {
    Serial.printf("Hello Counter received nº %d\n", data.counter);
}

//

static void sendWhoIsGW() {
    char buf[96];
    int n = snprintf(buf, sizeof(buf), "{\"type\":\"whois_gw\",\"node\":\"0x%04X\"}", radio.getLocalAddress());
    if (n > 0) {
        Serial.printf("[whois_gw] broadcast -> %s\n", buf);
        lora_send_json(BROADCAST_ADDR, buf);
    }
}

static void sendRoutesPerEntry() {
    if (!gw_ready || known_gw_addr == 0) {
        Serial.println("[routes] GW unknown; requesting...");
        sendWhoIsGW();
        return;
    }

    LM_LinkedList<RouteNode>* rt = radio.routingTableListCopy();
    if (!rt) return;
    rt->setInUse();

    const uint16_t me = radio.getLocalAddress();
    const uint16_t dst = known_gw_addr;

    for (int i = 0; i < radio.routingTableSize(); i++) {
        RouteNode* r = (*rt)[i];
        if (!r) continue;
        NetworkNode n = r->networkNode;

        char buf[96];
        int len = snprintf(buf, sizeof(buf), "{\"type\":\"route\",\"node\":\"0x%04X\",\"dst\":\"0x%04X\",\"via\":\"0x%04X\",\"metric\":%d,\"rsnr\":%d}", me, n.address, r->via, n.metric, r->receivedSNR);
        if (len > 0) {
            Serial.printf("[route] -> %04X %s\n", dst, buf);
            lora_send_json(dst, buf);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }

    rt->releaseInUse();
    delete rt;
}

#if ENABLE_TFS20
static void setupTFS20() {
    pinMode(TFS20_EN_PIN, OUTPUT);
    digitalWrite(TFS20_EN_PIN, LOW); // apagat per defecte
    pinMode(TFS20_RX_PIN, INPUT_PULLUP);
    TFS20Serial.setRxBufferSize(512);
    TFS20Serial.begin(TFS20_BAUD, SERIAL_8N1, TFS20_RX_PIN, TFS20_TX_PIN);
    Serial.printf("[TFS20] init UART baud=%u RX=%d TX=%d EN=%d\n",
                  (unsigned)TFS20_BAUD, (int)TFS20_RX_PIN, (int)TFS20_TX_PIN, (int)TFS20_EN_PIN);
}

// Parser local sense estat per evitar lectures encallades
#define TF_FRAME_LEN 9
static bool readTFminiFrameUntil(uint8_t frame[TF_FRAME_LEN], unsigned long deadlineMs) {
    uint8_t buf[TF_FRAME_LEN];
    int idx = 0;
    while ((long)(deadlineMs - millis()) > 0) {
        while (TFS20Serial.available()) {
            uint8_t b = (uint8_t)TFS20Serial.read();
            if (idx == 0) { if (b != 0x59) continue; buf[idx++] = b; continue; }
            if (idx == 1) { if (b != 0x59) { idx = 0; continue; } buf[idx++] = b; continue; }
            buf[idx++] = b;
            if (idx < TF_FRAME_LEN) continue;
            uint8_t sum = 0; for (int i=0;i<8;i++) sum += buf[i];
            if (sum == buf[8]) { memcpy(frame, buf, TF_FRAME_LEN); return true; }
            idx = 0;
            if (b == 0x59) buf[idx++] = 0x59;
        }
        vTaskDelay(1);
    }
    return false;
}

static bool tfs20_read_once(int& dist_mm, unsigned& strength, float& tempC) {
    // Alimenta sensor i neteja buffer
    digitalWrite(TFS20_EN_PIN, HIGH);
    vTaskDelay(150 / portTICK_PERIOD_MS);
    while (TFS20Serial.available()) (void)TFS20Serial.read();

    uint8_t frame[TF_FRAME_LEN];
    bool ok = readTFminiFrameUntil(frame, millis() + 1500);

    // Apaga sensor per estalvi independentment del resultat
    digitalWrite(TFS20_EN_PIN, LOW);

    if (!ok) return false;

    uint16_t dist_cm  = ((uint16_t)frame[3] << 8) | frame[2];
    uint16_t streng   = ((uint16_t)frame[5] << 8) | frame[4];
    float temperature = (((uint16_t)frame[7] << 8) | frame[6]) / 8.0f - 256.0f;

    dist_mm  = (int)dist_cm * 10;
    strength = (unsigned)streng;
    tempC    = temperature;
    return true;
}

// Sense tasca en segon pla: lectura sota demanda a loop()
#endif

/**
 * @brief Iterate through the payload of the packet and print the counter of the packet
 *
 * @param packet
 */
static void handleIncomingPacket(AppPacket<uint8_t>* packet) {
    Serial.printf("Packet arrived from %X with size %d\n", packet->src, packet->payloadSize);

    uint8_t* raw = packet->payload;
    size_t len = packet->getPayloadLength();

    if (len > 0 && raw[0] == '{' && len < 256) {
        char* s = (char*)malloc(len + 1);
        if (s) {
            memcpy(s, raw, len);
            s[len] = '\0';
            if (strstr(s, "\"type\":\"iam_gw\"") != NULL) {
                // try parse gw field, else use src
                const char* gk = strstr(s, "\"gw\"");
                uint16_t gw = packet->src;
                if (gk) {
                    const char* colon = strchr(gk, ':');
                    if (colon) {
                        while (*++colon && isspace((unsigned char)*colon)) {}
                        if (*colon == '\"') colon++;
                        char* endp = NULL;
                        unsigned long v = strtoul(colon, &endp, 0);
                        if (endp != colon) gw = (uint16_t)v;
                    }
                }
                known_gw_addr = gw;
                gw_ready = (known_gw_addr != 0);
                Serial.printf("[gw] Known GW set to %04X\n", known_gw_addr);
            } else {
                Serial.printf("[rx-json] from %04X: %s\n", packet->src, s);
            }
            free(s);
        }
        return;
    }

    if (len >= sizeof(dataPacket)) {
        dataPacket* dPacket = (dataPacket*)raw;
        size_t payloadLength = len / sizeof(dataPacket);
        for (size_t i = 0; i < payloadLength; i++) {
            printPacket(dPacket[i]);
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

            //Handle the packet (JSON or binary)
            if (packet) handleIncomingPacket(packet);

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

    Serial.println("Lora initialized");
}


void setup() {
    Serial.begin(115200);

    Serial.println("initBoard");
    pinMode(BOARD_LED, OUTPUT); //setup pin as output for indicator LED
    led_Flash(2, 125);          //two quick LED flashes to indicate program start
    setupLoraMesher();

    // initial GW discovery
    sendWhoIsGW();

#if ENABLE_TFS20
    setupTFS20();
#endif
}


void loop() {
	for (;;) {

		unsigned long now = millis();

		// 1) GW discovery retry cada 60s si cal
		if (!gw_ready && (long)(now - nextWhoisAt) >= 0) {
			sendWhoIsGW();
			nextWhoisAt = now + 60000UL;
		}

		// 2) Enviament de dades (counter + lectura TFS20) només al GW, cada TFS20_POLL_MS
		if (gw_ready && (long)(now - nextDataAt) >= 0) {
			uint32_t ctr = ++dataCounter;
			char buf[128];
#if ENABLE_TFS20
			int dist_mm = 0; unsigned strength = 0; float tempC = 0.0f;
			bool ok = tfs20_read_once(dist_mm, strength, tempC);
			if (ok) {
				snprintf(buf, sizeof(buf),
					 "{\"type\":\"hello_counter\",\"counter\":%u,\"dist_mm\":%d,\"strength\":%u,\"temp_C\":%.2f}",
					 (unsigned)ctr, dist_mm, strength, (double)tempC);
			} else {
				snprintf(buf, sizeof(buf),
					 "{\"type\":\"hello_counter\",\"counter\":%u,\"read_error\":\"can't read value\"}",
					 (unsigned)ctr);
			}
#else
			snprintf(buf, sizeof(buf),
				 "{\"type\":\"hello_counter\",\"counter\":%u,\"read_error\":\"sensor_disabled\"}",
				 (unsigned)ctr);
#endif
			Serial.printf("[data] -> GW %04X : %s\n", known_gw_addr, buf);
			lora_send_json(known_gw_addr, buf);
			nextDataAt = now + TFS20_POLL_MS;
		}

		// 3) Enviament de rutes cada 5 minuts
		if (gw_ready && (long)(now - nextRoutesAt) >= 0) {
			sendRoutesPerEntry();
			nextRoutesAt = now + 300000UL;
		}

		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
}
