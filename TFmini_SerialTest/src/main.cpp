#include <Arduino.h>

#ifndef TF_RX_PIN
#define TF_RX_PIN 40   // LIDAR TX -> MCU RX (Heltec V3 recomanat)
#endif
#ifndef TF_TX_PIN
#define TF_TX_PIN -1   // sense TX per llegir
#endif
#ifndef TF_BAUD
#define TF_BAUD   9600  // o 9600 segons el teu TFmini
#endif

#define TF_FRAME_LEN 9
HardwareSerial TF(2);

static bool readTFminiFrame(uint8_t frame[TF_FRAME_LEN]) {
  // no bloquejant: processa bytes disponibles i retorna quan troba un frame; fals si no
  static uint8_t buf[TF_FRAME_LEN];
  static int idx = 0;
  while (TF.available()) {
    uint8_t b = (uint8_t)TF.read();
    if (idx == 0) { if (b != 0x59) continue; buf[idx++] = b; continue; }
    if (idx == 1) { if (b != 0x59) { idx = 0; continue; } buf[idx++] = b; continue; }
    buf[idx++] = b;
    if (idx < TF_FRAME_LEN) continue;
    uint8_t sum = 0; for (int i=0;i<8;i++) sum += buf[i];
    if (sum == buf[8]) { memcpy(frame, buf, TF_FRAME_LEN); idx = 0; return true; }
    // resync si checksum falla
    idx = 0; if (b == 0x59) buf[idx++] = 0x59;
  }
  return false;
}

static void printRawAndProcessed(const uint8_t frame[TF_FRAME_LEN]) {
  Serial.print("RAW: ");
  for (int i = 0; i < TF_FRAME_LEN; i++) Serial.printf("%02X ", frame[i]);
  uint16_t dist_cm  = ((uint16_t)frame[3] << 8) | frame[2];
  uint16_t strength = ((uint16_t)frame[5] << 8) | frame[4];
  float tempC       = (((uint16_t)frame[7] << 8) | frame[6]) / 8.0f - 256.0f;
  Serial.printf(" | dist=%ucm strength=%u temp=%.2fC\n", dist_cm, strength, tempC);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("TFmini Serial Test");
  TF.begin(TF_BAUD, SERIAL_8N1, TF_RX_PIN, TF_TX_PIN);
  Serial.printf("UART2 RX=%d TX=%d baud=%u\n", TF_RX_PIN, TF_TX_PIN, (unsigned)TF_BAUD);
}

void loop() {
  uint8_t frame[TF_FRAME_LEN];
  unsigned long until = millis() + 1000; // finestra 1s
  bool ok = false;
  while ((long)(until - millis()) > 0) {
    if (readTFminiFrame(frame)) { ok = true; break; }
    delay(1);
  }
  if (ok) printRawAndProcessed(frame);
  else Serial.println("waiting frames...");
}
