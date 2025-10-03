#include <Arduino.h>

HardwareSerial RadarSerial(2);

const int RX_PIN = 15;
const int TX_PIN = 12;
const uint32_t BAUD_RATE = 115200;

// --- Comandos ---
uint8_t CMD_INIT[]      = {0x01, 0xC0, 0x00, 0x00, 0x00};
uint8_t CMD_MODE_TRIG[] = {0x01, 0xC0, 0x05, 0x00, 0x01, 0x01};
uint8_t CMD_READ[]      = {0x01, 0xC0, 0x02, 0x00, 0x00};

void sendCommand(const uint8_t *cmd, size_t len) {
  //Serial.print("Enviando comando: ");
  /*for (size_t i = 0; i < len; i++) {
    Serial.print("0x");
    if (cmd[i] < 16) Serial.print("0");
    Serial.print(cmd[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  */
  RadarSerial.write(cmd, len);
  RadarSerial.flush();
}

float bytesToFloat(const uint8_t *data) {
  uint32_t v = (uint32_t)data[0] |
               ((uint32_t)data[1] << 8) |
               ((uint32_t)data[2] << 16) |
               ((uint32_t)data[3] << 24);
  float f;
  memcpy(&f, &v, sizeof(float));
  return f;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== HLK-LD8001 (15 segundos) ===");

  RadarSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  delay(300);
  sendCommand(CMD_INIT, sizeof(CMD_INIT));
  delay(3000);
  sendCommand(CMD_MODE_TRIG, sizeof(CMD_MODE_TRIG));
  //Serial.println("Configuração concluída!");
}

void loop() {
  static unsigned long lastTrigger = 0;
  const unsigned long INTERVAL = 15000;
  
  if (millis() - lastTrigger >= INTERVAL) {
    sendCommand(CMD_READ, sizeof(CMD_READ));
    lastTrigger = millis();
  }

  // Buffer maior e lógica melhorada
  static uint8_t buf[256]; // Aumentado de 256 para 512
  static size_t idx = 0;
  static unsigned long lastDataTime = 0;
  static bool waitingForData = false;

  // Só processa dados se acabou de enviar comando
  if (millis() - lastTrigger < 2000) { // 2 segundos após envio do comando
    waitingForData = true;
  } else {
    waitingForData = false;
  }

  while (RadarSerial.available() && waitingForData) {
    int b = RadarSerial.read();
    if (b < 0) break;
    
    buf[idx++] = (uint8_t)b;
    lastDataTime = millis();
    
    // Buffer overflow mais inteligente
    if (idx >= sizeof(buf)) {
      //Serial.println("Buffer cheio - procurando frame...");
      // Procura frame antes de limpar
      for (size_t i = 0; i + 11 < idx; i++) {
        if (buf[i] == 0x01 && buf[i+5] == 0x0A && buf[i+6] == 0x0C) {
          if (i + 11 < idx) {
            float dist = bytesToFloat(&buf[i+8]);
            Serial.print("Distância: ");
            Serial.print(dist, 4);
            Serial.println(" m");
            break;
          }
        }
      }
      idx = 0; // Limpa buffer
    }
  }

  // Limpa buffer se não há dados há mais de 3 segundos
  if (idx > 0 && millis() - lastDataTime > 3000) {
    //Serial.println("Timeout - limpando buffer");
    idx = 0;
  }

  // Parse dos dados quando não está esperando novos dados
  if (idx >= 12 && !waitingForData) {
    for (size_t i = 0; i + 11 < idx; i++) {
      if (buf[i] == 0x01 && buf[i+5] == 0x0A && buf[i+6] == 0x0C) {
        if (i + 11 < idx) {
          float dist = bytesToFloat(&buf[i+8]);
          Serial.print("Distância: ");
          Serial.print(dist, 4);
          Serial.println(" m");
          idx = 0;
          break;
        }
      }
    }
  }
}