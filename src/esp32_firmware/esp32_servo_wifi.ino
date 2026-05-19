/**
 * esp32_servo_wifi.ino
 *
 * Firmware do braço robótico — ESP32
 * Recebe comandos UDP do PC e aciona 6 servos.
 *
 * Protocolo (7 bytes):
 *   [0]   START_BYTE (0xAA)
 *   [1-6] ângulo de cada servo (uint8, 0-180°)
 *          0=base  1=ombro  2=cotovelo  3=pulso  4=rotacao_pulso  5=garra
 *
 * ACK de volta ao PC (4 bytes):
 *   [0]  START_BYTE (0xAA)
 *   [1]  STATUS  (0x00=OK | 0x01=RANGE_ERR | 0x02=BUSY)
 *   [2]  índice do último servo atualizado (0-5)
 *   [3]  checksum XOR de [0-2]
 *
 * Modos de rede:
 *   ACCESS POINT (padrão): o ESP32 cria a rede "BracoRobotico".
 *   Para conectar a uma rede existente, altere USE_AP_MODE para false
 *   e preencha WIFI_SSID / WIFI_PASS.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

#define USE_AP_MODE   true          // true = cria AP | false = conecta a uma rede

// Modo Access Point
const char* AP_SSID = "BracoRobotico";
const char* AP_PASS = "braco1234";    // mínimo 8 caracteres (WPA2)

// Modo Station (usado apenas se USE_AP_MODE = false)
const char* WIFI_SSID = "MinhaRede";
const char* WIFI_PASS = "MinhaSenha";

const uint16_t UDP_PORT      = 4210;
const uint16_t UDP_ACK_PORT  = 4211;  // porta de destino dos ACKs no PC
WiFiUDP udp;

const uint8_t START_BYTE  = 0xAA;
const uint8_t ACK_OK      = 0x00;
const uint8_t ACK_RANGE   = 0x01;
const uint8_t ACK_BUSY    = 0x02;
const uint8_t CMD_LEN     = 7;
const uint8_t ACK_LEN     = 4;

// 13, 14, 16 → MG995/MG996R   |   17, 18, 19 → SG90
const int SERVO_PINS[6]   = {13, 14, 16, 17, 18, 19};
const int NUM_SERVOS      = 6;

const int SERVO_MIN_US[6] = {600, 600, 600, 500, 500, 500};
const int SERVO_MAX_US[6] = {2400, 2400, 2400, 2400, 2400, 2400};

Servo servos[NUM_SERVOS];
volatile bool busy = false;

const int LED_PIN = 2;   // LED azul interno do ESP32

void sendAck(IPAddress remoteIP, uint16_t remotePort,
             uint8_t status, uint8_t servoIdx);
void moveServo(uint8_t idx, uint8_t angle);
void initWifi();
void initServos();

void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Braço Robótico — ESP32 WiFi");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  initServos();
  initWifi();

  udp.begin(UDP_PORT);
  Serial.printf("[UDP] Escutando na porta %d\n", UDP_PORT);

  // Sinal visual: 3 piscadas rápidas = pronto
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize < CMD_LEN) return;

  uint8_t buf[CMD_LEN];
  udp.read(buf, CMD_LEN);

  IPAddress remoteIP   = udp.remoteIP();
  uint16_t  remotePort = udp.remotePort();

  // Valida start byte
  if (buf[0] != START_BYTE) {
    Serial.println("[WARN] Pacote ignorado: start byte inválido");
    return;
  }

  // Rejeita se ocupado (proteção simples)
  if (busy) {
    sendAck(remoteIP, remotePort, ACK_BUSY, 0xFF);
    return;
  }

  busy = true;
  digitalWrite(LED_PIN, HIGH);

  uint8_t lastIdx = 0;
  bool rangeError = false;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    uint8_t angle = buf[1 + i];

    if (angle > 180) {
      Serial.printf("[WARN] Servo %d ângulo fora do range: %d\n", i, angle);
      sendAck(remoteIP, remotePort, ACK_RANGE, i);
      rangeError = true;
      break;
    }

    moveServo(i, angle);
    lastIdx = i;
  }

  if (!rangeError) {
    sendAck(remoteIP, remotePort, ACK_OK, lastIdx);
  }

  digitalWrite(LED_PIN, LOW);
  busy = false;
}

void moveServo(uint8_t idx, uint8_t angle) {
  int us = map(angle, 0, 180, SERVO_MIN_US[idx], SERVO_MAX_US[idx]);
  servos[idx].writeMicroseconds(us);
}

void sendAck(IPAddress remoteIP, uint16_t remotePort,
             uint8_t status, uint8_t servoIdx) {
  uint8_t ack[ACK_LEN];
  ack[0] = START_BYTE;
  ack[1] = status;
  ack[2] = servoIdx;
  ack[3] = ack[0] ^ ack[1] ^ ack[2];  // checksum XOR

  udp.beginPacket(remoteIP, UDP_ACK_PORT);
  udp.write(ack, ACK_LEN);
  udp.endPacket();
}

void initServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], SERVO_MIN_US[i], SERVO_MAX_US[i]);
    servos[i].write(90);   // posição neutra inicial
    delay(50);
  }
  Serial.println("[SERVO] Todos inicializados em 90°");
}

void initWifi() {
#if USE_AP_MODE
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[WiFi] AP criado: SSID=%s  IP=%s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WiFi] Conectando a '%s'", WIFI_SSID);
  uint8_t attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Conectado! IP: %s\n",
                  WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WiFi] FALHA — verifique SSID/senha. Reiniciando em AP mode.");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
  }
#endif
}
