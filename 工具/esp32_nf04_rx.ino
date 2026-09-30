/*
 * ESP32-C3 SuperMini + NF-04 接收端。
 * 输出稳定的 NF04_HEARTBEAT/NF04_ERROR JSONL，电脑面板可直接读取。
 * 接线：SCK=GPIO6、MISO=GPIO2、MOSI=GPIO7、CSN=GPIO10、CE=GPIO9。
 */

#include <SPI.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10
#define SCK_PIN 6
#define MISO_PIN 2
#define MOSI_PIN 7

static const uint8_t PROTOCOL_V2 = 2;
static const uint8_t PROTOCOL_V3 = 3;
static const uint8_t HEARTBEAT_TYPE = 1;
static const uint8_t SD_INIT_OK = 1U << 0;
static const uint8_t LOG_ACTIVE = 1U << 1;
static const uint8_t SD_FULL = 1U << 2;

RF24 radio(CE_PIN, CSN_PIN);
const byte addr[6] = {'P', 'C', 'B', '3', '2', 0x00};

static uint32_t u32be(const uint8_t *p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] << 8) | p[3];
}

static void errorRecord(const char *kind, const char *message) {
  Serial.print("NF04_ERROR {\"kind\":\"");
  Serial.print(kind);
  Serial.print("\",\"message\":\"");
  Serial.print(message);
  Serial.println("\"}");
}

static void heartbeatRecord(const uint8_t *buf) {
  const uint8_t version = buf[3];
  if (version != PROTOCOL_V2 && version != PROTOCOL_V3) {
    errorRecord("unknown_version", "未知心跳协议版本");
    return;
  }
  if (buf[4] != HEARTBEAT_TYPE) {
    errorRecord("unknown_type", "未知心跳消息类型");
    return;
  }
  if (buf[0] == 0 && buf[1] == 0 && buf[2] == 0) {
    errorRecord("invalid_device", "设备ID为空");
    return;
  }
  const uint32_t seq = u32be(&buf[5]);
  const uint32_t uptime = u32be(&buf[9]);
  const uint8_t flags = (version == PROTOCOL_V3) ? buf[17] : 0;
  Serial.printf(
      "NF04_HEARTBEAT {\"device_id\":\"%02X%02X%02X\",\"gzp_ok\":%s,"
      "\"icp_ok\":%s,\"log_active\":%s,\"message_type\":%u,"
      "\"mts4_ok\":%s,\"qmi_ok\":%s,\"sd_full\":%s,"
      "\"sd_init_ok\":%s,\"seq\":%lu,\"uptime_ms\":%lu,"
      "\"version\":%u}\n",
      buf[0], buf[1], buf[2], buf[15] == 0xC1 ? "true" : "false",
      buf[14] == 0xB1 ? "true" : "false",
      version == PROTOCOL_V2 ? "null" : ((flags & LOG_ACTIVE) ? "true" : "false"),
      buf[4], buf[16] == 0xD1 ? "true" : "false",
      buf[13] == 0xA1 ? "true" : "false",
      version == PROTOCOL_V2 ? "null" : ((flags & SD_FULL) ? "true" : "false"),
      version == PROTOCOL_V2 ? "null" : ((flags & SD_INIT_OK) ? "true" : "false"),
      (unsigned long)seq, (unsigned long)uptime, version);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  if (!radio.begin()) {
    errorRecord("receiver_init", "NF-04无响应");
    return;
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);
  // One-way broadcast: no reverse ACK or automatic retry.  The sender emits
  // three copies and every protocol-valid physical copy is printed below.
  radio.setAutoAck(false);
  radio.setPayloadSize(32);
  // Keep the common address in both library paths for deployment symmetry;
  // no reverse ACK is enabled.
  radio.openWritingPipe(addr);
  radio.openReadingPipe(0, addr);
  radio.startListening();
  radio.powerUp();
  Serial.println("NF04_READY {\"format\":\"NF04_HEARTBEAT_JSONL\",\"channel\":76,\"payload_bytes\":32,\"auto_ack_pipe0\":false,\"setup_retr\":0}");
}

void loop() {
  if (!radio.available()) {
    delay(10);
    return;
  }
  uint8_t buf[32] = {0};
  radio.read(buf, sizeof(buf));
  heartbeatRecord(buf);
}
