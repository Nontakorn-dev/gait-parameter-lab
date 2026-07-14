/**
 * DernDee Foot IMU Node - ICM-20948 Packet V1
 *
 * Goals:
 * - ESP32-C3-safe single-core task model
 * - 100 Hz Packet V1 stream compatible with the frontend Browser BLE decoder
 * - Raw accel/gyro int16 delivery (no scaling, no fusion)
 * - Automatic BLE advertising restart after disconnect
 *
 * Notes:
 * - ICM-20948 is used in 6-axis mode only.
 * - Packet timestamps use micros() and wrap naturally at uint32_t.
 * - This sketch uses I2C banked register access directly to avoid external
 *   Arduino library dependencies.
 */

#include <Wire.h>

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// ========== 1. Configuration ==========
// Change device name per foot before flashing: "DernDee_L_Shank", "DernDee_R_Shank"
const char *SENSOR_ID = "DernDee_R_Shank";

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

const int IMU_INTERVAL_MS = 10; // 100 Hz outgoing packet rate
const int LED_PIN = 2;
const int I2C_SDA_PIN = 4;
const int I2C_SCL_PIN = 5;
const uint16_t IMU_PACKET_MAGIC = 0xAA55;
const uint8_t IMU_PACKET_VERSION = 1;
const uint8_t IMU_RING_BUFFER_CAPACITY = 64;  // ~640ms BLE-stall tolerance (uint8 index/count holds <=255)
const uint32_t DIAG_INTERVAL_MS = 1000;
const uint32_t DATA_READY_TIMEOUT_US = 2500;

// ========== 2. ICM-20948 Register Map ==========
const uint8_t ICM20948_ADDR_PRIMARY = 0x69;
const uint8_t ICM20948_ADDR_SECONDARY = 0x68;
const uint8_t ICM20948_WHO_AM_I = 0x00;         // Bank 0
const uint8_t ICM20948_USER_CTRL = 0x03;        // Bank 0
const uint8_t ICM20948_LP_CONFIG = 0x05;        // Bank 0
const uint8_t ICM20948_PWR_MGMT_1 = 0x06;       // Bank 0
const uint8_t ICM20948_PWR_MGMT_2 = 0x07;       // Bank 0
const uint8_t ICM20948_INT_PIN_CFG = 0x0F;      // Bank 0
const uint8_t ICM20948_ACCEL_XOUT_H = 0x2D;     // Bank 0
const uint8_t ICM20948_DATA_RDY_STATUS = 0x74;  // Bank 0
const uint8_t ICM20948_REG_BANK_SEL = 0x7F;     // Any bank

const uint8_t ICM20948_GYRO_SMPLRT_DIV = 0x00;   // Bank 2
const uint8_t ICM20948_GYRO_CONFIG_1 = 0x01;     // Bank 2
const uint8_t ICM20948_ACCEL_SMPLRT_DIV_1 = 0x10; // Bank 2
const uint8_t ICM20948_ACCEL_SMPLRT_DIV_2 = 0x11; // Bank 2
const uint8_t ICM20948_ACCEL_CONFIG = 0x14;      // Bank 2

const uint8_t ICM20948_BANK_0 = 0;
const uint8_t ICM20948_BANK_2 = 2;
const uint8_t ICM20948_WHOAMI_EXPECTED = 0xEA;

// Full-scale settings
const uint8_t ICM20948_ACCEL_FS_SEL_8G = 0x02;
const uint8_t ICM20948_GYRO_FS_SEL_2000DPS = 0x03;

// DLPF settings.
// Internal ODR is ~100 Hz (Nyquist 50 Hz), so the previous ~50/51 Hz cutoff
// (DLPFCFG=3) sat right at Nyquist and risked aliasing. Gait energy is below
// ~15 Hz, so drop the cutoff to ~24 Hz (DLPFCFG=4) for cleaner heel-strike
// signals with margin. Filter stays enabled (FCHOICE bit set in build*Config).
const uint8_t ICM20948_ACCEL_DLPF_CFG_24HZ = 0x04;  // ~23.4 Hz
const uint8_t ICM20948_GYRO_DLPF_CFG_24HZ = 0x04;   // ~23.9 Hz

// Internal ODR dividers.
// Gyro ODR with DLPF enabled: ~1100 / (1 + div)
// Accel ODR with DLPF enabled: ~1125 / (1 + div)
// Divider 10 keeps the sensor ODR slightly above the 100 Hz packet rate.
const uint8_t ICM20948_GYRO_DIV_100HZ = 10;
const uint16_t ICM20948_ACCEL_DIV_NEAR_100HZ = 10;

// ========== 3. Data Structures ==========
typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint8_t version;
  uint16_t seq;
  uint32_t timestamp_us;
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  uint8_t crc8;
} ImuPacketV1;

typedef struct {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
} ImuRawSample;

static_assert(sizeof(ImuPacketV1) == 22, "ImuPacketV1 must be 22 bytes");

// Ring Buffer
ImuPacketV1 g_packetBuffer[IMU_RING_BUFFER_CAPACITY];
volatile uint8_t g_packetHead = 0;
volatile uint8_t g_packetTail = 0;
volatile uint8_t g_packetCount = 0;

// Diagnostics / Shared State
volatile uint16_t g_nextSeq = 0;
volatile uint32_t g_readErrorCount = 0;
volatile uint32_t g_droppedSampleCount = 0;
volatile uint32_t g_sentPacketCount = 0;
volatile uint32_t g_connectCount = 0;
volatile uint32_t g_disconnectCount = 0;
volatile uint32_t g_lastDiagMs = 0;
volatile bool g_imuReady = false;

// Runtime state
uint8_t g_currentRegisterBank = 0xFF;
uint8_t g_icmAddress = ICM20948_ADDR_PRIMARY;

// RTOS Handles
TaskHandle_t imuTaskHandle = nullptr;
TaskHandle_t bleTaskHandle = nullptr;
BLECharacteristic *pImuChar = nullptr;
BLEServer *pServer = nullptr;
portMUX_TYPE packetMux = portMUX_INITIALIZER_UNLOCKED;

// Connection State
volatile bool deviceConnected = false;
bool oldDeviceConnected = false;

// Prototypes
bool selectBank(uint8_t bank);
bool probeI2CAddress(uint8_t address);
bool writeICMRegister(uint8_t bank, uint8_t reg, uint8_t data);
bool readICMRegister(uint8_t bank, uint8_t reg, uint8_t &data);
bool readICMRegisters(uint8_t bank, uint8_t startReg, uint8_t *buffer, size_t len);
bool initICM20948();
bool waitForDataReady(uint32_t timeoutUs);
bool readICM20948(ImuRawSample &sample);
void imuTask(void *pvParameters);
void bleTask(void *pvParameters);
bool pushPacket(const ImuPacketV1 &packet);
bool popPacket(ImuPacketV1 &packet);
uint8_t calcCRC8(const uint8_t *data, size_t len);
void logDiagnosticsIfDue();

uint8_t buildAccelConfig(uint8_t fsSel, uint8_t dlpfCfg) {
  return static_cast<uint8_t>(0x01 | (fsSel << 1) | (dlpfCfg << 3));
}

uint8_t buildGyroConfig1(uint8_t fsSel, uint8_t dlpfCfg) {
  return static_cast<uint8_t>(0x01 | (fsSel << 1) | (dlpfCfg << 3));
}

// BLE Callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) {
    (void)server;
    deviceConnected = true;
    portENTER_CRITICAL(&packetMux);
    g_connectCount++;
    portEXIT_CRITICAL(&packetMux);
    Serial.println("Client connected");
    digitalWrite(LED_PIN, HIGH);
  };

  void onDisconnect(BLEServer *server) {
    (void)server;
    deviceConnected = false;
    portENTER_CRITICAL(&packetMux);
    g_disconnectCount++;
    portEXIT_CRITICAL(&packetMux);
    Serial.println("Client disconnected");
    digitalWrite(LED_PIN, LOW);
  }
};

// ========== 4. Setup ==========
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  delay(500);
  Serial.println("--------------------------------");
  Serial.printf("Device: %s (ICM-20948 Foot IMU Packet V1)\n", SENSOR_ID);
  Serial.printf("Packet size: %u bytes, Ring buffer: %u packets\n",
                static_cast<unsigned int>(sizeof(ImuPacketV1)),
                static_cast<unsigned int>(IMU_RING_BUFFER_CAPACITY));

  g_imuReady = initICM20948();
  if (!g_imuReady) {
    Serial.println("IMU initialization failed. BLE will still advertise, but no samples will be streamed.");
  }

  // --- Init BLE ---
  BLEDevice::init(SENSOR_ID);
  BLEDevice::setMTU(64);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pImuChar = pService->createCharacteristic(CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_NOTIFY);
  pImuChar->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  // Web Bluetooth matches service filters from the primary advertisement only.
  // Keep the service UUID out of scan response so browser pickers can discover the node.
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.println("BLE advertising started");

  // --- Tasks ---
  // Use xTaskCreate instead of core pinning so this remains ESP32-C3 safe.
  xTaskCreate(imuTask, "imuTask", 4096, nullptr, 2, &imuTaskHandle);
  xTaskCreate(bleTask, "bleTask", 4096, nullptr, 1, &bleTaskHandle);
}

void loop() {
  if (!deviceConnected && oldDeviceConnected) {
    delay(200);
    pServer->startAdvertising();
    Serial.println("Restart advertising");
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  logDiagnosticsIfDue();
  vTaskDelay(pdMS_TO_TICKS(100));
}

// ========== 5. Tasks Logic ==========

void imuTask(void *pvParameters) {
  (void)pvParameters;
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    if (!g_imuReady) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    ImuRawSample raw;
    if (readICM20948(raw)) {
      ImuPacketV1 packet;
      packet.magic = IMU_PACKET_MAGIC;
      packet.version = IMU_PACKET_VERSION;

      portENTER_CRITICAL(&packetMux);
      packet.seq = g_nextSeq++;
      portEXIT_CRITICAL(&packetMux);

      packet.timestamp_us = static_cast<uint32_t>(micros());
      packet.ax = raw.ax;
      packet.ay = raw.ay;
      packet.az = raw.az;
      packet.gx = raw.gx;
      packet.gy = raw.gy;
      packet.gz = raw.gz;
      packet.crc8 = calcCRC8(reinterpret_cast<const uint8_t *>(&packet),
                             sizeof(ImuPacketV1) - 1);

      pushPacket(packet);

      if (bleTaskHandle != nullptr) {
        xTaskNotifyGive(bleTaskHandle);
      }
    } else {
      portENTER_CRITICAL(&packetMux);
      g_readErrorCount++;
      portEXIT_CRITICAL(&packetMux);
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(IMU_INTERVAL_MS));
  }
}

void bleTask(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    ImuPacketV1 txPacket;
    while (popPacket(txPacket)) {
      if (deviceConnected && pImuChar != nullptr) {
        pImuChar->setValue(reinterpret_cast<uint8_t *>(&txPacket),
                           sizeof(ImuPacketV1));
        pImuChar->notify();

        portENTER_CRITICAL(&packetMux);
        g_sentPacketCount++;
        portEXIT_CRITICAL(&packetMux);
      }
    }
  }
}

// ========== 6. ICM-20948 Utilities ==========

bool selectBank(uint8_t bank) {
  const uint8_t bankValue = static_cast<uint8_t>((bank & 0x03) << 4);
  if (g_currentRegisterBank == bankValue) {
    return true;
  }

  Wire.beginTransmission(g_icmAddress);
  Wire.write(ICM20948_REG_BANK_SEL);
  Wire.write(bankValue);
  if (Wire.endTransmission(true) != 0) {
    return false;
  }

  g_currentRegisterBank = bankValue;
  return true;
}

bool probeI2CAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission(true) == 0;
}

bool writeICMRegister(uint8_t bank, uint8_t reg, uint8_t data) {
  if (!selectBank(bank)) {
    return false;
  }

  Wire.beginTransmission(g_icmAddress);
  Wire.write(reg);
  Wire.write(data);
  return Wire.endTransmission(true) == 0;
}

bool readICMRegister(uint8_t bank, uint8_t reg, uint8_t &data) {
  if (!selectBank(bank)) {
    return false;
  }

  Wire.beginTransmission(g_icmAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t count = Wire.requestFrom(static_cast<int>(g_icmAddress), 1, true);
  if (count != 1 || Wire.available() != 1) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  data = Wire.read();
  return true;
}

bool readICMRegisters(uint8_t bank, uint8_t startReg, uint8_t *buffer, size_t len) {
  if (!selectBank(bank)) {
    return false;
  }

  Wire.beginTransmission(g_icmAddress);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t count = Wire.requestFrom(static_cast<int>(g_icmAddress), static_cast<int>(len), true);
  if (count != len || static_cast<size_t>(Wire.available()) != len) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }

  return true;
}

bool initICM20948() {
  uint8_t whoami = 0x00;
  g_currentRegisterBank = 0xFF;

  if (probeI2CAddress(ICM20948_ADDR_PRIMARY)) {
    g_icmAddress = ICM20948_ADDR_PRIMARY;
  } else if (probeI2CAddress(ICM20948_ADDR_SECONDARY)) {
    g_icmAddress = ICM20948_ADDR_SECONDARY;
    Serial.printf("ICM-20948 ACK detected at alternate address 0x%02X\n", g_icmAddress);
  } else {
    Serial.printf("ICM-20948 did not ACK on 0x%02X or 0x%02X\n",
                  ICM20948_ADDR_PRIMARY, ICM20948_ADDR_SECONDARY);
    return false;
  }

  if (!readICMRegister(ICM20948_BANK_0, ICM20948_WHO_AM_I, whoami)) {
    Serial.println("Failed to read ICM-20948 WHO_AM_I");
    return false;
  }

  if (whoami != ICM20948_WHOAMI_EXPECTED) {
    Serial.printf("Unexpected ICM-20948 WHO_AM_I: 0x%02X (expected 0x%02X)\n",
                  whoami, ICM20948_WHOAMI_EXPECTED);
    return false;
  }

  Serial.printf("ICM-20948 WHO_AM_I: 0x%02X\n", whoami);

  // Software reset and restart from a known state.
  if (!writeICMRegister(ICM20948_BANK_0, ICM20948_PWR_MGMT_1, 0x80)) {
    Serial.println("Failed to issue ICM-20948 reset");
    return false;
  }
  delay(100);
  g_currentRegisterBank = 0xFF;

  // Clock source: auto-select best available PLL clock, keep device awake.
  if (!writeICMRegister(ICM20948_BANK_0, ICM20948_PWR_MGMT_1, 0x01)) {
    Serial.println("Failed to configure ICM-20948 clock source");
    return false;
  }
  delay(10);

  // Ensure continuous sampling mode and no low-power cycling.
  if (!writeICMRegister(ICM20948_BANK_0, ICM20948_LP_CONFIG, 0x00)) {
    Serial.println("Failed to configure ICM-20948 low-power mode");
    return false;
  }

  // Keep I2C mode enabled, DMP/FIFO/I2C master disabled.
  if (!writeICMRegister(ICM20948_BANK_0, ICM20948_USER_CTRL, 0x00)) {
    Serial.println("Failed to configure ICM-20948 USER_CTRL");
    return false;
  }

  // Enable accel and gyro axes and keep reserved bits cleared.
  if (!writeICMRegister(ICM20948_BANK_0, ICM20948_PWR_MGMT_2, 0x00)) {
    Serial.println("Failed to enable ICM-20948 accel/gyro axes");
    return false;
  }

  // Sample rate dividers close to 100 Hz.
  if (!writeICMRegister(ICM20948_BANK_2, ICM20948_GYRO_SMPLRT_DIV,
                        ICM20948_GYRO_DIV_100HZ)) {
    Serial.println("Failed to set gyro sample rate divider");
    return false;
  }

  if (!writeICMRegister(ICM20948_BANK_2, ICM20948_ACCEL_SMPLRT_DIV_1,
                        static_cast<uint8_t>((ICM20948_ACCEL_DIV_NEAR_100HZ >> 8) & 0x0F))) {
    Serial.println("Failed to set accel sample rate divider high byte");
    return false;
  }

  if (!writeICMRegister(ICM20948_BANK_2, ICM20948_ACCEL_SMPLRT_DIV_2,
                        static_cast<uint8_t>(ICM20948_ACCEL_DIV_NEAR_100HZ & 0xFF))) {
    Serial.println("Failed to set accel sample rate divider low byte");
    return false;
  }

  // Full-scale and DLPF config.
  const uint8_t gyroConfig = buildGyroConfig1(ICM20948_GYRO_FS_SEL_2000DPS,
                                              ICM20948_GYRO_DLPF_CFG_24HZ);
  const uint8_t accelConfig = buildAccelConfig(ICM20948_ACCEL_FS_SEL_8G,
                                               ICM20948_ACCEL_DLPF_CFG_24HZ);

  if (!writeICMRegister(ICM20948_BANK_2, ICM20948_GYRO_CONFIG_1, gyroConfig)) {
    Serial.println("Failed to configure gyro full scale / DLPF");
    return false;
  }

  if (!writeICMRegister(ICM20948_BANK_2, ICM20948_ACCEL_CONFIG, accelConfig)) {
    Serial.println("Failed to configure accel full scale / DLPF");
    return false;
  }

  Serial.println("ICM-20948 configured: +/-8g, +/-2000dps, DLPF enabled, near-100Hz internal ODR");
  return true;
}

bool waitForDataReady(uint32_t timeoutUs) {
  const uint32_t startUs = micros();

  while (static_cast<uint32_t>(micros() - startUs) < timeoutUs) {
    uint8_t status = 0;
    if (!readICMRegister(ICM20948_BANK_0, ICM20948_DATA_RDY_STATUS, status)) {
      return false;
    }

    if ((status & 0x01) != 0) {
      return true;
    }

    delayMicroseconds(50);
  }

  return false;
}

bool readICM20948(ImuRawSample &sample) {
  uint8_t rawBytes[12];

  if (!waitForDataReady(DATA_READY_TIMEOUT_US)) {
    return false;
  }

  if (!readICMRegisters(ICM20948_BANK_0, ICM20948_ACCEL_XOUT_H, rawBytes, sizeof(rawBytes))) {
    return false;
  }

  sample.ax = static_cast<int16_t>((rawBytes[0] << 8) | rawBytes[1]);
  sample.ay = static_cast<int16_t>((rawBytes[2] << 8) | rawBytes[3]);
  sample.az = static_cast<int16_t>((rawBytes[4] << 8) | rawBytes[5]);
  sample.gx = static_cast<int16_t>((rawBytes[6] << 8) | rawBytes[7]);
  sample.gy = static_cast<int16_t>((rawBytes[8] << 8) | rawBytes[9]);
  sample.gz = static_cast<int16_t>((rawBytes[10] << 8) | rawBytes[11]);
  return true;
}

// ========== 7. Buffer / CRC / Diagnostics ==========

bool pushPacket(const ImuPacketV1 &packet) {
  bool droppedOldest = false;

  portENTER_CRITICAL(&packetMux);

  if (g_packetCount == IMU_RING_BUFFER_CAPACITY) {
    g_packetTail = (g_packetTail + 1) % IMU_RING_BUFFER_CAPACITY;
    g_packetCount--;
    g_droppedSampleCount++;
    droppedOldest = true;
  }

  g_packetBuffer[g_packetHead] = packet;
  g_packetHead = (g_packetHead + 1) % IMU_RING_BUFFER_CAPACITY;
  g_packetCount++;

  portEXIT_CRITICAL(&packetMux);
  return !droppedOldest;
}

bool popPacket(ImuPacketV1 &packet) {
  bool hasPacket = false;

  portENTER_CRITICAL(&packetMux);
  if (g_packetCount > 0) {
    packet = g_packetBuffer[g_packetTail];
    g_packetTail = (g_packetTail + 1) % IMU_RING_BUFFER_CAPACITY;
    g_packetCount--;
    hasPacket = true;
  }
  portEXIT_CRITICAL(&packetMux);

  return hasPacket;
}

// CRC-8-ATM (poly 0x07, init 0x00)
uint8_t calcCRC8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;

  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if ((crc & 0x80) != 0) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

void logDiagnosticsIfDue() {
  uint32_t nowMs = millis();
  uint32_t lastDiagMs = 0;

  portENTER_CRITICAL(&packetMux);
  lastDiagMs = g_lastDiagMs;
  portEXIT_CRITICAL(&packetMux);

  if (nowMs - lastDiagMs < DIAG_INTERVAL_MS) {
    return;
  }

  uint32_t readErrors = 0;
  uint32_t droppedSamples = 0;
  uint32_t sentPackets = 0;
  uint32_t connectCount = 0;
  uint32_t disconnectCount = 0;
  uint8_t queuedPackets = 0;
  uint16_t nextSeq = 0;

  portENTER_CRITICAL(&packetMux);
  g_lastDiagMs = nowMs;
  readErrors = g_readErrorCount;
  droppedSamples = g_droppedSampleCount;
  sentPackets = g_sentPacketCount;
  connectCount = g_connectCount;
  disconnectCount = g_disconnectCount;
  queuedPackets = g_packetCount;
  nextSeq = g_nextSeq;
  portEXIT_CRITICAL(&packetMux);

  Serial.printf(
      "diag imu=%u connected=%u seq=%u sent=%lu queued=%u read_err=%lu dropped=%lu conn=%lu disc=%lu\n",
      g_imuReady ? 1 : 0, deviceConnected ? 1 : 0, nextSeq,
      static_cast<unsigned long>(sentPackets),
      static_cast<unsigned int>(queuedPackets),
      static_cast<unsigned long>(readErrors),
      static_cast<unsigned long>(droppedSamples),
      static_cast<unsigned long>(connectCount),
      static_cast<unsigned long>(disconnectCount));
}