//
// Wio-SX1262 bring-up, transmit, receive and sleep. See lora_link.h for the
// pin map, the settings and why sleep is part of the API.
//
#include "lora_link.h"

// The XIAO ESP32S3 talks to the Wio-SX1262 over FSPI on D8/D9/D10.
SPIClass loraSpi(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, loraSpi);

// Set from the DIO1 interrupt, cleared by loraPoll(). The ISR does nothing
// else: the SPI read that empties the FIFO belongs in the main loop.
static volatile bool packetWaiting = false;

static bool  asleep       = false;
static bool  receiving    = false;
static float lastRssiDbm  = 0.0f;
static float lastSnrDb    = 0.0f;

const char *loraStatusName(int16_t state) {
  switch (state) {
    case RADIOLIB_ERR_NONE:              return "OK";
    case RADIOLIB_ERR_CHIP_NOT_FOUND:    return "CHIP_NOT_FOUND (check wiring/pins)";
    case RADIOLIB_ERR_SPI_CMD_TIMEOUT:   return "SPI_CMD_TIMEOUT (BUSY pin stuck?)";
    case RADIOLIB_ERR_SPI_CMD_INVALID:   return "SPI_CMD_INVALID";
    case RADIOLIB_ERR_SPI_CMD_FAILED:    return "SPI_CMD_FAILED";
    case RADIOLIB_ERR_INVALID_FREQUENCY: return "INVALID_FREQUENCY";
    case RADIOLIB_ERR_INVALID_OUTPUT_POWER: return "INVALID_OUTPUT_POWER";
    case RADIOLIB_ERR_CRC_MISMATCH:      return "CRC_MISMATCH";
    case RADIOLIB_ERR_RX_TIMEOUT:        return "RX_TIMEOUT";
    case RADIOLIB_ERR_TX_TIMEOUT:        return "TX_TIMEOUT";
    default:                             return "see RadioLib TypeDef.h";
  }
}

static void halt(const char *what, int16_t state) {
  Serial.printf("[lora] %s failed: %d (%s)\n", what, state, loraStatusName(state));
  Serial.println("[lora] halted - fix the above and reset the board");
  while (true) {
    delay(1000);
  }
}

#if defined(ESP32)
ICACHE_RAM_ATTR
#endif
static void onDio1() {
  packetWaiting = true;
}

void loraBringUp(const char *roleName) {
  Serial.printf("\n=== Wio-SX1262: %s ===\n", roleName);
  Serial.printf("[lora] %.1f MHz  BW %.1f kHz  SF%d  CR4/%d  sync 0x%02X  %d dBm\n",
                (double)LORA_FREQUENCY, (double)LORA_BANDWIDTH,
                LORA_SPREADING_FACTOR, LORA_CODING_RATE,
                LORA_SYNC_WORD, LORA_TX_POWER);

  loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  int16_t state = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH,
                              LORA_SPREADING_FACTOR, LORA_CODING_RATE,
                              LORA_SYNC_WORD, LORA_TX_POWER,
                              LORA_PREAMBLE_LEN, LORA_TCXO_VOLTAGE,
                              false /* use DC-DC regulator */);
  if (state != RADIOLIB_ERR_NONE) {
    halt("radio.begin", state);
  }

  // The Wio-SX1262 switches its RF front end from DIO2.
  state = radio.setDio2AsRfSwitch(true);
  if (state != RADIOLIB_ERR_NONE) {
    halt("setDio2AsRfSwitch", state);
  }

  // Explicit header + payload CRC, so a corrupted frame is dropped by the
  // radio rather than surfacing as a plausible-looking floor number. The
  // packets carry their own CRC16 as well - this one costs nothing and saves
  // the bridge from ever seeing most of the damage.
  state = radio.setCRC(2);
  if (state != RADIOLIB_ERR_NONE) {
    halt("setCRC", state);
  }

  asleep    = false;
  receiving = false;
  Serial.println("[lora] radio ready");
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------
int16_t loraTransmit(const uint8_t *data, size_t len) {
  // A transmit from sleep would go out with the chip still waking, so the wake
  // is done here rather than trusting every caller to have done it.
  if (asleep) {
    loraWake();
  }
  int16_t state = radio.transmit(const_cast<uint8_t *>(data), len);
  receiving = false;   // transmit leaves the radio in standby
  return state;
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------
int16_t loraStartReceive() {
  if (asleep) {
    loraWake();
  }
  radio.setPacketReceivedAction(onDio1);
  int16_t state = radio.startReceive();
  receiving = (state == RADIOLIB_ERR_NONE);
  return state;
}

bool loraPoll(uint8_t *buf, size_t cap, size_t *outLen, int16_t *outState) {
  if (!packetWaiting) {
    return false;
  }
  packetWaiting = false;

  size_t len = radio.getPacketLength();
  if (len > cap) {
    // Not one of ours - the longest packet this system sends is 24 bytes.
    // Truncate rather than overrun, and let the length the caller sees tell it
    // something was wrong.
    len = cap;
  }

  int16_t state = radio.readData(buf, len);

  // RSSI and SNR are read after readData while they still describe the packet
  // just taken, not the noise floor of whatever the radio hears next.
  lastRssiDbm = radio.getRSSI();
  lastSnrDb   = radio.getSNR();

  if (outLen)   *outLen   = len;
  if (outState) *outState = state;

  // readData drops the radio to standby. Continuous receive has to be armed
  // again or the bridge goes deaf after its first packet.
  if (receiving) {
    radio.startReceive();
  }
  return true;
}

float loraLastRssi() {
  return lastRssiDbm;
}

float loraLastSnr() {
  return lastSnrDb;
}

// ---------------------------------------------------------------------------
// Power
// ---------------------------------------------------------------------------
void loraSleep() {
  if (asleep) {
    return;
  }
  // Warm sleep: the configuration survives, so waking is a standby command
  // rather than a full begin() with its TCXO settling time.
  radio.sleep(true);
  asleep    = true;
  receiving = false;
}

void loraWake() {
  if (!asleep) {
    return;
  }
  radio.standby();
  asleep = false;
}

bool loraIsAsleep() {
  return asleep;
}
