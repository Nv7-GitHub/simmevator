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

// Counts the times the configuration had to be put back on a radio that had
// lost it. Nothing here acts on the value - it is exposed so a main can print
// it, because a count that climbs is the only outward sign of a supply or
// wiring fault that otherwise looks exactly like ordinary packet loss.
static uint32_t recoveries = 0;

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

// Every setting this node depends on, in one place. Split out of loraBringUp
// because the recovery paths below have to put back exactly what bring-up put
// on; a second copy of these calls would drift from this one the first time a
// setting changes, and the drift would only show up as a radio that recovers
// into a configuration nothing else on the link shares. Leaves the radio in
// standby. On failure *what names the call that failed, so a caller that
// cannot continue can say which one it was.
static int16_t applyRadioConfig(const char **what) {
  int16_t state = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH,
                              LORA_SPREADING_FACTOR, LORA_CODING_RATE,
                              LORA_SYNC_WORD, LORA_TX_POWER,
                              LORA_PREAMBLE_LEN, LORA_TCXO_VOLTAGE,
                              false /* use DC-DC regulator */);
  if (state != RADIOLIB_ERR_NONE) {
    *what = "radio.begin";
    return state;
  }

  // The Wio-SX1262 switches its RF front end from DIO2.
  state = radio.setDio2AsRfSwitch(true);
  if (state != RADIOLIB_ERR_NONE) {
    *what = "setDio2AsRfSwitch";
    return state;
  }

  // Explicit header + payload CRC, so a corrupted frame is dropped by the
  // radio rather than surfacing as a plausible-looking floor number. This is
  // the only CRC on the link - neither LoRa format carries one of its own
  // (spec 4.1, and the note in elev_packet.h), which is what lets an 8-byte
  // STATE stay 8 bytes.
  state = radio.setCRC(2);
  if (state != RADIOLIB_ERR_NONE) {
    *what = "setCRC";
    return state;
  }

  return RADIOLIB_ERR_NONE;
}

// Puts the configuration back on a radio that has been found not to have it,
// and says so on Serial - a node that silently re-configures itself in a loop
// looks identical to one that is working. Returns the RadioLib status; the
// caller decides what to do with a radio that would not take its settings,
// since halting is only right at bring-up.
static int16_t recoverRadio(const char *why) {
  const char *what = "radio config";
  recoveries++;
  int16_t state = applyRadioConfig(&what);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("[lora] %s: radio reconfigured (recovery %lu)\n",
                  why, (unsigned long)recoveries);
  } else {
    Serial.printf("[lora] %s: reconfigure failed at %s: %d (%s)\n",
                  why, what, state, loraStatusName(state));
  }
  // applyRadioConfig resets the chip, so whatever it was doing before is gone.
  asleep    = false;
  receiving = false;
  return state;
}

void loraBringUp(const char *roleName) {
  Serial.printf("\n=== Wio-SX1262: %s ===\n", roleName);
  Serial.printf("[lora] %.1f MHz  BW %.1f kHz  SF%d  CR4/%d  sync 0x%02X  %d dBm\n",
                (double)LORA_FREQUENCY, (double)LORA_BANDWIDTH,
                LORA_SPREADING_FACTOR, LORA_CODING_RATE,
                LORA_SYNC_WORD, LORA_TX_POWER);

  loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  const char *what  = "radio config";
  int16_t     state = applyRadioConfig(&what);
  if (state != RADIOLIB_ERR_NONE) {
    halt(what, state);
  }

  asleep    = false;
  receiving = false;
  Serial.println("[lora] radio ready");
}

uint32_t loraRecoveryCount() {
  return recoveries;
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
    int16_t rearm = radio.startReceive();
    if (rearm != RADIOLIB_ERR_NONE) {
      // A dropped re-arm is silent and permanent: the chip stays in standby,
      // DIO1 never fires, nothing else in the system polls the radio, and the
      // bridge sits there looking healthy until someone climbs to floor 5 and
      // power-cycles it. Retry once first - a BUSY-line timeout or an SPI
      // collision is usually gone by the next transaction - and if the second
      // attempt fails too, treat it as a chip that has lost its settings
      // rather than a busy one, and put them back.
      rearm = radio.startReceive();
      if (rearm != RADIOLIB_ERR_NONE) {
        Serial.printf("[lora] re-arm failed twice: %d (%s)\n",
                      rearm, loraStatusName(rearm));
        if (recoverRadio("re-arm") == RADIOLIB_ERR_NONE) {
          // recoverRadio cleared `receiving`; loraStartReceive re-attaches the
          // DIO1 handler that the chip reset dropped and sets it again.
          rearm = loraStartReceive();
        }
      }
      receiving = (rearm == RADIOLIB_ERR_NONE);
    }
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
  int16_t state = radio.standby();
  asleep = false;

  // Warm sleep only retains the configuration if the chip stayed powered. On
  // the car it may not have: spec 3.6 leaves the buck input electrolytic off,
  // and a +22 dBm burst pulls ~118 mA, so a sag can reset the SX1262. It comes
  // back at power-on defaults - GFSK, wrong frequency, no sync word, no CRC,
  // DIO2 not driving the RF switch - and from then on every transmit reports
  // success into the void while the bridge hears nothing. GetPacketType is
  // read back over SPI rather than from RadioLib's cached state, so a modem
  // that is no longer LoRa is honest evidence that the reset happened.
  ModemType_t modem = RADIOLIB_MODEM_NONE;
  if (state != RADIOLIB_ERR_NONE ||
      radio.getModem(&modem) != RADIOLIB_ERR_NONE ||
      modem != RADIOLIB_MODEM_LORA) {
    recoverRadio("wake");
  }
}

bool loraIsAsleep() {
  return asleep;
}
