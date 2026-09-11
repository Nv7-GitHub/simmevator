//
// ESP-NOW flood: the radio and the schedule. See espnow_mesh.h for the frame
// path, why relaying is jittered, and why the receive callback does nothing.
//
#include "espnow_mesh.h"

#include <WiFi.h>
#include <esp_random.h>
#include <esp_wifi.h>

const uint8_t MESH_BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// platformio.ini sets the dedup depth as MESH_DEDUP_RING and mesh_packet.h
// sizes the ring from MESH_DEDUP_DEPTH. A build flag changed in one place and
// not the other belongs here, at compile time, rather than in a mesh that
// dedups over a different window than the spec documents.
static_assert(MESH_DEDUP_RING == MESH_DEDUP_DEPTH,
              "MESH_DEDUP_RING and MESH_DEDUP_DEPTH disagree");

static_assert(MESH_MAX_FRAME_BYTES <= ESP_NOW_MAX_DATA_LEN,
              "a mesh frame does not fit in a single ESP-NOW frame");

static_assert(MESH_RELAY_JITTER_MAX_MS >= MESH_RELAY_JITTER_MIN_MS,
              "the relay jitter window is inverted");

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static MeshRole         gRole    = MESH_ROLE_RELAY;
static MeshFrameHandler gHandler = NULL;
static MeshCounters     gCounters;
static MeshDedup        gDedup;

// Minting starts at 1 so that meshLastOrigSeq() == 0 keeps meaning "nothing
// yet" for the whole life of the node.
static uint16_t gNextOrigSeq = 1;
static uint16_t gLastOrigSeq = 0;

// ---------------------------------------------------------------------------
// Receive ring - written by the WiFi task, read by loop()
//
// Single producer, single consumer, so the two indices need no lock: the
// producer only ever advances the head after the slot behind it is filled, and
// the consumer only ever advances the tail after it has copied the slot out.
// Both are byte-wide and the ring is a power-of-two-free modulo, which the
// compiler turns into a compare rather than a divide at this size.
// ---------------------------------------------------------------------------
struct RxSlot {
  uint8_t bytes[MESH_MAX_FRAME_BYTES];
  uint8_t len;
  uint8_t mac[6];
  int8_t  rssi;
};

static RxSlot           gRx[MESH_RX_QUEUE_DEPTH];
static volatile uint8_t gRxHead = 0;
static volatile uint8_t gRxTail = 0;

// MeshCounters is written only from loop(), so the three things the callback
// has to report reach it through these instead. 32-bit aligned words on both
// the S3 and the WROOM-32, so a torn read is not possible.
static volatile uint32_t gCbHeard     = 0;
static volatile uint32_t gCbQueueFull = 0;
static volatile uint32_t gCbTooLong   = 0;

// dropLength also counts decode failures from loop(), so the callback's share
// of it is folded in as a difference rather than assigned over the top.
static uint32_t gFoldedTooLong = 0;

// ---------------------------------------------------------------------------
// Relay slots - frames waiting out their jitter
// ---------------------------------------------------------------------------
struct RelaySlot {
  uint8_t  bytes[MESH_MAX_FRAME_BYTES];
  uint8_t  len;
  uint32_t dueMs;
  bool     used;
};

static RelaySlot gRelay[MESH_RELAY_SLOTS];

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

const char *meshStatusName(esp_err_t err) {
  switch (err) {
    case ESP_OK:                   return "OK";
    case ESP_ERR_ESPNOW_NOT_INIT:  return "ESPNOW_NOT_INIT";
    case ESP_ERR_ESPNOW_ARG:       return "ESPNOW_ARG (bad peer/length?)";
    case ESP_ERR_ESPNOW_NO_MEM:    return "ESPNOW_NO_MEM";
    case ESP_ERR_ESPNOW_FULL:      return "ESPNOW_FULL (too many peers)";
    case ESP_ERR_ESPNOW_NOT_FOUND: return "ESPNOW_NOT_FOUND (peer not added)";
    case ESP_ERR_ESPNOW_INTERNAL:  return "ESPNOW_INTERNAL";
    case ESP_ERR_ESPNOW_EXIST:     return "ESPNOW_EXIST (peer already added)";
    case ESP_ERR_ESPNOW_IF:        return "ESPNOW_IF (wrong interface)";
    case ESP_ERR_WIFI_NOT_INIT:    return "WIFI_NOT_INIT";
    case ESP_ERR_WIFI_NOT_STARTED: return "WIFI_NOT_STARTED";
    case ESP_ERR_INVALID_ARG:      return "INVALID_ARG";
    default:                       return esp_err_to_name(err);
  }
}

const char *meshFormatMac(const uint8_t *mac, char *out) {
  sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return out;
}

static void halt(const char *what, esp_err_t err) {
  Serial.printf("[mesh] %s failed: %d (%s)\n", what, (int)err, meshStatusName(err));
  Serial.println("[mesh] halted - fix the above and reset the board");
  while (true) {
    delay(1000);
  }
}

// esp_random() rather than Arduino's random(): it is the hardware RNG, which is
// properly seeded once the radio is on, and every node in the mesh boots from
// the same image. A software PRNG left at its default seed would hand all ten
// displays the same jitter sequence, which is the collision this wait exists to
// prevent.
static uint32_t relayJitterMs() {
  const uint32_t span = (uint32_t)(MESH_RELAY_JITTER_MAX_MS - MESH_RELAY_JITTER_MIN_MS) + 1u;
  return (uint32_t)MESH_RELAY_JITTER_MIN_MS + (esp_random() % span);
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------

// MESH_REPEATS copies back to back. Broadcast ESP-NOW frames are never
// acknowledged and never retried by the driver, so esp_now_send() reporting OK
// means our own PHY accepted the frame, not that anyone heard it.
static void sendCopies(const uint8_t *bytes, size_t len) {
  for (int i = 0; i < MESH_REPEATS; i++) {
    esp_err_t err = esp_now_send(MESH_BROADCAST_ADDR, bytes, len);
    if (err == ESP_OK) {
      gCounters.sends++;
    } else {
      gCounters.sendFails++;
    }
  }
}

static bool queueRelay(const uint8_t *bytes, size_t len, uint32_t nowMs) {
  for (int i = 0; i < MESH_RELAY_SLOTS; i++) {
    if (gRelay[i].used) continue;
    memcpy(gRelay[i].bytes, bytes, len);
    gRelay[i].len   = (uint8_t)len;
    gRelay[i].dueMs = nowMs + relayJitterMs();
    gRelay[i].used  = true;
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Receive callback - runs on the WiFi task. memcpy and return, nothing else.
// ---------------------------------------------------------------------------
static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data,
                         int dataLen) {
  // Read-modify-write spelled out: C++20 deprecates ++ on a volatile, and this
  // is the single producer, so there is nothing to race with.
  gCbHeard = gCbHeard + 1;

  if (data == NULL || dataLen <= 0 || dataLen > (int)MESH_MAX_FRAME_BYTES) {
    // Longer than any frame this format can produce, so it will not fit a ring
    // slot and cannot be handed to the decoder to be rejected properly. It is
    // a length failure all the same - counted as one rather than vanishing.
    gCbTooLong = gCbTooLong + 1;
    return;
  }

  const uint8_t head = gRxHead;
  const uint8_t next = (uint8_t)((head + 1) % MESH_RX_QUEUE_DEPTH);
  if (next == gRxTail) {
    gCbQueueFull = gCbQueueFull + 1;
    return;
  }

  RxSlot &slot = gRx[head];
  memcpy(slot.bytes, data, (size_t)dataLen);
  slot.len = (uint8_t)dataLen;
  if (info != NULL && info->src_addr != NULL) {
    memcpy(slot.mac, info->src_addr, sizeof(slot.mac));
  } else {
    memset(slot.mac, 0, sizeof(slot.mac));
  }
  // 0 where the IDF build does not hand the callback its radio metadata, which
  // the handler is documented to expect.
  slot.rssi = (info != NULL && info->rx_ctrl != NULL) ? (int8_t)info->rx_ctrl->rssi : 0;

  gRxHead = next;
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

void meshBringUp(MeshRole role, MeshFrameHandler onFrame, const char *roleName) {
  gRole    = role;
  gHandler = onFrame;
  memset(&gCounters, 0, sizeof(gCounters));
  gCbHeard     = 0;
  gCbQueueFull = 0;
  gCbTooLong   = 0;
  gFoldedTooLong = 0;

  meshDedupInit(&gDedup);
  for (int i = 0; i < MESH_RELAY_SLOTS; i++) gRelay[i].used = false;

  Serial.printf("\n=== ESP-NOW mesh: %s ===\n", roleName);

  // Station mode with no association: ESP-NOW rides on raw 802.11 frames, and
  // joining an AP would let the AP drag us onto a different channel.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);

  // Modem sleep parks the radio between beacons. With nothing to keep us awake
  // that silently eats received frames - a display that hears the bridge for a
  // second and then goes deaf, with no counter anywhere showing why.
  WiFi.setSleep(false);

  // LR alone, not LR alongside 11b/g/n: every node in this system is an ESP32,
  // so there is nothing to stay compatible with, and leaving the legacy
  // protocols enabled lets the PHY pick one of them for a frame and throw away
  // the ~7 dB that is the entire reason for being here.
  esp_err_t err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  if (err != ESP_OK) {
    halt("esp_wifi_set_protocol(LR)", err);
  }

  err = esp_wifi_set_channel(MESH_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) {
    halt("esp_wifi_set_channel", err);
  }

  // Ask for maximum power, then report what the PHY actually granted - the API
  // quantises the request and the ceiling also depends on the calibration data
  // flashed into the module, which differs between the XIAO and the CYD.
  err = esp_wifi_set_max_tx_power(MESH_TX_POWER_QDBM);
  if (err != ESP_OK) {
    halt("esp_wifi_set_max_tx_power", err);
  }
  int8_t actualQdbm = 0;
  err = esp_wifi_get_max_tx_power(&actualQdbm);
  if (err != ESP_OK) {
    halt("esp_wifi_get_max_tx_power", err);
  }

  err = esp_now_init();
  if (err != ESP_OK) {
    halt("esp_now_init", err);
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, MESH_BROADCAST_ADDR, sizeof(peer.peer_addr));
  peer.channel = MESH_CHANNEL;  // 0 would mean "whatever channel we are on"
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;         // broadcast cannot be encrypted
  err = esp_now_add_peer(&peer);
  if (err != ESP_OK) {
    halt("esp_now_add_peer", err);
  }

  // The protocol bit above only makes LR available; this is what pins the
  // frames to it. Without it the driver is free to send at a legacy rate to a
  // peer it has no rate history for, which is every peer here - broadcast.
  esp_now_rate_config_t rate = {};
  rate.phymode = WIFI_PHY_MODE_LR;
  rate.rate    = MESH_PHY_RATE;
  rate.ersu    = false;
  rate.dcm     = false;
  err = esp_now_set_peer_rate_config(MESH_BROADCAST_ADDR, &rate);
  if (err != ESP_OK) {
    halt("esp_now_set_peer_rate_config", err);
  }

  err = esp_now_register_recv_cb(onEspNowRecv);
  if (err != ESP_OK) {
    halt("esp_now_register_recv_cb", err);
  }

  char mac[18];
  uint8_t self[6];
  esp_wifi_get_mac(WIFI_IF_STA, self);
  Serial.printf("[mesh] channel %d  tx power %.2f dBm (requested %.2f)  "
                "LR 250 kbps  our MAC %s\n",
                (int)MESH_CHANNEL, actualQdbm / 4.0f,
                MESH_TX_POWER_QDBM / 4.0f, meshFormatMac(self, mac));
  Serial.printf("[mesh] %s: hop limit %d, %d copies per frame, "
                "relay jitter %d-%d ms, dedup depth %d\n",
                gRole == MESH_ROLE_ORIGIN ? "origin (mints origSeq)"
                                          : "relay (never mints)",
                (int)MESH_HOP_LIMIT, (int)MESH_REPEATS,
                (int)MESH_RELAY_JITTER_MIN_MS, (int)MESH_RELAY_JITTER_MAX_MS,
                (int)MESH_DEDUP_DEPTH);
  Serial.printf("[mesh] frame %u bytes max (%u header + %u payload + %u crc)\n",
                (unsigned)MESH_MAX_FRAME_BYTES, (unsigned)MESH_HEADER_BYTES,
                (unsigned)MESH_MAX_PAYLOAD, (unsigned)MESH_CRC_BYTES);
  Serial.println("[mesh] radio ready");
}

// ---------------------------------------------------------------------------
// Publish - origin only
// ---------------------------------------------------------------------------

bool meshPublishLora(const uint8_t *lora, size_t loraLen) {
  if (gRole != MESH_ROLE_ORIGIN) {
    // A second node minting into the same u16 space produces collisions that
    // every dedup ring downstream reads as duplicates and silently swallows,
    // so this is refused rather than merely discouraged.
    gCounters.dropOversize++;
    return false;
  }

  const uint16_t seq = gNextOrigSeq;

  uint8_t frame[MESH_MAX_FRAME_BYTES];
  const size_t len = meshWrapLora(frame, sizeof(frame), MESH_HOP_LIMIT, seq,
                                  lora, loraLen);
  if (len == 0) {
    gCounters.dropOversize++;
    return false;
  }

  gNextOrigSeq = (uint16_t)(gNextOrigSeq + 1);
  if (gNextOrigSeq == 0) gNextOrigSeq = 1;
  gLastOrigSeq = seq;

  // Our own origSeq goes into our own ring: displays relay this frame back at
  // us, and without this the bridge would hand every echo to its handler as a
  // foreign origin and warn about itself.
  meshDedupInsert(&gDedup, seq);

  // No jitter here. The origin is the only node transmitting this frame at
  // this instant, so there is nothing to collide with, and the wait would be
  // pure latency on every screen.
  sendCopies(frame, len);
  gCounters.published++;
  return true;
}

// ---------------------------------------------------------------------------
// Service
// ---------------------------------------------------------------------------

static void countDecodeFailure(MeshDecodeStatus st) {
  switch (st) {
    case MESH_ERR_SHORT:   gCounters.dropShort++;   break;
    case MESH_ERR_MAGIC:   gCounters.dropMagic++;   break;
    case MESH_ERR_VERSION: gCounters.dropVersion++; break;
    case MESH_ERR_LENGTH:  gCounters.dropLength++;  break;
    case MESH_ERR_CRC:     gCounters.dropCrc++;     break;
    case MESH_OK:                                   break;
  }
}

static void serviceOneFrame(const RxSlot &slot, uint32_t nowMs) {
  MeshFrame frame;
  const MeshDecodeStatus st = meshUnpack(slot.bytes, slot.len, &frame);
  if (st != MESH_OK) {
    countDecodeFailure(st);
    return;
  }

  if (meshDedupSeenOrInsert(&gDedup, frame.origSeq)) {
    gCounters.deduped++;
    return;
  }
  gCounters.accepted++;

  if (gRole != MESH_ROLE_ORIGIN) {
    gLastOrigSeq = frame.origSeq;
  }

  // Relay decision first, handler second: the handler may redraw a panel or
  // write NVS, and a frame held behind that is latency on every node further
  // from the bridge than this one.
  uint8_t relay[MESH_MAX_FRAME_BYTES];
  memcpy(relay, slot.bytes, slot.len);
  if (meshDecrementHop(relay, slot.len)) {
    if (!queueRelay(relay, slot.len, nowMs)) {
      gCounters.dropRelayFull++;
    }
  } else {
    gCounters.dropHopExhausted++;
  }

  if (gHandler != NULL) {
    gHandler(frame, slot.mac, slot.rssi);
  }
}

void meshService() {
  // Fold in what the callback could not write itself.
  // heard and dropRxQueueFull have no other writer, so they are simply taken
  // over; dropLength shares a bucket with the decoder and takes the difference.
  gCounters.heard           = gCbHeard;
  gCounters.dropRxQueueFull = gCbQueueFull;
  const uint32_t tooLong = gCbTooLong;
  gCounters.dropLength += tooLong - gFoldedTooLong;
  gFoldedTooLong = tooLong;

  const uint32_t nowMs = millis();

  // Bounded by the ring depth rather than by "until empty": the producer runs
  // on another task and could otherwise keep this loop fed indefinitely while
  // loop() never gets to service the relays it is queueing.
  for (int drained = 0; drained < MESH_RX_QUEUE_DEPTH; drained++) {
    const uint8_t tail = gRxTail;
    if (tail == gRxHead) break;

    // Copied out before the tail moves, so the callback cannot overwrite the
    // slot while it is being decoded.
    RxSlot slot = gRx[tail];
    gRxTail = (uint8_t)((tail + 1) % MESH_RX_QUEUE_DEPTH);

    serviceOneFrame(slot, nowMs);
  }

  // Unsigned subtraction compared as signed, so a relay whose wait straddles
  // the 49-day millis() wrap fires instead of waiting out the whole rollover.
  for (int i = 0; i < MESH_RELAY_SLOTS; i++) {
    if (!gRelay[i].used) continue;
    if ((int32_t)(nowMs - gRelay[i].dueMs) < 0) continue;
    sendCopies(gRelay[i].bytes, gRelay[i].len);
    gRelay[i].used = false;
    gCounters.relayed++;
  }
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

const MeshCounters &meshCounters() {
  return gCounters;
}

uint16_t meshLastOrigSeq() {
  return gLastOrigSeq;
}

void meshPrintCounters() {
  Serial.printf("[mesh] heard=%lu accepted=%lu dedup=%lu relayed=%lu "
                "published=%lu | sends=%lu failed=%lu | lastSeq=%u | "
                "dropped: short=%lu magic=%lu version=%lu length=%lu crc=%lu "
                "hop=%lu rxfull=%lu relayfull=%lu oversize=%lu\n",
                (unsigned long)gCounters.heard,
                (unsigned long)gCounters.accepted,
                (unsigned long)gCounters.deduped,
                (unsigned long)gCounters.relayed,
                (unsigned long)gCounters.published,
                (unsigned long)gCounters.sends,
                (unsigned long)gCounters.sendFails,
                (unsigned)gLastOrigSeq,
                (unsigned long)gCounters.dropShort,
                (unsigned long)gCounters.dropMagic,
                (unsigned long)gCounters.dropVersion,
                (unsigned long)gCounters.dropLength,
                (unsigned long)gCounters.dropCrc,
                (unsigned long)gCounters.dropHopExhausted,
                (unsigned long)gCounters.dropRxQueueFull,
                (unsigned long)gCounters.dropRelayFull,
                (unsigned long)gCounters.dropOversize);
}
