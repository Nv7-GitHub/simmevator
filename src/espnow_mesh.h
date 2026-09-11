#pragma once
//
// The ESP-NOW flood layer: bridge -> displays -> displays.
//
// mesh_packet.h owns the frame: the envelope, the CRC, the hop decrement and
// the dedup ring, all board-agnostic so they can be exercised on the host.
// This file owns the radio and the schedule - bringing WiFi up, getting frames
// out of the PHY and deciding when a heard frame goes back on the air.
//
// One implementation runs on every node. The XIAO ESP32S3 bridge and the ten
// ESP32-WROOM-32 displays are different chips, so nothing here may assume one
// or the other: no pin, no chip-specific peripheral, and no IDF API that only
// one of the two cores carries.
//
// The only asymmetry is MeshRole. Both roles relay - spec section 4.2 makes
// the flood rule identical everywhere, which is what removes the routing
// table - but only MESH_ROLE_ORIGIN may mint origSeq, because a second node
// minting into the same u16 space would produce collisions the dedup ring
// reads as duplicates and silently swallows.
//
// ---------------------------------------------------------------------------
// Why relaying is jittered, and why that is not optional
// ---------------------------------------------------------------------------
// Ten displays hear the same frame within microseconds of each other. If each
// one relays as soon as it has decoded it, all ten transmit into the same air
// at the same instant: CSMA backoff is measured against a channel that was
// idle a moment ago, so they do not back off from each other, and every
// listener gets ten overlapping frames it can decode none of. The relay then
// dies exactly where the mesh needed it most, one hop from the bridge.
//
// A random MESH_RELAY_JITTER_MIN_MS..MAX_MS wait spreads those ten transmits
// across a window far wider than the ~1.3 ms a 42-byte frame occupies at the
// LR rate, so they queue instead of colliding. This is the difference between
// a mesh that works and one that does not; it is not a tuning refinement.
//
// ---------------------------------------------------------------------------
// Why nothing is done in the receive callback
// ---------------------------------------------------------------------------
// esp_now_register_recv_cb installs a callback that runs on the WiFi task, not
// on loop(). Anything slow there - a CRC over the frame, a Serial.printf, a
// TFT write, and above all the relay jitter wait - stalls the stack that is
// trying to hand over the next frame, and frames received during the stall are
// dropped inside the driver where this layer cannot even count them. The
// callback therefore does exactly one thing: memcpy the bytes into a ring and
// return. Every decision below happens in meshService(), from loop().
//

#include <Arduino.h>
#include <esp_now.h>

#include "mesh_packet.h"

// ---------------------------------------------------------------------------
// Radio configuration - platformio.ini [mesh_base] sets all of these, and sets
// them identically for bridge_rx and floor_display. The defaults here exist so
// the file still compiles if it is pulled into an environment that forgot.
// ---------------------------------------------------------------------------

// Nothing associates with an AP, so this is simply the channel the raw frames
// go out on. Every node must agree or they are deaf to each other.
#ifndef MESH_CHANNEL
#define MESH_CHANNEL 1
#endif

// Maximum transmit power in quarter-dBm. esp_wifi_set_max_tx_power accepts
// [8, 84] and quantises; 84 lands on the hardware ceiling of 80 (20 dBm).
// meshBringUp reads the value back and reports what the PHY actually granted
// rather than what was asked for - the ceiling also depends on the calibration
// data flashed into the module, which differs between these two boards.
#ifndef MESH_TX_POWER_QDBM
#define MESH_TX_POWER_QDBM 84
#endif

// Back-to-back sends of each frame. A flood has no delivery guarantee; three
// copies make loss unlikely, not impossible.
#ifndef MESH_REPEATS
#define MESH_REPEATS 3
#endif

// The anti-collision window described above.
#ifndef MESH_RELAY_JITTER_MIN_MS
#define MESH_RELAY_JITTER_MIN_MS 5
#endif
#ifndef MESH_RELAY_JITTER_MAX_MS
#define MESH_RELAY_JITTER_MAX_MS 40
#endif

// platformio.ini names the dedup depth MESH_DEDUP_RING; mesh_packet.h, which
// predates the build flag and is host-testable on its own, names it
// MESH_DEDUP_DEPTH. espnow_mesh.cpp static_asserts that the two agree, so a
// build flag edited in one place and not the other is a compile error rather
// than a mesh that quietly dedups over a different window than documented.
#ifndef MESH_DEDUP_RING
#define MESH_DEDUP_RING MESH_DEDUP_DEPTH
#endif

// The 802.11 LR rate frames are transmitted at. Espressif's long-range PHY is
// worth roughly 7 dB of sensitivity over 802.11b, which is exactly the margin
// a floor-to-floor link through reinforced concrete is short of. 250 kbps
// rather than 500 kbps: it is the more sensitive of the two LR rates, and at
// 42 bytes a frame the airtime difference is under a millisecond, which no
// part of this system can measure.
#ifndef MESH_PHY_RATE
#define MESH_PHY_RATE WIFI_PHY_RATE_LORA_250K
#endif

// ---------------------------------------------------------------------------
// Queue depths
//
// Both are sized against the burst the flood can actually produce: one STATE
// frame reaches a node from up to a handful of neighbours at once, each sent
// MESH_REPEATS times. Everything past the first copy is a dedup hit that costs
// a ring scan and nothing else, so these only have to cover the burst, not the
// traffic rate.
// ---------------------------------------------------------------------------
#ifndef MESH_RX_QUEUE_DEPTH
#define MESH_RX_QUEUE_DEPTH 12
#endif

// Frames waiting out their jitter. A node only ever relays frames it has not
// seen, so this holds distinct origSeqs, not copies - 8 is well past anything
// the 2 s STATE cadence can produce inside a 40 ms window.
#ifndef MESH_RELAY_SLOTS
#define MESH_RELAY_SLOTS 8
#endif

// ---------------------------------------------------------------------------
// Role
// ---------------------------------------------------------------------------
enum MeshRole {
  MESH_ROLE_ORIGIN,  // the bridge: mints origSeq, and relays like everyone else
  MESH_ROLE_RELAY    // a display: relays only, meshPublishLora is refused
};

// ---------------------------------------------------------------------------
// Counters
//
// A node's own view of the mesh, so a display on floor 9 can be asked over
// Serial whether it is hearing the bridge directly, hearing it through its
// neighbours, or hearing a frame per relay and throwing most of them away.
// Written only from loop(), so they can be read without a lock.
// ---------------------------------------------------------------------------
struct MeshCounters {
  uint32_t heard;      // frames the WiFi task handed us, before any validation
  uint32_t accepted;   // decoded, and new to this node
  uint32_t deduped;    // decoded, but this origSeq had already been seen
  uint32_t relayed;    // put back on the air after the jitter wait
  uint32_t published;  // origin only: frames minted here
  uint32_t sends;      // esp_now_send calls that the driver accepted
  uint32_t sendFails;  // esp_now_send calls that it did not

  // Drops, by reason. The first five mirror MeshDecodeStatus; the rest are
  // this layer's own.
  uint32_t dropShort;
  uint32_t dropMagic;
  uint32_t dropVersion;
  uint32_t dropLength;
  uint32_t dropCrc;
  uint32_t dropHopExhausted;  // reached its hop limit here and goes no further
  uint32_t dropRxQueueFull;   // loop() did not drain fast enough
  uint32_t dropRelayFull;     // more distinct frames in flight than slots
  uint32_t dropOversize;      // asked to publish something this node may not
                              // flood: the wrong role, or a body that does not
                              // fit the envelope
};

// ---------------------------------------------------------------------------
// Receive callback
//
// Called from meshService(), never from the WiFi task, so it may take its time
// - redraw a screen, write NVS, print. `rssi` is the frame as this node heard
// it, or 0 on IDF versions whose ESP-NOW callback does not carry the radio
// metadata. The relay decision has already been made and does not depend on
// what the handler does.
// ---------------------------------------------------------------------------
typedef void (*MeshFrameHandler)(const MeshFrame &frame, const uint8_t *srcMac,
                                 int8_t rssi);

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// ff:ff:ff:ff:ff:ff - registered as a peer by meshBringUp. Broadcast is the
// whole point: there are no MAC addresses to configure anywhere in this
// system, so a display can be swapped for a spare with no reflash of anything
// else.
extern const uint8_t MESH_BROADCAST_ADDR[6];

// Brings up WiFi in station mode on MESH_CHANNEL at maximum power, switches
// the PHY to LR, starts ESP-NOW and registers the broadcast peer. Halts (with
// an explanation on Serial) on failure, since every later call would fail the
// same way. `roleName` is for the banner only.
void meshBringUp(MeshRole role, MeshFrameHandler onFrame, const char *roleName);

// Origin only. Wraps a LoRa packet exactly as it came off the SX1262 - tag,
// txId, seq and body - mints the next origSeq, and floods it. Returns false
// and counts a drop if the role is not MESH_ROLE_ORIGIN or the packet does not
// fit the envelope.
bool meshPublishLora(const uint8_t *lora, size_t loraLen);

// Call every loop(). Drains the receive ring, and puts relays on the air as
// their jitter expires. Never blocks: a frame whose wait has not elapsed is
// left for a later call.
void meshService();

const MeshCounters &meshCounters();

// One line of the above to Serial, for a node reporting its own view.
void meshPrintCounters();

// The last origSeq minted (origin) or accepted (relay). 0 means none yet,
// which is why minting starts at 1.
uint16_t meshLastOrigSeq();

// Prints an esp_err_t as a name where we know one.
const char *meshStatusName(esp_err_t err);

// Formats a MAC into a caller-supplied buffer of at least 18 bytes.
const char *meshFormatMac(const uint8_t *mac, char *out);
