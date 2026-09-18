#pragma once
//
// Wio-SX1262 bring-up and link, shared by both radio nodes.
//
// Hardware: Seeed XIAO ESP32S3 + Wio-SX1262 expansion module, on both the car
// node and the bridge. The radio settings are identical at both ends - only
// the direction differs, which is why this is one module and not two: the car
// transmits STATE/STATS packets and the bridge sits in receive.
//
// Ported from elevatormons' lora_link, which is proven on this stack. The two
// things added here are the receive side (elevatormons' version only ever
// transmitted from this module) and explicit sleep.
//
// --- on sleep ---
// The car node is the reason loraSleep()/loraWake() are in the API rather than
// hidden inside a transmit call. The SX1262 idles at several mA in standby,
// and the spec 2 budget has ~19 mA of total headroom for a 30-day battery -
// leaving the radio in standby between the 2 s STATE packets would spend a
// large fraction of that doing nothing. So the car is expected to sleep the
// radio after every transmit and wake it just before the next, and making that
// explicit means a caller that forgets is a visible omission rather than a
// battery that quietly lasts a week. The bridge is wall powered and never
// sleeps the radio: it must stay in receive to hear the car.
//
// Sleep is warm - the configuration is retained across it, so loraWake() is a
// standby command and not a second bring-up.
//

#include <Arduino.h>
#include <RadioLib.h>

// ---------------------------------------------------------------------------
// Pin map: Wio-SX1262 as seen by the XIAO ESP32S3 host
// ---------------------------------------------------------------------------
#define LORA_NSS   41   // SPI chip select
#define LORA_DIO1  39   // IRQ
#define LORA_NRST  42   // reset
#define LORA_BUSY  40   // busy
#define LORA_SCK    7   // D8
#define LORA_MISO   8   // D9
#define LORA_MOSI   9   // D10

// The module carries a 1.8 V TCXO on DIO3 and uses DIO2 as the RF switch.
#define LORA_TCXO_VOLTAGE 1.8f

// ---------------------------------------------------------------------------
// Radio configuration (US 915 MHz ISM band)
//
// The real values come from [lora_base] in platformio.ini, where the evidence
// behind each one is written down. These fallbacks only keep the file
// compilable on its own.
// ---------------------------------------------------------------------------
// LORA_FREQUENCY is the one radio setting that is NOT shared, and so it is the
// one with no fallback. 913.0 / 915.0 / 917.0 MHz come from [elev_a] /
// [elev_b] / [elev_c], one section per shaft (spec 2.1).
//
// A default here would read as harmless and would not be. Every unset build
// would quietly become elevator B, so the three cars would land back on one
// frequency - the pure-ALOHA case that loses about one STATE packet in three -
// and a bridge built without an elevator would sit 2 MHz away from its own car
// and simply never hear it. Neither failure shows up at compile time, and the
// second one looks exactly like a dead radio. Fail here instead.
#ifndef LORA_FREQUENCY
#error "LORA_FREQUENCY is not set - build one of the per-elevator environments (elevator_tx_a/_b/_c, bridge_rx_a/_b/_c), which pull it from [elev_a]/[elev_b]/[elev_c] in platformio.ini (spec 5.1)"
#endif
#ifndef LORA_BANDWIDTH
#define LORA_BANDWIDTH 125.0f   // kHz
#endif
#ifndef LORA_SPREADING_FACTOR
#define LORA_SPREADING_FACTOR 10
#endif
#ifndef LORA_CODING_RATE
#define LORA_CODING_RATE 8      // 4/8
#endif
#ifndef LORA_SYNC_WORD
#define LORA_SYNC_WORD 0x34     // private network sync word
#endif
#ifndef LORA_TX_POWER
#define LORA_TX_POWER 22        // dBm; SX1262 maximum
#endif
#ifndef LORA_PREAMBLE_LEN
#define LORA_PREAMBLE_LEN 8
#endif

// ---------------------------------------------------------------------------
// Shared radio bring-up
// ---------------------------------------------------------------------------
extern SPIClass loraSpi;
extern SX1262 radio;

// Brings up SPI and the SX1262 with the settings above. Halts (with an
// explanation on Serial) if the radio cannot be initialised, since every later
// call would fail the same way. The radio is left in standby, not receive -
// call loraStartReceive() if this node listens.
void loraBringUp(const char *roleName);

// Prints a RadioLib status code as a name where we know one.
const char *loraStatusName(int16_t state);

// How many times the settings above have had to be put back on a radio found
// without them - a receive that would not re-arm, or a chip that came back
// from a supply sag at power-on defaults. It should stay 0; a main is expected
// to print it, because a count that climbs is the only outward sign of a
// supply or wiring fault that otherwise reads as ordinary packet loss.
uint32_t loraRecoveryCount();

// ---------------------------------------------------------------------------
// Transmit side - the car node
// ---------------------------------------------------------------------------

// Sends one packet and returns when the radio is done with it, so the caller
// can act on the RadioLib status. An 8-byte STATE is 297 ms of airtime and a
// 24-byte STATS is 494 ms (spec 2.1); the car's 1 Hz sampler must not be
// inside this call, so transmit from a point in the loop where losing half a
// second costs nothing.
int16_t loraTransmit(const uint8_t *data, size_t len);

// ---------------------------------------------------------------------------
// Receive side - the bridge
//
// Interrupt driven: DIO1 fires on a completed packet and sets a flag, and
// loraPoll() does the SPI read from the main loop. Reading a packet out of the
// ISR would mean SPI transactions at interrupt time, which is how a shared bus
// gets corrupted.
// ---------------------------------------------------------------------------

// Puts the radio into continuous receive and attaches the DIO1 handler.
// Returns a RadioLib status; a failure here means this node is deaf.
int16_t loraStartReceive();

// Non-blocking. Returns false when no packet has arrived since the last call.
// On true, buf holds len bytes and loraLastRssi()/loraLastSnr() describe that
// packet. A frame the radio rejected (CRC mismatch, in-progress corruption)
// also returns true with *outState set to the failure, so the caller can count
// it - the payload in that case is not to be trusted.
bool loraPoll(uint8_t *buf, size_t cap, size_t *outLen, int16_t *outState);

// Link quality of the most recently polled packet.
float loraLastRssi();   // dBm
float loraLastSnr();    // dB

// ---------------------------------------------------------------------------
// Power - see the note at the top of this file
// ---------------------------------------------------------------------------

// Drops the radio to sleep with its configuration retained. Anything in flight
// is abandoned, so only call this once a transmit has returned.
void loraSleep();

// Brings the radio back to standby, ready to transmit or to be put back into
// receive. Cheap - this is a standby command, not a re-initialisation.
void loraWake();

// Whether the radio is currently asleep, so a caller can avoid a redundant
// wake without tracking the state itself.
bool loraIsAsleep();
