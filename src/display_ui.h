#pragma once
//
// The floor display's rendering layer - ELEGOO CYD, ILI9341 240x320 panel run
// in landscape, so the canvas is 320 wide by 240 tall.
//
// This module draws and nothing else. It owns the panel, the backlight, the
// RGB LED and the floor-label table; it owns no radio, no mesh, no packet
// parsing and no clock. floor_display.cpp receives STATE and STATS over
// ESP-NOW, decides what is true and how stale it is, fills in a
// DisplayUiState, and calls displayUpdate(). That split is what lets the
// layout be reasoned about - and changed - without touching the mesh, and
// what stops the renderer from quietly inventing a floor when a packet is
// late.
//
// Layout, spec section 6:
//
//   +----------------------------+--------------+
//   |                            |      ^       |  arrow only while moving
//   |      ######     ##         |     UP       |
//   |      ##  ##     ##         |              |
//   |      ##  ##     ##         |   12.7 V     |  colour-coded bar
//   |      ##  ##     ##         |   ####__     |
//   |      ######     ##         |              |
//   |                            |   2.1 mi     |  last 24 h
//   |                            |   today      |
//   +----------------------------+--------------+
//        left ~200 px                 right ~120 px
//
// ---------------------------------------------------------------------------
// Why the big number is drawn rather than typed
// ---------------------------------------------------------------------------
// The floor number has to be readable from the far end of a corridor, which
// puts it at ~160-190 px tall. Every bitmap font TFT_eSPI can carry is
// designed at 8-48 px and scaled up by integer pixel replication, so at that
// size the edges are 4-8 px staircases and the whole glyph looks soft. Seven
// filled polygons per digit cost a few hundred microseconds, scale exactly,
// and stay sharp. sevenSegDigit() below is the whole mechanism: give it a
// position, a height, a thickness and a colour and everything else in this
// file is derived from the height, so changing DISPLAY_DIGIT_MAX_H rescales
// the number and nothing else needs editing.
//

#include <Arduino.h>
#include <TFT_eSPI.h>

// ---------------------------------------------------------------------------
// Board wiring - defaults match the ESP32-2432S028R; platformio.ini sets them
// ---------------------------------------------------------------------------

// Ambient light sensor. ADC1_CH6, input-only pad, so there is nothing to
// share it with.
#ifndef LDR_PIN
#define LDR_PIN 34
#endif

// The onboard RGB LED. These are ACTIVE LOW: each cathode-side pin sinks
// through the LED to 3V3, so driving a pin LOW lights that channel and driving
// it HIGH puts it out. Getting this backwards gives a display that glows white
// when everything is fine and goes dark when the link dies - exactly inverted
// from what the corridor needs. Everything in display_ui.cpp works in
// "0 = off, 255 = full" terms and inverts once, at the pin.
#ifndef RGB_LED_R_PIN
#define RGB_LED_R_PIN 4
#endif
#ifndef RGB_LED_G_PIN
#define RGB_LED_G_PIN 16
#endif
#ifndef RGB_LED_B_PIN
#define RGB_LED_B_PIN 17
#endif

// ---------------------------------------------------------------------------
// Floor labels
// ---------------------------------------------------------------------------
// A comma-separated list, parsed once at startup into an indexable table.
// Simmons serves 1-10 with no basement so index n maps to label n, but the
// table is the reason a basement ("B,1,2,...") or a skipped floor
// ("1,2,3,5,6" in a building with no 4) is a build flag rather than a code
// change. The renderer never formats a floor index as a number directly.
#ifndef FLOOR_LABELS
#define FLOOR_LABELS "1,2,3,4,5,6,7,8,9,10"
#endif
#ifndef FLOOR_LABEL_COUNT
#define FLOOR_LABEL_COUNT 10
#endif

// Hard ceiling on the parsed table. The algorithm's own ladder is a 64-entry
// array centred on +32, so no building this code can track has more than 64
// floors; 32 is well past the tallest thing a single LoRa hop covers anyway.
#ifndef DISPLAY_MAX_LABELS
#define DISPLAY_MAX_LABELS 32
#endif

// A label is drawn as seven-segment glyphs, and two of them already fill the
// left pane. Anything longer is truncated rather than shrunk, because a number
// that changes size on arrival reads as a fault.
#ifndef DISPLAY_MAX_LABEL_CHARS
#define DISPLAY_MAX_LABEL_CHARS 2
#endif

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

// Landscape. TFT_WIDTH/TFT_HEIGHT in platformio.ini are the panel's native
// portrait dimensions; these are the canvas after setRotation().
#define DISPLAY_W 320
#define DISPLAY_H 240

// The vertical rule between the number and the readouts. 200 leaves the number
// two full-size digits and still gives the right pane room for "12.7 V" in
// font 4 without hyphenating anything.
#ifndef DISPLAY_SPLIT_X
#define DISPLAY_SPLIT_X 200
#endif

// Tallest a digit is allowed to get. One digit is height-limited by this;
// two digits are width-limited by the left pane and come out around 158 px.
// Both are set from here - see displayNumberHeight() in the .cpp.
#ifndef DISPLAY_DIGIT_MAX_H
#define DISPLAY_DIGIT_MAX_H 190
#endif

// ---------------------------------------------------------------------------
// Seven-segment proportions
//
// Everything is a ratio of the digit height, so one number controls the size.
// ---------------------------------------------------------------------------

// Stroke thickness, as height/DISPLAY_SEG_THICK_DIV. 1/6 is what a moulded LED
// module uses: noticeably thinner loses contrast at corridor distance, and
// noticeably thicker starts closing the counters in 0, 8 and 9.
#ifndef DISPLAY_SEG_THICK_DIV
#define DISPLAY_SEG_THICK_DIV 6
#endif

// Digit width as height * NUM / DEN. 0.55 is the classic seven-segment aspect;
// squarer digits read as a dot-matrix panel rather than a floor indicator.
#ifndef DISPLAY_SEG_ASPECT_NUM
#define DISPLAY_SEG_ASPECT_NUM 11
#endif
#ifndef DISPLAY_SEG_ASPECT_DEN
#define DISPLAY_SEG_ASPECT_DEN 20
#endif

// ---------------------------------------------------------------------------
// Backlight
// ---------------------------------------------------------------------------

// 5 kHz, 8-bit. Above anything the panel's backlight transistor can make
// audible, and far above the ~200 Hz where a phone camera pointed at the
// screen picks up banding.
#ifndef DISPLAY_BL_PWM_HZ
#define DISPLAY_BL_PWM_HZ 5000
#endif
#ifndef DISPLAY_BL_PWM_BITS
#define DISPLAY_BL_PWM_BITS 8
#endif

// Never fully off. A dark panel in a dark corridor is indistinguishable from a
// dead one, and the whole point of the display is to be believed.
#ifndef DISPLAY_BL_MIN
#define DISPLAY_BL_MIN 24
#endif
#ifndef DISPLAY_BL_MAX
#define DISPLAY_BL_MAX 255
#endif

// The CYD wires the LDR as the upper leg of a divider to 3V3, so the ADC reads
// LOW in bright light and climbs towards full scale in the dark - the opposite
// of what most people assume. Set this to 0 if a batch turns up strapped the
// other way; it is the only thing that has to change.
#ifndef DISPLAY_LDR_BRIGHT_IS_LOW
#define DISPLAY_LDR_BRIGHT_IS_LOW 1
#endif

// The two ends of the mapping, in raw 12-bit ADC counts at 11 dB attenuation.
// Measured on a bench CYD: ~400 under normal corridor lighting, ~3000 with the
// room lights off. Anything outside the pair clamps.
#ifndef DISPLAY_LDR_BRIGHT_RAW
#define DISPLAY_LDR_BRIGHT_RAW 400
#endif
#ifndef DISPLAY_LDR_DARK_RAW
#define DISPLAY_LDR_DARK_RAW 3000
#endif

// How often the LDR is sampled, and the smoothing applied to it as a shift:
// filtered += (raw - filtered) >> SHIFT. At 100 ms and shift 4 the time
// constant is ~1.6 s, so a person walking past - under a second of shadow -
// moves the filtered value by well under the hysteresis band below and the
// backlight never reacts to them at all.
#ifndef DISPLAY_LDR_SAMPLE_MS
#define DISPLAY_LDR_SAMPLE_MS 100
#endif
#ifndef DISPLAY_LDR_FILTER_SHIFT
#define DISPLAY_LDR_FILTER_SHIFT 4
#endif

// Deadband around the currently committed target, in duty counts. A new target
// is only adopted once the filtered light level asks for something this far
// away, which is what stops the duty oscillating between two adjacent values
// when the ambient level sits exactly on a boundary.
#ifndef DISPLAY_BL_HYSTERESIS
#define DISPLAY_BL_HYSTERESIS 14
#endif

// Ramp rate: one duty count per this many milliseconds. 8 ms makes a full
// 0 -> 255 sweep take ~2 s. That is the trade-off in both directions - fast
// enough that walking into a lit corridor does not leave the screen dim behind
// you, slow enough that the change reads as a fade rather than a step, which
// is what the eye interprets as a fault.
#ifndef DISPLAY_BL_RAMP_MS
#define DISPLAY_BL_RAMP_MS 8
#endif

// ---------------------------------------------------------------------------
// Battery thresholds (spec 3.2), in millivolts
//
// A 4S LiFePO4 discharge curve is almost flat, so the bar is a coarse
// indicator and not a fuel gauge - see spec section 9. These are the numbers
// the colour changes at.
// ---------------------------------------------------------------------------
#ifndef DISPLAY_VBAT_FULL_MV
#define DISPLAY_VBAT_FULL_MV 13300
#endif
#ifndef DISPLAY_VBAT_NOMINAL_MV
#define DISPLAY_VBAT_NOMINAL_MV 12800
#endif
#ifndef DISPLAY_VBAT_WARN_MV
#define DISPLAY_VBAT_WARN_MV 12000
#endif
#ifndef DISPLAY_VBAT_CRITICAL_MV
#define DISPLAY_VBAT_CRITICAL_MV 11200
#endif

// ---------------------------------------------------------------------------
// Staleness (spec 6)
// ---------------------------------------------------------------------------
// These are not used by this module - the caller times the packets and hands
// over two booleans - but they are the thresholds the rendering was designed
// around, so they live next to it rather than being open-coded in
// floor_display.cpp.
//
// Both thresholds are measured against the STATS heartbeat, and neither against
// STATE. That is deliberate, and it is a correction to what spec 6 originally
// said.
//
// STATE is only sent while the car is moving, plus a 10 s hold after it stops
// (spec 2.2). So silence on STATE is the NORMAL parked condition - it means
// nothing is happening, not that anything is wrong, and the floor on the screen
// is perfectly current. Dimming on it, as the first version did at 10 s, left
// every screen dimmed through every idle period: most of a working day, and all
// of a night. The question a dimmed screen should answer is "am I still hearing
// the transmitter", and the 60 s STATS heartbeat is the only thing that answers
// it, because STATS goes out whether the car moves or not.
//
// 150 s is two missed heartbeats plus a margin. One miss must never dim a
// screen: spec 9 is explicit that the flood mesh has no delivery guarantee, so
// a single loss is expected and a threshold that flapped on it would have ten
// screens blinking at each other. 180 s is three missed heartbeats, at which
// point the link is not merely unlucky.
#ifndef DISPLAY_DIM_STALE_MS
#define DISPLAY_DIM_STALE_MS 150000
#endif
#ifndef DISPLAY_STATS_STALE_MS
#define DISPLAY_STATS_STALE_MS 180000
#endif

// ---------------------------------------------------------------------------
// What the renderer is given
// ---------------------------------------------------------------------------

struct DisplayUiState {
  // 1-based confirmed floor index into the label table, or 0 for "no confirmed
  // floor" - a fresh transmitter whose model is not ready yet, or a display
  // that has not heard anything since boot. 0 draws as "--".
  uint8_t floorIndex;

  // Live fractional position in floor units, 1.0 == floor 1. While moving the
  // number tracks this so it sweeps 3 -> 4 -> 5; on arrival floorIndex wins.
  // Ignored unless positionValid.
  float position;
  bool  positionValid;

  bool    moving;
  uint8_t direction;  // ELEV_DIR_IDLE / _UP / _DOWN; only drawn while moving

  // Battery and odometer come from STATS, so they may legitimately be absent
  // for the first minute after boot. Both are drawn as "--" when not valid.
  float batteryVolts;
  bool  batteryValid;
  float dist24hMiles;
  bool  distValid;

  // No STATS for DISPLAY_DIM_STALE_MS: the readout dims. No STATS for
  // DISPLAY_STATS_STALE_MS: it greys out and the LED goes amber. statsStale
  // implies the worse of the two. Neither is driven by STATE - see the note on
  // the thresholds above for why silence on STATE means nothing is wrong.
  bool stateStale;
  bool statsStale;
};

// A DisplayUiState with everything unknown and nothing stale - the state to
// start from, so a caller never has to remember to zero a field.
DisplayUiState displayUiStateInit();

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

// Parses FLOOR_LABELS, brings up the panel in landscape, allocates the number
// sprite, starts the backlight at its midpoint and puts the LED out. Paints
// the static chrome. Returns false only if the number sprite could not be
// allocated - the display still works in that case, it just repaints the
// number directly and flickers while doing it.
bool displayBegin();

// Call from loop() as often as convenient. Samples the LDR, advances the
// backlight ramp one step at a time and does nothing else. Cheap enough to
// call every pass; the two internal timers decide when there is work.
void displayTick();

// Region-limited repaint. Cheap to call at any rate - see the invalidation
// rule in the .cpp - so the caller can simply call it whenever anything might
// have changed rather than working out whether it did.
void displayUpdate(const DisplayUiState &s);

// Forces the next displayUpdate() to repaint everything, chrome included.
// For use after something else has drawn over the screen.
void displayInvalidate();

// A centred two-line message in ordinary fonts, for boot and for "waiting for
// the first packet". Invalidates, so the next displayUpdate() repaints in
// full.
void displaySplash(const char *line1, const char *line2);

// ---------------------------------------------------------------------------
// Backlight and LED
// ---------------------------------------------------------------------------

// Current backlight duty, 0-255, after ramping. Diagnostics only.
uint8_t displayBacklight();

// Pins the backlight at a fixed duty and stops the LDR from moving it; pass a
// negative value to hand control back to the light sensor.
void displayBacklightOverride(int duty);

// 0-255 per channel, in "0 = off" terms. The active-low inversion happens
// here and nowhere else.
void displayLedRgb(uint8_t r, uint8_t g, uint8_t b);

// ---------------------------------------------------------------------------
// Floor labels
// ---------------------------------------------------------------------------

// Label for a 1-based floor index, or "--" if the index is 0 or past the end
// of the parsed table. Never returns NULL.
const char *displayFloorLabel(uint8_t floorIndex);

// How many labels FLOOR_LABELS actually parsed into. Compared against
// FLOOR_LABEL_COUNT at startup and reported on the console if they disagree,
// because a mistyped label list otherwise shows up as a display that is
// silently one floor off at the top of the building.
uint8_t displayFloorLabelCount();

// ---------------------------------------------------------------------------
// Seven-segment primitives
//
// Public because they are the interesting part, and because they take a
// TFT_eSPI& - which a TFT_eSprite also is - so the same code draws into the
// sprite and straight onto the panel.
// ---------------------------------------------------------------------------

// Stroke thickness and cell width for a given digit height, both derived from
// the ratios above. At least 1 px and 1 px respectively.
int sevenSegThickness(int height);
int sevenSegWidth(int height);

// Gap between adjacent digit cells. Proportional, so a two-digit group looks
// the same at any size.
int sevenSegSpacing(int height);

// Total width of a string drawn by sevenSegString(), for centring.
int sevenSegStringWidth(const char *text, int height);

// One glyph, top-left at (x, y), in filled polygons. Digits 0-9 render as the
// familiar seven segments; a handful of letters that a seven-segment display
// can actually form (A b C d E F G H J L n o P r S U y) render as their usual
// approximations, and anything else - including a space - renders as the
// middle bar alone, which is what "unknown floor" should look like.
void sevenSegDigit(TFT_eSPI &gfx, int x, int y, int height, int thickness,
                   uint16_t colour, char glyph);

// A whole label, left edge at x. Returns the width drawn.
int sevenSegString(TFT_eSPI &gfx, int x, int y, int height, int thickness,
                   uint16_t colour, const char *text);
