//
// Rendering for the floor display. See display_ui.h for the layout, the wiring
// and the reasoning behind every constant.
//
#include "display_ui.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "elev_packet.h"  // ELEV_DIR_* only - nothing here parses a packet

// ===========================================================================
// Palette
//
// RGB565. The three ink colours are the staleness ladder from spec section 6,
// and they are the only thing staleness changes: the backlight belongs to the
// LDR, so dimming the *readout* means dimming the ink, not the panel. A
// display that dropped its backlight when the link died would be indis-
// tinguishable at a glance from one in a dark corridor.
// ===========================================================================
static const uint16_t COL_BG        = 0x0000;  // black
static const uint16_t COL_INK_FRESH = 0xFFFF;  // white
static const uint16_t COL_INK_DIM   = 0x9CF3;  // ~60% grey, no STATS for 150 s
static const uint16_t COL_INK_GREY  = 0x5ACB;  // ~35% grey, no STATS for 180 s
static const uint16_t COL_CHROME    = 0x3186;  // the divider rule
static const uint16_t COL_BAR_OK    = 0x07E0;  // green,  battery ok
static const uint16_t COL_BAR_WARN  = 0xFD20;  // orange, CHARGE SOON
static const uint16_t COL_BAR_LOW   = 0xF800;  // red,    CHARGE BATTERY

// ===========================================================================
// Region geometry
//
// All of it derived from DISPLAY_SPLIT_X and the canvas size, so moving the
// rule moves the layout.
// ===========================================================================

// The number's sprite. 4 px of margin on each side of the left pane.
static const int16_t NUM_X = 4;
static const int16_t NUM_Y = 14;
static const int16_t NUM_W = DISPLAY_SPLIT_X - 2 * NUM_X;
static const int16_t NUM_H = 192;   // ends at y = 206, clear of the banner

// The CHARGE BATTERY banner: a full-width strip along the bottom. The number
// moved up 8 px and lost 4 px of height to make room, rather than having the
// banner overlap it - the sprite repaints its whole rectangle, so an overlap
// would erase the banner's top edge on every floor change. 4 px off a ~190 px
// digit is invisible from a corridor; a banner that flickers is not.
static const int16_t BANNER_H = 30;
static const int16_t BANNER_Y = DISPLAY_H - BANNER_H;

// Right pane, inset 6 px from the rule and from the right edge.
static const int16_t RP_X  = DISPLAY_SPLIT_X + 6;
static const int16_t RP_W  = DISPLAY_W - RP_X - 6;
static const int16_t RP_CX = RP_X + RP_W / 2;

static const int16_t ARROW_Y = 6,   ARROW_H = 72;
static const int16_t VOLT_Y  = 82,  VOLT_H  = 28;
static const int16_t BAR_Y   = 114, BAR_H   = 20;
static const int16_t DIST_Y  = 150, DIST_H  = 50;

// Battery bar, centred in the BAR region.
static const int16_t BAR_W = 100;

// ===========================================================================
// Seven-segment glyphs
//
// Bit per segment, in the usual order:
//
//        aaaa
//       f    b
//       f    b
//        gggg
//       e    c
//       e    c
//        dddd
// ===========================================================================
static const uint8_t SEG_A = 0x01, SEG_B = 0x02, SEG_C = 0x04, SEG_D = 0x08;
static const uint8_t SEG_E = 0x10, SEG_F = 0x20, SEG_G = 0x40;

static uint8_t segmentsFor(char glyph) {
  switch (glyph) {
    case '0': return SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
    case '1': return SEG_B | SEG_C;
    case '2': return SEG_A | SEG_B | SEG_D | SEG_E | SEG_G;
    case '3': return SEG_A | SEG_B | SEG_C | SEG_D | SEG_G;
    case '4': return SEG_B | SEG_C | SEG_F | SEG_G;
    case '5': return SEG_A | SEG_C | SEG_D | SEG_F | SEG_G;
    case '6': return SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
    case '7': return SEG_A | SEG_B | SEG_C;
    case '8': return SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
    case '9': return SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G;
    // The letters a seven-segment cell can actually form without ambiguity.
    // This is what makes "B" for a basement or "M" -> "G" for a ground floor
    // a FLOOR_LABELS edit rather than a new rendering path.
    case 'A': case 'a': return SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G;
    case 'B': case 'b': return SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
    case 'C': case 'c': return SEG_A | SEG_D | SEG_E | SEG_F;
    case 'D': case 'd': return SEG_B | SEG_C | SEG_D | SEG_E | SEG_G;
    case 'E': case 'e': return SEG_A | SEG_D | SEG_E | SEG_F | SEG_G;
    case 'F': case 'f': return SEG_A | SEG_E | SEG_F | SEG_G;
    case 'G': case 'g': return SEG_A | SEG_C | SEG_D | SEG_E | SEG_F;
    case 'H': case 'h': return SEG_B | SEG_C | SEG_E | SEG_F | SEG_G;
    case 'J': case 'j': return SEG_B | SEG_C | SEG_D | SEG_E;
    case 'L': case 'l': return SEG_D | SEG_E | SEG_F;
    case 'N': case 'n': return SEG_C | SEG_E | SEG_G;
    case 'O': case 'o': return SEG_C | SEG_D | SEG_E | SEG_G;
    case 'P': case 'p': return SEG_A | SEG_B | SEG_E | SEG_F | SEG_G;
    case 'R': case 'r': return SEG_E | SEG_G;
    case 'S': case 's': return SEG_A | SEG_C | SEG_D | SEG_F | SEG_G;
    case 'U': case 'u': return SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
    case 'Y': case 'y': return SEG_B | SEG_C | SEG_D | SEG_F | SEG_G;
    default:            return SEG_G;  // '-', ' ', anything unmapped
  }
}

int sevenSegThickness(int height) {
  int t = height / DISPLAY_SEG_THICK_DIV;
  return t < 1 ? 1 : t;
}

int sevenSegWidth(int height) {
  int w = height * DISPLAY_SEG_ASPECT_NUM / DISPLAY_SEG_ASPECT_DEN;
  return w < 1 ? 1 : w;
}

int sevenSegSpacing(int height) {
  int s = height / 9;
  return s < 1 ? 1 : s;
}

// A convex polygon as a triangle fan from vertex 0. Every segment below is a
// six-sided convex slab, so this is the only fill primitive needed - and it
// goes through TFT_eSPI's virtual drawFastHLine, which is what lets the same
// call draw into a sprite.
static void fillConvexPoly(TFT_eSPI &gfx, const int16_t *xs, const int16_t *ys,
                           int n, uint16_t colour) {
  for (int i = 1; i + 1 < n; i++) {
    gfx.fillTriangle(xs[0], ys[0], xs[i], ys[i], xs[i + 1], ys[i + 1], colour);
  }
}

// A horizontal segment: a slab `len` wide and `t` tall, mitred to a point at
// each end so it interlocks with the verticals at the corners the way a
// moulded LED digit does.
static void segH(TFT_eSPI &gfx, int x, int y, int len, int t, uint16_t colour) {
  const int16_t h = (int16_t)(t / 2);
  const int16_t xs[6] = { (int16_t)(x + h), (int16_t)(x + len - h),
                          (int16_t)(x + len), (int16_t)(x + len - h),
                          (int16_t)(x + h), (int16_t)x };
  const int16_t ys[6] = { (int16_t)y, (int16_t)y, (int16_t)(y + h),
                          (int16_t)(y + t), (int16_t)(y + t), (int16_t)(y + h) };
  fillConvexPoly(gfx, xs, ys, 6, colour);
}

// The same slab stood on end.
static void segV(TFT_eSPI &gfx, int x, int y, int len, int t, uint16_t colour) {
  const int16_t h = (int16_t)(t / 2);
  const int16_t xs[6] = { (int16_t)(x + h), (int16_t)(x + t), (int16_t)(x + t),
                          (int16_t)(x + h), (int16_t)x, (int16_t)x };
  const int16_t ys[6] = { (int16_t)y, (int16_t)(y + h), (int16_t)(y + len - h),
                          (int16_t)(y + len), (int16_t)(y + len - h),
                          (int16_t)(y + h) };
  fillConvexPoly(gfx, xs, ys, 6, colour);
}

void sevenSegDigit(TFT_eSPI &gfx, int x, int y, int height, int thickness,
                   uint16_t colour, char glyph) {
  const int w = sevenSegWidth(height);
  const int t = thickness < 1 ? 1 : thickness;

  // The dark hairline between touching segments. A quarter of the stroke is
  // enough to read as a gap at corridor distance without eating the stroke.
  const int g = (t / 4) < 1 ? 1 : (t / 4);

  const int hLen = w - 2 * g;              // horizontal slab length
  const int vLen = (height + t) / 2 - 2 * g;  // vertical slab length
  if (hLen < 2 || vLen < 2) return;

  const int midY = y + (height - t) / 2;
  const uint8_t segs = segmentsFor(glyph);

  if (segs & SEG_A) segH(gfx, x + g, y, hLen, t, colour);
  if (segs & SEG_G) segH(gfx, x + g, midY, hLen, t, colour);
  if (segs & SEG_D) segH(gfx, x + g, y + height - t, hLen, t, colour);
  if (segs & SEG_F) segV(gfx, x, y + g, vLen, t, colour);
  if (segs & SEG_B) segV(gfx, x + w - t, y + g, vLen, t, colour);
  if (segs & SEG_E) segV(gfx, x, midY + g, vLen, t, colour);
  if (segs & SEG_C) segV(gfx, x + w - t, midY + g, vLen, t, colour);
}

int sevenSegStringWidth(const char *text, int height) {
  if (text == NULL || *text == '\0') return 0;
  const int n = (int)strlen(text);
  return n * sevenSegWidth(height) + (n - 1) * sevenSegSpacing(height);
}

int sevenSegString(TFT_eSPI &gfx, int x, int y, int height, int thickness,
                   uint16_t colour, const char *text) {
  if (text == NULL) return 0;
  const int w    = sevenSegWidth(height);
  const int step = w + sevenSegSpacing(height);
  int cursor = x;
  for (const char *p = text; *p; p++) {
    sevenSegDigit(gfx, cursor, y, height, thickness, colour, *p);
    cursor += step;
  }
  return cursor - x - sevenSegSpacing(height);
}

// ===========================================================================
// Floor labels
//
// FLOOR_LABELS arrives as a string literal. It is copied into a writable
// array once and split in place - each comma becomes a NUL and the table holds
// pointers into that array - so there is no allocation and no second copy of
// the text.
// ===========================================================================
static char        gLabelBuf[] = FLOOR_LABELS;
static const char *gLabels[DISPLAY_MAX_LABELS];
static uint8_t     gLabelCount = 0;

static void parseFloorLabels() {
  gLabelCount = 0;
  char *p = gLabelBuf;
  while (*p && gLabelCount < DISPLAY_MAX_LABELS) {
    gLabels[gLabelCount++] = p;
    char *comma = strchr(p, ',');
    if (comma == NULL) break;
    *comma = '\0';
    p = comma + 1;
  }

  // A mistyped list otherwise shows up months later as a display that is
  // silently one floor off at the top of the building, which is exactly the
  // failure this table exists to prevent. Say so on the console instead.
  if (gLabelCount != FLOOR_LABEL_COUNT) {
    Serial.printf("display: FLOOR_LABELS parsed %u labels, "
                  "FLOOR_LABEL_COUNT says %d - check the build flags\n",
                  (unsigned)gLabelCount, (int)FLOOR_LABEL_COUNT);
  }
}

const char *displayFloorLabel(uint8_t floorIndex) {
  if (floorIndex == 0 || floorIndex > gLabelCount) return "--";
  return gLabels[floorIndex - 1];
}

uint8_t displayFloorLabelCount() {
  return gLabelCount;
}

// ===========================================================================
// Panel, sprite and the cache that drives the invalidation rule
// ===========================================================================
static TFT_eSPI    tft;
static TFT_eSprite numberSprite(&tft);
static bool        haveSprite = false;

// ---------------------------------------------------------------------------
// The invalidation rule
// ---------------------------------------------------------------------------
// Nothing is compared against the incoming DisplayUiState. What is cached and
// compared is the *rendered form* of each region - the label string, the
// formatted voltage, the quantised bar length, the formatted distance, the ink
// colour, the arrow direction. A region repaints only when its rendered form
// differs from what is already on the glass.
//
// This matters because the inputs move constantly and the output does not.
// posQ8 arrives every 2 s and jitters by a few 1/256ths even while the car is
// parked; a float comparison would repaint the 190 px number every single
// STATE packet for a change nobody can see. The voltage is the same story at
// 0.1 V resolution. Comparing what will be drawn, rather than what it was
// drawn from, collapses all of that to zero work.
//
// A full-screen clear per frame would be the alternative, and on a 240x320
// ILI9341 at 55 MHz that is ~28 ms of visible black. Region repaints of the
// small text areas are a few hundred microseconds, and the number goes through
// an off-screen sprite so it is replaced in one push with no intermediate
// blank frame.
// ---------------------------------------------------------------------------
struct RenderedForm {
  char     label[DISPLAY_MAX_LABEL_CHARS + 1];
  uint16_t numberInk;
  bool     arrowShown;
  uint8_t  arrowDir;
  uint16_t arrowInk;
  char     volts[10];
  uint16_t voltInk;
  uint8_t  barPct;  // quantised - see barPercent()
  uint16_t barCol;
  uint8_t  barLevel;
  bool     banner;
  char     dist[12];
  uint16_t distInk;
};

static RenderedForm gShown;
static bool         gChromeValid = false;

// ===========================================================================
// Backlight
// ===========================================================================
static const uint8_t BL_CHANNEL  = 0;
static const uint8_t LED_CHANNEL_R = 1;
static const uint8_t LED_CHANNEL_G = 2;
static const uint8_t LED_CHANNEL_B = 3;

static int32_t  gLdrAcc    = -1;  // EMA accumulator, raw << DISPLAY_LDR_FILTER_SHIFT
static uint8_t  gBlTarget  = DISPLAY_BL_MAX;
static uint8_t  gBlDuty    = DISPLAY_BL_MAX;
static int16_t  gBlOverride = -1;
static uint32_t gLdrNextMs = 0;
static uint32_t gRampPrevMs = 0;

// The Arduino-ESP32 LEDC API was rewritten in core 3.x: channels disappeared
// and everything is addressed by pin. Both spellings are kept so this file
// builds against whichever core the platform pulls in.
static void pwmAttach(uint8_t pin, uint8_t channel, uint32_t hz, uint8_t bits) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcAttach(pin, hz, bits);
#else
  ledcSetup(channel, (double)hz, bits);
  ledcAttachPin(pin, channel);
#endif
}

static void pwmWrite(uint8_t pin, uint8_t channel, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

// Maps a filtered LDR reading to the duty the room is asking for.
static uint8_t dutyForLight(int raw) {
#if DISPLAY_LDR_BRIGHT_IS_LOW
  int32_t darkness = raw - DISPLAY_LDR_BRIGHT_RAW;
  int32_t span     = DISPLAY_LDR_DARK_RAW - DISPLAY_LDR_BRIGHT_RAW;
#else
  int32_t darkness = DISPLAY_LDR_BRIGHT_RAW - raw;
  int32_t span     = DISPLAY_LDR_BRIGHT_RAW - DISPLAY_LDR_DARK_RAW;
#endif
  if (span <= 0) return DISPLAY_BL_MAX;
  if (darkness < 0) darkness = 0;
  if (darkness > span) darkness = span;

  const int32_t range = DISPLAY_BL_MAX - DISPLAY_BL_MIN;
  return (uint8_t)(DISPLAY_BL_MAX - (range * darkness) / span);
}

uint8_t displayBacklight() {
  return gBlDuty;
}

void displayBacklightOverride(int duty) {
  if (duty < 0) {
    gBlOverride = -1;
    return;
  }
  if (duty > DISPLAY_BL_MAX) duty = DISPLAY_BL_MAX;
  gBlOverride = (int16_t)duty;
  gBlTarget   = (uint8_t)duty;
}

void displayTick() {
  const uint32_t now = millis();

  if (gBlOverride < 0 && (int32_t)(now - gLdrNextMs) >= 0) {
    gLdrNextMs = now + DISPLAY_LDR_SAMPLE_MS;

    const int raw = analogRead(LDR_PIN);
    if (gLdrAcc < 0) {
      gLdrAcc = (int32_t)raw << DISPLAY_LDR_FILTER_SHIFT;
    } else {
      gLdrAcc += raw - (gLdrAcc >> DISPLAY_LDR_FILTER_SHIFT);
    }

    const uint8_t want = dutyForLight((int)(gLdrAcc >> DISPLAY_LDR_FILTER_SHIFT));

    // Hysteresis, not a comparison: the target only moves when the room has
    // asked for something meaningfully different. Without it a level sitting
    // on a boundary flips the target every sample and the ramp below chases it
    // forever, which is visible as a slow breathing of the panel.
    const int delta = (int)want - (int)gBlTarget;
    if (delta > DISPLAY_BL_HYSTERESIS || delta < -DISPLAY_BL_HYSTERESIS) {
      gBlTarget = want;
    }
  }

  // Ramp: one duty count per DISPLAY_BL_RAMP_MS, so a full-scale change takes
  // ~2 s. Steps missed while the loop was busy are made up, but only up to 32
  // at a time - after a long stall (a mesh burst, an NVS write) catching up in
  // one jump would be exactly the step change the ramp exists to avoid.
  if (gBlDuty != gBlTarget) {
    uint32_t elapsed = now - gRampPrevMs;
    if (elapsed >= DISPLAY_BL_RAMP_MS) {
      uint32_t steps = elapsed / DISPLAY_BL_RAMP_MS;
      if (steps > 32) steps = 32;
      gRampPrevMs = now;

      int diff = (int)gBlTarget - (int)gBlDuty;
      int move = (int)steps;
      if (move > abs(diff)) move = abs(diff);
      gBlDuty = (uint8_t)((int)gBlDuty + (diff > 0 ? move : -move));
      pwmWrite(TFT_BL, BL_CHANNEL, gBlDuty);
    }
  } else {
    gRampPrevMs = now;
  }
}

// ===========================================================================
// RGB LED - ACTIVE LOW
//
// Each channel's pin sits on the cathode side of the LED with the anode at
// 3V3, so a LOW pin sinks current and lights that colour and a HIGH pin puts
// it out. The inversion happens here, once, and every other line in this file
// speaks in "0 = off, 255 = full".
// ===========================================================================
void displayLedRgb(uint8_t r, uint8_t g, uint8_t b) {
  pwmWrite(RGB_LED_R_PIN, LED_CHANNEL_R, 255u - r);
  pwmWrite(RGB_LED_G_PIN, LED_CHANNEL_G, 255u - g);
  pwmWrite(RGB_LED_B_PIN, LED_CHANNEL_B, 255u - b);
}

// ===========================================================================
// Lifecycle
// ===========================================================================

DisplayUiState displayUiStateInit() {
  DisplayUiState s;
  s.floorIndex    = 0;
  s.position      = 0.0f;
  s.positionValid = false;
  s.moving        = false;
  s.direction     = ELEV_DIR_IDLE;
  s.batteryVolts  = 0.0f;
  s.batteryValid  = false;
  s.batteryLevel  = DISPLAY_BATTERY_OK;
  s.dist24hMiles  = 0.0f;
  s.distValid     = false;
  s.stateStale    = false;
  s.statsStale    = false;
  return s;
}

void displayInvalidate() {
  memset(&gShown, 0, sizeof(gShown));
  gShown.label[0] = '\0';
  gChromeValid    = false;
}

bool displayBegin() {
  parseFloorLabels();

  tft.init();
  // Landscape. TFT_WIDTH/TFT_HEIGHT in the build flags are the panel's native
  // portrait dimensions; rotation is a runtime call, which is why the canvas
  // constants in the header are 320x240 and the flags are 240x320.
  tft.setRotation(1);
  tft.fillScreen(COL_BG);

  // TFT_eSPI's init() leaves TFT_BL driven as a plain output. Take it back as
  // a PWM pin before anything tries to dim it.
  pwmAttach(TFT_BL, BL_CHANNEL, DISPLAY_BL_PWM_HZ, DISPLAY_BL_PWM_BITS);
  gBlDuty   = (DISPLAY_BL_MAX + DISPLAY_BL_MIN) / 2;
  gBlTarget = gBlDuty;
  pwmWrite(TFT_BL, BL_CHANNEL, gBlDuty);
  gRampPrevMs = millis();

  // 1 kHz is plenty for an indicator; the eye integrates it and nothing else
  // on this board cares.
  pwmAttach(RGB_LED_R_PIN, LED_CHANNEL_R, 1000, 8);
  pwmAttach(RGB_LED_G_PIN, LED_CHANNEL_G, 1000, 8);
  pwmAttach(RGB_LED_B_PIN, LED_CHANNEL_B, 1000, 8);
  displayLedRgb(0, 0, 0);

  analogReadResolution(12);
  // 11 dB puts full scale near 3.1 V, which is the only attenuation that spans
  // the divider's whole swing from full sun to a dark corridor.
  analogSetPinAttenuation(LDR_PIN, ADC_11db);

  // 8 bits per pixel: 192x196 is 37.6 kB this way and 75 kB at 16, and this
  // chip also has to hold the WiFi stack for ESP-NOW. The number is drawn in
  // three greys and nothing else, all of which survive the 3-3-2 quantisation
  // exactly.
  numberSprite.setColorDepth(8);
  haveSprite = (numberSprite.createSprite(NUM_W, NUM_H) != NULL);
  if (!haveSprite) {
    Serial.println(F("display: number sprite allocation failed - "
                     "falling back to direct draw, expect flicker"));
  }

  displayInvalidate();
  return haveSprite;
}

void displaySplash(const char *line1, const char *line2) {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COL_INK_FRESH, COL_BG);
  if (line1) tft.drawString(line1, DISPLAY_W / 2, DISPLAY_H / 2 - 16, 4);
  tft.setTextColor(COL_INK_DIM, COL_BG);
  if (line2) tft.drawString(line2, DISPLAY_W / 2, DISPLAY_H / 2 + 16, 2);
  displayInvalidate();
}

// ===========================================================================
// Rendering
// ===========================================================================

static uint16_t inkFor(const DisplayUiState &s) {
  if (s.statsStale) return COL_INK_GREY;  // link presumed dead
  if (s.stateStale) return COL_INK_DIM;   // last floor still believed, just old
  return COL_INK_FRESH;
}

// Which label the big number shows. While the car is moving the number tracks
// the live fractional position so it sweeps 3 -> 4 -> 5; the moment a floor is
// confirmed, floorIndex wins. That ordering is the whole point of posQ8 being
// animation-only (spec 4.1): it is never allowed to decide anything.
static void currentLabel(const DisplayUiState &s, char *out, size_t cap) {
  uint8_t idx = s.floorIndex;
  if (s.moving && s.positionValid) {
    const long rounded = lroundf(s.position);
    if (rounded >= 1 && rounded <= (long)gLabelCount) idx = (uint8_t)rounded;
  }
  strncpy(out, displayFloorLabel(idx), cap - 1);
  out[cap - 1] = '\0';
}

// Height that makes the label as large as the pane allows. One digit is
// limited by DISPLAY_DIGIT_MAX_H, two by the width of the left pane - around
// 158 px. Stepping down until it fits is a handful of multiplies and avoids
// re-deriving the aspect algebra every time the ratios change.
static int labelHeight(const char *label) {
  for (int h = DISPLAY_DIGIT_MAX_H; h > 16; h -= 2) {
    if (sevenSegStringWidth(label, h) <= NUM_W && h <= NUM_H) return h;
  }
  return 16;
}

static void paintNumber(const char *label, uint16_t ink) {
  const int h = labelHeight(label);
  const int t = sevenSegThickness(h);
  const int w = sevenSegStringWidth(label, h);

  if (haveSprite) {
    numberSprite.fillSprite(COL_BG);
    sevenSegString(numberSprite, (NUM_W - w) / 2, (NUM_H - h) / 2, h, t, ink, label);
    numberSprite.pushSprite(NUM_X, NUM_Y);
  } else {
    tft.fillRect(NUM_X, NUM_Y, NUM_W, NUM_H, COL_BG);
    sevenSegString(tft, NUM_X + (NUM_W - w) / 2, NUM_Y + (NUM_H - h) / 2, h, t,
                   ink, label);
  }
}

static void paintArrow(bool shown, uint8_t dir, uint16_t ink) {
  tft.fillRect(RP_X, ARROW_Y, RP_W, ARROW_H, COL_BG);
  if (!shown) return;

  const int16_t top = ARROW_Y + 2;
  const char   *text;
  if (dir == ELEV_DIR_DOWN) {
    tft.fillRect(RP_CX - 9, top, 18, 16, ink);
    tft.fillTriangle(RP_CX - 26, top + 16, RP_CX + 26, top + 16, RP_CX, top + 46, ink);
    text = "DOWN";
  } else {
    tft.fillTriangle(RP_CX, top, RP_CX - 26, top + 30, RP_CX + 26, top + 30, ink);
    tft.fillRect(RP_CX - 9, top + 30, 18, 16, ink);
    text = "UP";
  }
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(ink, COL_BG);
  tft.drawString(text, RP_CX, ARROW_Y + ARROW_H - 10, 2);
}

static void paintVolts(const char *text, uint16_t ink) {
  tft.fillRect(RP_X, VOLT_Y, RP_W, VOLT_H, COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(ink, COL_BG);
  tft.drawString(text, RP_CX, VOLT_Y + VOLT_H / 2, 4);
}

// Percentage of the bar to fill, quantised. The bar's interior is 96 px, so
// 4% steps are ~4 px - finer than that is below what the eye resolves on this
// panel and only serves to trigger repaints.
static uint8_t barPercent(float volts, bool valid) {
  if (!valid) return 0;
  const int32_t mv = (int32_t)(volts * 1000.0f + 0.5f);
  const int32_t lo = DISPLAY_VBAT_EMPTY_MV;
  const int32_t hi = DISPLAY_VBAT_FULL_MV;
  int32_t pct = (mv - lo) * 100 / (hi - lo);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t)((pct / 4) * 4);
}

// Colour comes from the transmitter's level, never from the voltage here.
//
// A battery WARNING keeps its colour even once STATS has gone stale, where
// everything else on the screen greys out. The likeliest reason the link dies
// right after a low-battery warning is that the battery ran out - so greying
// the warning at that moment would hide it at exactly the point it came true.
// An OK bar does grey, since "the battery was fine three minutes ago" is not
// worth asserting in green.
static uint16_t barColour(uint8_t level, bool valid, bool statsStale) {
  if (!valid) return COL_INK_GREY;
  if (level == DISPLAY_BATTERY_CRITICAL) return COL_BAR_LOW;
  if (level == DISPLAY_BATTERY_LOW) return COL_BAR_WARN;
  return statsStale ? COL_INK_GREY : COL_BAR_OK;
}

static void paintBar(uint8_t pct, uint16_t colour, uint8_t level) {
  const int16_t x = RP_CX - BAR_W / 2;
  const int16_t y = BAR_Y + 2;
  const int16_t h = BAR_H - 4;

  tft.fillRect(RP_X, BAR_Y, RP_W, BAR_H, COL_BG);

  // Once a warning is up the bar becomes a solid badge that says what to do.
  // A nearly empty outline in orange is a thing you have to interpret; a filled
  // block reading CHARGE SOON is not.
  if (level != DISPLAY_BATTERY_OK) {
    tft.fillRect(x, y, BAR_W, h, colour);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(COL_BG, colour);
    tft.drawString(level == DISPLAY_BATTERY_CRITICAL ? "CHARGE NOW" : "CHARGE SOON",
                   RP_CX, y + h / 2 + 1, 2);
    return;
  }

  tft.drawRect(x, y, BAR_W, h, colour);
  const int16_t inner = BAR_W - 4;
  const int16_t fill  = (int16_t)((int32_t)inner * pct / 100);
  if (fill > 0) tft.fillRect(x + 2, y + 2, fill, h - 4, colour);
}

static void paintDistance(const char *text, uint16_t ink) {
  tft.fillRect(RP_X, DIST_Y, RP_W, DIST_H, COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(ink, COL_BG);
  tft.drawString(text, RP_CX, DIST_Y + 14, 4);
  // "today" rides in the same region rather than being static chrome: it has
  // to fade with the number it labels, or a greyed-out screen keeps one bright
  // word on it.
  tft.drawString("today", RP_CX, DIST_Y + 40, 2);
}

// The bottom strip. Steady rather than flashing: these screens are in public
// corridors, and a steady red band is already unmissable at distance. When it
// comes down it has to put back what it covered - the foot of the divider rule
// and the byline - because paintChrome only runs on a full repaint.
static void paintBanner(bool on) {
  if (on) {
    tft.fillRect(0, BANNER_Y, DISPLAY_W, BANNER_H, COL_BAR_LOW);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(COL_INK_FRESH, COL_BAR_LOW);
    tft.drawString("CHARGE BATTERY", DISPLAY_W / 2, BANNER_Y + BANNER_H / 2 + 1, 4);
    return;
  }
  tft.fillRect(0, BANNER_Y, DISPLAY_W, BANNER_H, COL_BG);
  tft.drawFastVLine(DISPLAY_SPLIT_X, BANNER_Y, (DISPLAY_H - 8) - BANNER_Y, COL_CHROME);
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(COL_CHROME, COL_BG);
  tft.drawString("Made by Nv7", NUM_X + 2, DISPLAY_H - 6, 1);
  tft.setTextDatum(MC_DATUM);
}

static void paintChrome() {
  tft.fillScreen(COL_BG);
  tft.drawFastVLine(DISPLAY_SPLIT_X, 8, DISPLAY_H - 16, COL_CHROME);

  // Byline, bottom left. It sits in chrome grey at font 1 so it reads as part
  // of the bezel rather than as data - from corridor distance it should not
  // compete with the floor number for attention. Drawn here rather than in
  // displayUpdate because it never changes: paintChrome runs once, and the
  // number sprite above it stops at NUM_Y + NUM_H, so nothing repaints over it.
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(COL_CHROME, COL_BG);
  tft.drawString("Made by Nv7", NUM_X + 2, DISPLAY_H - 6, 1);
  tft.setTextDatum(MC_DATUM);

  gChromeValid = true;
}

void displayUpdate(const DisplayUiState &s) {
  if (!gChromeValid) paintChrome();

  const uint16_t ink = inkFor(s);

  // --- the number ---
  char label[DISPLAY_MAX_LABEL_CHARS + 1];
  currentLabel(s, label, sizeof(label));
  if (strcmp(label, gShown.label) != 0 || ink != gShown.numberInk) {
    paintNumber(label, ink);
    strncpy(gShown.label, label, sizeof(gShown.label) - 1);
    gShown.label[sizeof(gShown.label) - 1] = '\0';
    gShown.numberInk = ink;
  }

  // --- direction arrow, only while moving ---
  const bool    arrowShown = s.moving && s.direction != ELEV_DIR_IDLE;
  const uint8_t arrowDir   = arrowShown ? s.direction : (uint8_t)ELEV_DIR_IDLE;
  if (arrowShown != gShown.arrowShown || arrowDir != gShown.arrowDir ||
      ink != gShown.arrowInk) {
    paintArrow(arrowShown, arrowDir, ink);
    gShown.arrowShown = arrowShown;
    gShown.arrowDir   = arrowDir;
    gShown.arrowInk   = ink;
  }

  // --- battery voltage ---
  char volts[10];
  if (s.batteryValid) {
    snprintf(volts, sizeof(volts), "%.1f V", (double)s.batteryVolts);
  } else {
    snprintf(volts, sizeof(volts), "-- V");
  }
  // The voltage takes the warning colour too, and like the badge it does not
  // grey out when the link goes stale - see barColour().
  const uint16_t voltInk =
      (s.batteryValid && s.batteryLevel == DISPLAY_BATTERY_CRITICAL) ? COL_BAR_LOW :
      (s.batteryValid && s.batteryLevel == DISPLAY_BATTERY_LOW)      ? COL_BAR_WARN :
                                                                       ink;
  if (strcmp(volts, gShown.volts) != 0 || voltInk != gShown.voltInk) {
    paintVolts(volts, voltInk);
    strncpy(gShown.volts, volts, sizeof(gShown.volts) - 1);
    gShown.volts[sizeof(gShown.volts) - 1] = '\0';
    gShown.voltInk = voltInk;
  }

  // --- battery bar / badge ---
  const uint8_t  level  = s.batteryValid ? s.batteryLevel : (uint8_t)DISPLAY_BATTERY_OK;
  const uint8_t  pct    = barPercent(s.batteryVolts, s.batteryValid);
  const uint16_t barCol = barColour(level, s.batteryValid, s.statsStale);
  if (pct != gShown.barPct || barCol != gShown.barCol || level != gShown.barLevel) {
    paintBar(pct, barCol, level);
    gShown.barPct   = pct;
    gShown.barCol   = barCol;
    gShown.barLevel = level;
  }

  // --- CHARGE BATTERY banner ---
  const bool banner = (level == DISPLAY_BATTERY_CRITICAL);
  if (banner != gShown.banner) {
    paintBanner(banner);
    gShown.banner = banner;
  }

  // --- 24 h distance ---
  char dist[12];
  if (s.distValid) {
    snprintf(dist, sizeof(dist), "%.1f mi", (double)s.dist24hMiles);
  } else {
    snprintf(dist, sizeof(dist), "-- mi");
  }
  if (strcmp(dist, gShown.dist) != 0 || ink != gShown.distInk) {
    paintDistance(dist, ink);
    strncpy(gShown.dist, dist, sizeof(gShown.dist) - 1);
    gShown.dist[sizeof(gShown.dist) - 1] = '\0';
    gShown.distInk = ink;
  }

  // --- LED ---
  // Off when healthy: ten lit LEDs in a stairwell at night is a complaint
  // waiting to happen, and the screen already says everything. Amber once
  // STATS has been missing for 180 s, per spec section 6, so a dead link is
  // visible from the corridor without reading the screen at all.
  //
  // Steady red for CHARGE BATTERY, and it outranks amber. The two usually
  // arrive together - a flat pack is what silences the transmitter - and red is
  // the one that tells you what to do about it.
  if (banner) {
    displayLedRgb(255, 0, 0);
  } else {
    displayLedRgb(s.statsStale ? 255 : 0, s.statsStale ? 96 : 0, 0);
  }
}
