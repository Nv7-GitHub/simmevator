#pragma once
//
// Pack voltage for the 4S LiFePO4 in the car, read through a resistive divider
// on the XIAO ESP32S3's D0 pad.
//
// Spec section 3.2. The divider is 1 MOhm over 200 kOhm, ratio 6.0, so the
// 10.0-14.6 V the pack can present arrives at the pin as 1.667-2.433 V. That
// span is the whole reason for those two resistances: the S3's ADC is only
// linear over roughly 0.15-2.8 V at 12 dB attenuation, and the divider places
// both ends of the pack's range comfortably inside it with margin at the top
// for a charger that overshoots.
//
// Two calls, matching bmp390_sensor.h: batteryBegin() once from setup(), then
// batteryReadMv() as often as wanted - in practice once per STATS packet, once
// a minute. There is no switching MOSFET across the divider because it draws
// 12 uA, 0.06% of the ~19 mA average the power budget is built on.
//
// Accuracy is +-2-3% uncalibrated, about +-0.4 V at 12 V. That answers "does
// this need charging?" and is not a fuel gauge - see spec section 9.
//

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Wiring and calibration - all set from platformio.ini
// ---------------------------------------------------------------------------

// D0 = GPIO1 = ADC1_CH0, the only free ADC1 pad on this board: D4/D5 are the
// I2C bus, D8/D9/D10 are the radio's SPI, and GPIO39-42 are the radio's
// control lines. ADC1 matters rather than just "an ADC" - ADC2 is unusable
// while WiFi is active, which on the bridge and the displays it is.
#ifndef VBAT_ADC_PIN
#define VBAT_ADC_PIN 1
#endif

// The divider as it appears on the schematic, so the ratio below reads like
// the board rather than like a magic 6.0.
#ifndef VBAT_DIVIDER_R1
#define VBAT_DIVIDER_R1 1000000
#endif
#ifndef VBAT_DIVIDER_R2
#define VBAT_DIVIDER_R2 200000
#endif

// One-point correction against a DMM, set per unit after assembly. 1/1 leaves
// the reading as the eFuse calibration and the nominal resistances give it.
#ifndef VBAT_CAL_NUM
#define VBAT_CAL_NUM 1
#endif
#ifndef VBAT_CAL_DEN
#define VBAT_CAL_DEN 1
#endif

// Samples per reading, reduced by MEDIAN rather than mean - see batteryReadMv.
// There is no filter capacitor across the divider, so this is the whole filter.
// 32 at the S3's ~100 ksps is under a millisecond, and it buries both the couple
// of LSBs of noise the ADC shows on a static input and the occasional wild SAR
// sample, well below the +-2-3% the divider tolerance already costs.
#ifndef VBAT_SAMPLES
#define VBAT_SAMPLES 32
#endif

// ---------------------------------------------------------------------------
// LiFePO4 4S thresholds - spec section 3.2
// ---------------------------------------------------------------------------
// A LiFePO4 discharge curve is almost flat from 13.3 V down to about 12.8 V
// and then falls away, so these two points are far more informative than the
// voltage anywhere in the middle of the pack's life.

// ~20% remaining. This is the bit the STATS packet carries as
// ELEV_FLAG_LOW_BATTERY.
#ifndef VBAT_WARN_MV
#define VBAT_WARN_MV 12000
#endif

// Below this the BMS is close to cutting off; the pack is done.
#ifndef VBAT_CRITICAL_MV
#define VBAT_CRITICAL_MV 11200
#endif

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Sets the attenuation on VBAT_ADC_PIN and takes one throwaway reading. Must
// run before batteryReadMv(); calling it twice is harmless.
void batteryBegin();

// Pack voltage in millivolts, the median of VBAT_SAMPLES readings scaled back
// through the divider and the calibration ratio. Returns 0 if batteryBegin()
// has not run - zero is not a voltage a connected pack can produce, so a
// caller that ignores this still shows something obviously wrong rather than
// a plausible number.
uint16_t batteryReadMv();

// Last value batteryReadMv() produced, without touching the ADC. For the code
// paths that want the number they already paid for.
uint16_t batteryLastMv();

// Warn threshold, not critical: this is what goes on the air, and a display
// that greys out at 12.0 V still has hours of runtime behind it.
static inline bool batteryIsLow(uint16_t mv) {
  return mv != 0 && mv < VBAT_WARN_MV;
}

static inline bool batteryIsCritical(uint16_t mv) {
  return mv != 0 && mv < VBAT_CRITICAL_MV;
}
