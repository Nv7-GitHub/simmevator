#pragma once
//
// Pack voltage for the 4S LiFePO4 in the car, read through a resistive divider
// on the XIAO ESP32S3's D0 pad.
//
// Spec section 3.2. The divider is 100 kOhm over 20 kOhm, ratio 6.0, so the
// 10.0-14.6 V the pack can present arrives at the pin as 1.667-2.433 V. That
// span is the whole reason for that ratio: the S3's ADC is only
// linear over roughly 0.15-2.8 V at 12 dB attenuation, and the divider places
// both ends of the pack's range comfortably inside it with margin at the top
// for a charger that overshoots. The resistances are kilohms rather than
// megohms because there is no filter cap: the ADC's sample capacitor charges
// through the divider's ~16.7 kOhm Thevenin resistance, not ~167 kOhm, so each
// sample settles instead of reading low.
//
// Two calls, matching bmp390_sensor.h: batteryBegin() once from setup(), then
// batteryReadMv() as often as wanted - in practice once per STATS packet, once
// a minute. There is no switching MOSFET across the divider because it draws
// ~122 uA at 14.6 V, ~0.5% of the ~23 mA average the power budget is built on.
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
#define VBAT_DIVIDER_R1 100000
#endif
#ifndef VBAT_DIVIDER_R2
#define VBAT_DIVIDER_R2 20000
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
// and then falls away, so the useful thresholds are at the knee rather than
// anywhere in the middle of the pack's life.
//
// The notice each one gives is what sets it, and it is short. At the ~23 mA the
// budget assumes, 10 Ah is ~435 h, so every 10% of the pack is ~1.8 days. An
// earlier version warned at 12.0 V believing it was ~20% left; it is nearer 10%,
// which is under two days - a weekend away and the car goes dark unwarned.

// ~20% remaining, ~3.5 days of notice. The screens show CHARGE SOON. Sent as
// ELEV_FLAG_LOW_BATTERY.
#ifndef VBAT_WARN_MV
#define VBAT_WARN_MV 12800
#endif

// ~10% remaining, under two days. The screens show a CHARGE BATTERY banner.
// Sent as ELEV_FLAG_CRITICAL_BATTERY. The BMS cuts off around 10 V below this.
#ifndef VBAT_CRITICAL_MV
#define VBAT_CRITICAL_MV 12000
#endif

// A warning clears only above this - rested-full for 4S LiFePO4 - and never on
// the way down. Voltage wobbles by tens of millivolts with temperature and
// load, so a plain comparison against 12.8 V would flicker in and out for days
// around the threshold, and a warning that has disappeared by itself when you
// happen to walk past is the easiest kind to miss. Discharge does not reverse
// without a charger, so nothing short of a charged pack should clear it.
#ifndef VBAT_CHARGED_MV
#define VBAT_CHARGED_MV 13300
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

enum BatteryLevel {
  BATTERY_OK = 0,
  BATTERY_LOW,       // below VBAT_WARN_MV at some point since the last charge
  BATTERY_CRITICAL   // below VBAT_CRITICAL_MV at some point since the last charge
};

const char *batteryLevelName(BatteryLevel level);

// Folds one reading into the latched level and returns it. Escalates the moment
// a reading crosses a threshold; returns to BATTERY_OK only once a reading is at
// or above VBAT_CHARGED_MV. A reading of 0 (ADC not up) leaves the level alone.
//
// The latch is in RAM, so a reboot re-derives the level from its first reading.
// That is deliberate: the usual reason this node reboots is that the pack was
// taken off to charge, and a charged pack should come back clear.
BatteryLevel batteryLevelUpdate(uint16_t mv);

// The latched level, without taking a reading.
BatteryLevel batteryLevel();
