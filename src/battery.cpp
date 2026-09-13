//
// Battery divider reads. See battery.h for the divider, the thresholds and why
// the 1.67-2.43 V working span was chosen.
//
#include "battery.h"

static bool     adcUp    = false;
static uint16_t lastMv   = 0;

// The ESP32-S3 ADC is not linear across its whole input range. Between roughly
// 0.15 V and 2.8 V at 12 dB it is, and outside that it saturates in a way the
// eFuse calibration cannot undo - which is why the divider was picked to land
// the pack's 10.0-14.6 V inside those bounds rather than to use the full scale.
//
// The core's default attenuation is already 12 dB, but that is a default, not a
// contract: it has changed between core versions and a stray analogSetAttenuation()
// elsewhere would move it globally. Setting it per pin here means this reading
// cannot be silently re-ranged by code that knows nothing about the divider.
//
// ADC_11db is the Arduino name for what the datasheet and the IDF now call
// 12 dB: the nominal figure was corrected to match the measured attenuation,
// the hardware setting did not change. The Arduino enum still spells it 11.
static const adc_attenuation_t kVbatAtten = ADC_11db;

void batteryBegin() {
  analogSetPinAttenuation(VBAT_ADC_PIN, kVbatAtten);

  // The first conversion after a range change reads the previous configuration
  // on some silicon revisions. Throwing it away costs microseconds.
  (void)analogReadMilliVolts(VBAT_ADC_PIN);

  adcUp = true;

  Serial.printf("Battery sense: GPIO%d, %d/%d divider (ratio %.2f), cal %d/%d\n",
                VBAT_ADC_PIN, (int)VBAT_DIVIDER_R1, (int)VBAT_DIVIDER_R2,
                (double)(VBAT_DIVIDER_R1 + VBAT_DIVIDER_R2) / (double)VBAT_DIVIDER_R2,
                (int)VBAT_CAL_NUM, (int)VBAT_CAL_DEN);
}

uint16_t batteryReadMv() {
  if (!adcUp) {
    return 0;
  }

  // analogReadMilliVolts rather than analogRead with a hand-rolled scale: it
  // applies the per-chip eFuse calibration curve, which on this part is worth
  // several percent and is not reproducible from a datasheet constant.
  //
  // Median of VBAT_SAMPLES, not mean. There is no filter capacitor across the
  // divider - the pack voltage moves over hours and this runs once a minute, so
  // the filtering is done here instead. The S3's SAR ADC throws the occasional
  // wild sample, and one of those pulls a 32-sample mean by its full error
  // while leaving the median untouched.
  uint16_t s[VBAT_SAMPLES];
  for (int i = 0; i < VBAT_SAMPLES; i++) {
    s[i] = (uint16_t)analogReadMilliVolts(VBAT_ADC_PIN);
  }
  // Insertion sort: VBAT_SAMPLES is 32, so this is a few hundred cycles once a
  // minute and not worth anything cleverer.
  for (int i = 1; i < VBAT_SAMPLES; i++) {
    uint16_t v = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > v) {
      s[j + 1] = s[j];
      j--;
    }
    s[j + 1] = v;
  }
  // Even count, so average the two middle samples rather than picking one.
  uint32_t pinMv = ((uint32_t)s[VBAT_SAMPLES / 2 - 1] +
                    (uint32_t)s[VBAT_SAMPLES / 2]) / 2u;

  // 64-bit because the intermediates are not small: 2433 mV x 120 kOhm is
  // 2.9e8, and a DMM calibration numerator like 13280 then takes the pack
  // figure past a uint32. Headroom for any divider values, too.
  uint64_t packMv = (uint64_t)pinMv *
                    (uint64_t)(VBAT_DIVIDER_R1 + VBAT_DIVIDER_R2) /
                    (uint64_t)VBAT_DIVIDER_R2;
  packMv = packMv * (uint64_t)VBAT_CAL_NUM / (uint64_t)VBAT_CAL_DEN;

  // The STATS field is a u16, and a 4S pack cannot reach 65 V. Clamping rather
  // than wrapping means a shorted divider reads absurdly high instead of
  // absurdly plausible.
  if (packMv > 0xFFFF) {
    packMv = 0xFFFF;
  }

  lastMv = (uint16_t)packMv;
  return lastMv;
}

uint16_t batteryLastMv() {
  return lastMv;
}

// ---------------------------------------------------------------------------
// Latched level
// ---------------------------------------------------------------------------
static BatteryLevel gLevel = BATTERY_OK;

const char *batteryLevelName(BatteryLevel level) {
  switch (level) {
    case BATTERY_OK:       return "ok";
    case BATTERY_LOW:      return "LOW";
    case BATTERY_CRITICAL: return "CRITICAL";
  }
  return "?";
}

BatteryLevel batteryLevelUpdate(uint16_t mv) {
  if (mv == 0) return gLevel;

  if (mv >= VBAT_CHARGED_MV) {
    gLevel = BATTERY_OK;
  } else if (mv < VBAT_CRITICAL_MV) {
    gLevel = BATTERY_CRITICAL;
  } else if (mv < VBAT_WARN_MV && gLevel == BATTERY_OK) {
    gLevel = BATTERY_LOW;
  }
  // Between the thresholds and VBAT_CHARGED_MV the level holds wherever it is:
  // that band is the hysteresis.
  return gLevel;
}

BatteryLevel batteryLevel() { return gLevel; }
