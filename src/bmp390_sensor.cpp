//
// BMP390 bring-up and reads over Wire1. See bmp390_sensor.h for the wiring and
// the reasoning behind the oversampling / ODR choices.
//
#include "bmp390_sensor.h"

#include <Adafruit_BMP3XX.h>
#include <Wire.h>

static Adafruit_BMP3XX bmp;
static bool    sensorUp   = false;
static uint8_t sensorAddr = 0;

// The two addresses a BMP390 can present, depending on how SDO is strapped.
static const uint8_t kCandidateAddrs[] = { BMP390_I2C_ADDR,
                                           (BMP390_I2C_ADDR == 0x77) ? 0x76 : 0x77 };

// Pokes one address and returns the raw Wire status code:
//   0 = a device acknowledged
//   2 = clean NACK - the bus is working, there is simply nobody at this address
//   anything else = bus-level failure (stuck line, no pull-ups, arbitration
//                   loss). This distinction is the useful one: a bus with no
//                   pull-ups fails every probe, which looks nothing like an
//                   empty-but-healthy bus even though both find zero devices.
static uint8_t probeAddress(uint8_t addr) {
  Wire1.beginTransmission(addr);
  return Wire1.endTransmission();
}

// Pokes one address and reports whether anything acknowledged.
static bool addressResponds(uint8_t addr) {
  return probeAddress(addr) == 0;
}

uint8_t bmp390Address() {
  return sensorAddr;
}

bool bmp390BringUp(const char *role) {
  Serial.println();
  Serial.println(F("==========================================="));
  Serial.print(F("BMP390 bring-up - "));
  Serial.println(role);
  Serial.println(F("==========================================="));

  Serial.printf("I2C bus 1: SDA=GPIO%d SCL=GPIO%d @ %lu Hz\n",
                BMP390_SDA_PIN, BMP390_SCL_PIN, (unsigned long)BMP390_I2C_HZ);

  if (!Wire1.begin(BMP390_SDA_PIN, BMP390_SCL_PIN, BMP390_I2C_HZ)) {
    Serial.println(F("Wire1.begin() FAILED - check the pin numbers above"));
    return false;
  }

  // Scan the whole bus before touching the driver. If the BMP390 is missing
  // this says whether *anything* is out there, which separates "wrong address"
  // from "nothing wired / no pull-ups" without reaching for a scope.
  //
  // The full 7-bit space is probed, not just 0x08-0x77. A BMP390 cannot live
  // in the reserved ranges, but a device that ACKs there is worth knowing
  // about: it means something on the bus is misbehaving, and an ACK anywhere
  // at all proves the wiring and pull-ups are good.
  Serial.println(F("Scanning bus 1, full 7-bit range 0x00-0x7F:"));
  uint8_t  found  = 0;
  uint16_t nacks  = 0;   // clean "nobody home"
  uint16_t faults = 0;   // bus never completed the transaction
  uint8_t  firstFaultCode = 0;

  for (uint8_t addr = 0x00; addr <= 0x7F; addr++) {
    uint8_t rc = probeAddress(addr);
    if (rc == 0) {
      bool reserved = (addr <= 0x07) || (addr >= 0x78);
      Serial.printf("   0x%02X  ACK%s\n", addr,
                    reserved ? "   <- reserved address, not a normal device" : "");
      found++;
    } else if (rc == 2) {
      nacks++;
    } else {
      if (faults == 0) {
        firstFaultCode = rc;
      }
      faults++;
    }
  }

  Serial.printf("   %u device%s found, %u clean NACK%s, %u bus fault%s\n",
                found,  found  == 1 ? "" : "s",
                nacks,  nacks  == 1 ? "" : "s",
                faults, faults == 1 ? "" : "s");

  // Turn those counts into the conclusion they imply, so the console says what
  // to go check rather than leaving three numbers to interpret.
  if (faults > 0) {
    Serial.printf("   Bus-level failures (first Wire code %u): SDA or SCL is not\n",
                  firstFaultCode);
    Serial.println(F("   returning high. Check the pull-ups and that neither line"));
    Serial.println(F("   is shorted to ground or to the other."));
  } else if (found == 0) {
    Serial.println(F("   Every address NACKed cleanly, so the bus is electrically"));
    Serial.println(F("   healthy - pull-ups are present and both lines toggle."));
    Serial.println(F("   Nothing is listening: check the sensor's 3V3 and GND, and"));
    Serial.println(F("   that SDA/SCL go to the pins named above."));
  }

  // Try the configured address first, then the alternate strap. Reporting the
  // fallback rather than hiding it means a board strapped the other way works
  // now and you still know to fix BMP390_I2C_ADDR.
  for (uint8_t i = 0; i < sizeof(kCandidateAddrs); i++) {
    uint8_t addr = kCandidateAddrs[i];
    if (!addressResponds(addr)) {
      continue;
    }
    if (!bmp.begin_I2C(addr, &Wire1)) {
      Serial.printf("0x%02X acknowledged but is not a BMP390 (chip ID mismatch)\n", addr);
      continue;
    }
    sensorAddr = addr;
    sensorUp   = true;
    if (addr != BMP390_I2C_ADDR) {
      Serial.printf("NOTE: found at 0x%02X, not the configured 0x%02X - SDO is strapped\n"
                    "      the other way. Set -DBMP390_I2C_ADDR=0x%02X to match.\n",
                    addr, BMP390_I2C_ADDR, addr);
    }
    break;
  }

  if (!sensorUp) {
    Serial.printf("BMP390 NOT FOUND at 0x%02X or 0x%02X.\n",
                  kCandidateAddrs[0], kCandidateAddrs[1]);
    Serial.println(F("Check SDA/SCL, 3V3, GND, and that the bus has pull-ups."));
    return false;
  }

  Serial.printf("BMP390 up at 0x%02X\n", sensorAddr);

  // Each of these validates against the others inside the Bosch driver, so a
  // rejected setting means the combination is unachievable, not a bus fault.
  if (!bmp.setPressureOversampling(BMP390_PRESSURE_OVERSAMPLING) ||
      !bmp.setTemperatureOversampling(BMP390_TEMPERATURE_OVERSAMPLING) ||
      !bmp.setIIRFilterCoeff(BMP390_IIR_COEFF) ||
      !bmp.setOutputDataRate(BMP390_ODR)) {
    Serial.println(F("Sensor configuration REJECTED - the oversampling and ODR"));
    Serial.println(F("combination is not achievable. Lower the ODR or the"));
    Serial.println(F("oversampling (see bmp390_sensor.h)."));
    sensorUp = false;
    return false;
  }

  Serial.println(F("Config: pressure x8, temperature x2, IIR coeff 3, ODR 25 Hz"));
  Serial.printf("Altitude reference: %.2f hPa\n", (double)BMP390_SEA_LEVEL_HPA);

  // The first conversion after configuration runs with an unfilled IIR filter,
  // so throw it away rather than reporting a settling value as a measurement.
  bmp.performReading();

  Serial.println(F("-------------------------------------------"));
  return true;
}

Bmp390Reading bmp390Read() {
  Bmp390Reading r = { false, 0.0f, 0.0f, 0.0f };

  if (!sensorUp || !bmp.performReading()) {
    return r;
  }

  r.valid        = true;
  r.pressurePa   = bmp.pressure;
  r.temperatureC = bmp.temperature;

  // Barometric formula, same one Adafruit_BMP3XX::readAltitude uses. Computed
  // here from the reading we already have rather than calling readAltitude(),
  // which would trigger a second conversion for the same sample.
  float ratio = (r.pressurePa / 100.0f) / BMP390_SEA_LEVEL_HPA;
  r.altitudeM = 44330.0f * (1.0f - powf(ratio, 1.0f / 5.255f));

  return r;
}
