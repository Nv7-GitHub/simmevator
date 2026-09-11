#pragma once
//
// BMP390 barometric pressure / temperature sensor on the XIAO ESP32S3's
// second I2C peripheral (Wire1), wired to the D4/D5 pads.
//
// Ported unchanged from elevatormons - this is the module the 3 h reference
// capture was taken with, so the preset below is the one every constant in
// floor_monitor.h was tuned against. Bus 1 is used rather than the default
// Wire so that the sensor can never disturb anything sitting on bus 0.
//
// Everything the sensor needs is behind two calls - bmp390BringUp() once from
// setup(), then bmp390Read() as often as you like - so a consumer never has to
// know which driver is underneath or which bus it is on.
//

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------

// The XIAO ESP32S3's standard I2C pads: D4 = GPIO5 (SDA), D5 = GPIO6 (SCL).
// These are free on the car node - the Wio-SX1262 talks over SPI on D8/D9/D10
// and does not touch them.
#ifndef BMP390_SDA_PIN
#define BMP390_SDA_PIN 5
#endif
#ifndef BMP390_SCL_PIN
#define BMP390_SCL_PIN 6
#endif

// 400 kHz fast mode. The BMP390 handles it, and at 1 Hz sampling the bus is
// idle almost all the time anyway - this just shortens each transaction.
#ifndef BMP390_I2C_HZ
#define BMP390_I2C_HZ 400000
#endif

// 0x77 with SDO high (the usual breakout default), 0x76 with SDO pulled low.
// bmp390BringUp() scans the bus and tries the other address if this one is
// silent, so a board strapped the other way still comes up - it just says so.
#ifndef BMP390_I2C_ADDR
#define BMP390_I2C_ADDR 0x77
#endif

// ---------------------------------------------------------------------------
// Sensor configuration
// ---------------------------------------------------------------------------
// Bosch's "indoor navigation" preset: x8 pressure oversampling, x2 temperature,
// IIR coefficient 3. That lands around 5 cm of altitude noise, which is the
// point of using a BMP390 rather than a BMP280 in a lift shaft.
//
// The output data rate has to be slow enough for the conversion to finish. At
// x8/x2 a measurement takes ~21 ms (Bosch's formula: 234 + 392 + 8*2020 + 163 +
// 2*2020 us), so 50 Hz - a 20 ms period - is rejected by the driver's own
// oversampling/ODR validation. 25 Hz is the fastest rate this preset allows.
#ifndef BMP390_PRESSURE_OVERSAMPLING
#define BMP390_PRESSURE_OVERSAMPLING BMP3_OVERSAMPLING_8X
#endif
#ifndef BMP390_TEMPERATURE_OVERSAMPLING
#define BMP390_TEMPERATURE_OVERSAMPLING BMP3_OVERSAMPLING_2X
#endif
#ifndef BMP390_IIR_COEFF
#define BMP390_IIR_COEFF BMP3_IIR_FILTER_COEFF_3
#endif
#ifndef BMP390_ODR
#define BMP390_ODR BMP3_ODR_25_HZ
#endif

// Reference pressure for the altitude conversion, in hPa. 1013.25 is the
// standard atmosphere; altitude computed against it is only meaningful as a
// *relative* number unless you set this to the local sea-level pressure. The
// floor algorithm only ever looks at differences, which is exactly why the
// 9.9 m of reference drift over the 3 h capture does not move a floor number.
#ifndef BMP390_SEA_LEVEL_HPA
#define BMP390_SEA_LEVEL_HPA 1013.25f
#endif

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

struct Bmp390Reading {
  bool  valid;         // false if the conversion failed; other fields are stale
  float pressurePa;    // absolute pressure, pascals
  float temperatureC;  // degrees celsius
  float altitudeM;     // barometric altitude against BMP390_SEA_LEVEL_HPA
};

// Brings up Wire1, scans it, opens the sensor and applies the configuration
// above. Prints what it finds at every step - which addresses answered, which
// one the driver attached to, and the chip revision - so a miswire or a
// wrong-address board is one line of console rather than silent zeroes.
// Returns false if the sensor never came up; the caller decides what to do.
bool bmp390BringUp(const char *role);

// Takes one reading. Returns a reading with valid == false if the sensor is
// not up or the conversion failed.
Bmp390Reading bmp390Read();

// Address the driver actually attached to, or 0 if bring-up failed.
uint8_t bmp390Address();
