//
// NVS persistence for the learned building model. See nvs_model.h for the
// throttle, the record checks and why a failing record is discarded whole.
//
#include "nvs_model.h"

#include <Preferences.h>

#include "crc16.h"

// The stored record: a small header, then FloorModelState verbatim.
//
// The CRC covers the payload only. FloorModelState is memset to zero by
// FloorMonitor::save() before any field is written, so the padding the
// compiler inserts around its doubles is deterministic and a byte-wise CRC
// over the struct is reproducible - which it would not be if the padding held
// whatever was on the stack.
//
// payloadLen is stored rather than assumed: it is what catches a record
// written by a build whose FloorModelState grew without FLOOR_STATE_SCHEMA
// being bumped. The schema check alone would let that through and restore()
// would read past the end of a shorter record.
struct NvsModelRecord {
  uint16_t magic;
  uint16_t schemaVersion;
  uint16_t payloadLen;
  uint16_t crc;
  FloorModelState state;
};

static Preferences prefs;
static bool            nvsUp        = false;
static bool            restored     = false;
static bool            dirty        = false;
static uint32_t        lastSaveMs   = 0;
static uint32_t        writeCount   = 0;
static NvsModelStatus  lastStatus   = NVS_MODEL_EMPTY;

static uint16_t payloadCrc(const FloorModelState *s) {
  return crc16Ccitt((const uint8_t *)s, sizeof(*s));
}

const char *nvsModelStatusName(NvsModelStatus s) {
  switch (s) {
    case NVS_MODEL_OK:          return "ok";
    case NVS_MODEL_EMPTY:       return "empty";
    case NVS_MODEL_SKIPPED:     return "skipped";
    case NVS_MODEL_BAD_MAGIC:   return "bad magic";
    case NVS_MODEL_BAD_SIZE:    return "bad size";
    case NVS_MODEL_BAD_SCHEMA:  return "bad schema";
    case NVS_MODEL_BAD_CRC:     return "bad crc";
    case NVS_MODEL_ERR_OPEN:    return "nvs open failed";
    case NVS_MODEL_ERR_WRITE:   return "nvs write failed";
  }
  return "unknown";
}

bool nvsModelBegin() {
  if (nvsUp) {
    return true;
  }

  if (!prefs.begin(NVS_MODEL_NAMESPACE, false)) {
    // Not recoverable from here: either the nvs partition is missing from the
    // partition table or the flash is failing. The node still tracks floors,
    // it just forgets them, so this is reported and execution continues.
    lastStatus = NVS_MODEL_ERR_OPEN;
    Serial.printf("NVS: namespace \"%s\" would not open - the model will not "
                  "persist this run\n", NVS_MODEL_NAMESPACE);
    return false;
  }

  nvsUp = true;
  lastSaveMs = millis();
  Serial.printf("NVS: \"%s\"/\"%s\", record %u bytes, save every %lu ms when dirty\n",
                NVS_MODEL_NAMESPACE, NVS_MODEL_KEY,
                (unsigned)sizeof(NvsModelRecord),
                (unsigned long)NVS_SAVE_INTERVAL_MS);
  return true;
}

NvsModelStatus nvsModelLoad(FloorMonitor *fm) {
  if (!nvsUp || fm == NULL) {
    lastStatus = NVS_MODEL_ERR_OPEN;
    return lastStatus;
  }

  size_t stored = prefs.getBytesLength(NVS_MODEL_KEY);
  if (stored == 0) {
    lastStatus = NVS_MODEL_EMPTY;
    Serial.println(F("NVS: no stored model - cold boot, the pitch will be learned"));
    return lastStatus;
  }
  if (stored != sizeof(NvsModelRecord)) {
    lastStatus = NVS_MODEL_BAD_SIZE;
    Serial.printf("NVS: stored record is %u bytes, this build expects %u - discarded\n",
                  (unsigned)stored, (unsigned)sizeof(NvsModelRecord));
    return lastStatus;
  }

  NvsModelRecord rec;
  size_t got = prefs.getBytes(NVS_MODEL_KEY, &rec, sizeof(rec));
  if (got != sizeof(rec)) {
    lastStatus = NVS_MODEL_BAD_SIZE;
    Serial.printf("NVS: short read, %u of %u bytes - discarded\n",
                  (unsigned)got, (unsigned)sizeof(rec));
    return lastStatus;
  }

  if (rec.magic != NVS_MODEL_MAGIC) {
    lastStatus = NVS_MODEL_BAD_MAGIC;
    Serial.printf("NVS: magic 0x%04X, expected 0x%04X - not our record, discarded\n",
                  rec.magic, (unsigned)NVS_MODEL_MAGIC);
    return lastStatus;
  }
  if (rec.payloadLen != (uint16_t)sizeof(FloorModelState) ||
      rec.schemaVersion != (uint16_t)FLOOR_STATE_SCHEMA) {
    lastStatus = NVS_MODEL_BAD_SCHEMA;
    Serial.printf("NVS: record is schema %u/%u bytes, this build is %u/%u - discarded\n",
                  rec.schemaVersion, rec.payloadLen,
                  (unsigned)FLOOR_STATE_SCHEMA, (unsigned)sizeof(FloorModelState));
    return lastStatus;
  }

  uint16_t crc = payloadCrc(&rec.state);
  if (crc != rec.crc) {
    lastStatus = NVS_MODEL_BAD_CRC;
    Serial.printf("NVS: CRC 0x%04X, stored 0x%04X - corrupted, discarded\n",
                  crc, rec.crc);
    return lastStatus;
  }

  // restore() runs its own schema check and is the only thing that touches the
  // monitor. It returning false after the header already agreed would mean the
  // header and the payload disagree about the schema, which is a corrupted
  // record the CRC happened not to catch.
  if (!fm->restore(&rec.state)) {
    lastStatus = NVS_MODEL_BAD_SCHEMA;
    Serial.println(F("NVS: FloorMonitor rejected the payload - discarded"));
    return lastStatus;
  }

  restored = true;
  dirty = false;
  lastStatus = NVS_MODEL_OK;
  Serial.printf("NVS: model restored - pitch %.3f m, %d floors, %lu trips, "
                "%lu stops, %.0f m\n",
                fm->pitch(), fm->nFloors(),
                (unsigned long)fm->trips(), (unsigned long)fm->stops(),
                fm->distanceM());
  return lastStatus;
}

NvsModelStatus nvsModelSave(const FloorMonitor *fm) {
  if (!nvsUp || fm == NULL) {
    lastStatus = NVS_MODEL_ERR_OPEN;
    return lastStatus;
  }

  NvsModelRecord rec;
  memset(&rec, 0, sizeof(rec));
  fm->save(&rec.state);
  rec.magic = NVS_MODEL_MAGIC;
  rec.schemaVersion = (uint16_t)FLOOR_STATE_SCHEMA;
  rec.payloadLen = (uint16_t)sizeof(FloorModelState);
  rec.crc = payloadCrc(&rec.state);

  size_t written = prefs.putBytes(NVS_MODEL_KEY, &rec, sizeof(rec));
  if (written != sizeof(rec)) {
    // Preferences returns 0 on failure and cannot say why. The useful signal
    // is that it happened at all - a healthy partition does not short-write -
    // so the throttle clock is deliberately not advanced and the next tick
    // tries again rather than waiting out another interval.
    lastStatus = NVS_MODEL_ERR_WRITE;
    Serial.printf("NVS: write stored %u of %u bytes - the model is not persisted\n",
                  (unsigned)written, (unsigned)sizeof(rec));
    return lastStatus;
  }

  dirty = false;
  lastSaveMs = millis();
  writeCount++;
  lastStatus = NVS_MODEL_OK;
  return lastStatus;
}

void nvsModelMarkDirty() {
  dirty = true;
}

NvsModelStatus nvsModelMaybeSave(uint32_t nowMs, const FloorMonitor *fm) {
  if (!nvsUp) {
    return NVS_MODEL_ERR_OPEN;
  }
  if (!dirty) {
    return NVS_MODEL_SKIPPED;
  }
  // Unsigned difference, so the millis() wrap at 49.7 days costs one late save
  // rather than an arithmetic sign error that would stall saves forever.
  if ((uint32_t)(nowMs - lastSaveMs) < (uint32_t)NVS_SAVE_INTERVAL_MS) {
    return NVS_MODEL_SKIPPED;
  }
  return nvsModelSave(fm);
}

bool nvsModelRestored() {
  return restored;
}

uint32_t nvsModelWriteCount() {
  return writeCount;
}

NvsModelStatus nvsModelLastStatus() {
  return lastStatus;
}

NvsModelStatus nvsModelErase() {
  if (!nvsUp) {
    lastStatus = NVS_MODEL_ERR_OPEN;
    return lastStatus;
  }
  // remove() reports false when the key was not there, which is the same end
  // state the caller asked for.
  prefs.remove(NVS_MODEL_KEY);
  restored = false;
  dirty = false;
  lastStatus = NVS_MODEL_OK;
  Serial.println(F("NVS: stored model erased - next boot is a cold boot"));
  return lastStatus;
}
