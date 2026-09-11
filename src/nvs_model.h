#pragma once
//
// The learned building model, kept across reboots and battery swaps.
//
// Spec section 5.1. FloorMonitor already exposes the whole persistent model as
// one pointer-free struct - FloorModelState, with save()/restore() either side
// of it - so this file adds only what NVS needs on top: a namespace, a record
// header with a schema version and a CRC, and the write throttle.
//
// ---------------------------------------------------------------------------
// Why the throttle exists
// ---------------------------------------------------------------------------
// Saving on every confirmed stop would be about 2000 writes a day - the 3 h
// reference capture holds 256 confirmed stops. Saving every
// NVS_SAVE_INTERVAL_MS *when something has changed* is 288 a day. This node is
// expected to run for years on a flash part with a finite erase count, so the
// difference is not an optimisation.
//
// What the throttle costs is up to one interval of newly learned model on a
// hard reset. That is a few landings' worth of running means against a ladder
// that took hours to build, so the restored model is still worth far more than
// the cold boot it replaces.
//
// ---------------------------------------------------------------------------
// Why a record that fails a check is discarded whole
// ---------------------------------------------------------------------------
// A partially applied model is worse than no model: a pitch without its height
// ladder, or a ladder with a stale ref, produces confident wrong floors rather
// than the withheld output a genuine cold boot gives. So a bad magic, a wrong
// schema, a short read or a CRC mismatch all lead to exactly one outcome - the
// FloorMonitor is left untouched and the caller is told it is a cold boot.
//

#include <Arduino.h>

#include "floor_monitor.h"

// ---------------------------------------------------------------------------
// Where the record lives
// ---------------------------------------------------------------------------
// Preferences namespaces are limited to 15 characters, so this is not the
// project name spelled out.
#ifndef NVS_MODEL_NAMESPACE
#define NVS_MODEL_NAMESPACE "simmevator"
#endif

#ifndef NVS_MODEL_KEY
#define NVS_MODEL_KEY "model"
#endif

// Distinguishes "this key holds our record" from "this key holds whatever the
// previous firmware put here", before any of the bytes are believed.
#define NVS_MODEL_MAGIC 0x5E1F

// 5 minutes when something changed: 288 writes/day - see the note above.
#ifndef NVS_SAVE_INTERVAL_MS
#define NVS_SAVE_INTERVAL_MS 300000
#endif

// ---------------------------------------------------------------------------
// Outcomes
// ---------------------------------------------------------------------------
// Every one of these is reported rather than folded into a bool. A CRC
// mismatch means a corrupted record and is worth investigating; a missing key
// on a fresh unit is normal; a failed write means the flash or the partition
// table is wrong and the node will silently stop persisting if nobody says so.
enum NvsModelStatus {
  NVS_MODEL_OK = 0,
  NVS_MODEL_EMPTY,        // no record stored yet - a genuinely fresh unit
  NVS_MODEL_SKIPPED,      // nothing changed, or the throttle interval has not elapsed
  NVS_MODEL_BAD_MAGIC,    // the key holds something that is not one of our records
  NVS_MODEL_BAD_SIZE,     // stored length disagrees with the record this build expects
  NVS_MODEL_BAD_SCHEMA,   // a record from a different FLOOR_STATE_SCHEMA
  NVS_MODEL_BAD_CRC,      // right shape, corrupted contents
  NVS_MODEL_ERR_OPEN,     // the namespace would not open - NVS itself is unhealthy
  NVS_MODEL_ERR_WRITE     // putBytes stored nothing or a short record
};

const char *nvsModelStatusName(NvsModelStatus s);

// True for the statuses that mean "there was a record and it was unusable", as
// opposed to a fresh unit. Those deserve a console line and, if they repeat, a
// look at the flash.
static inline bool nvsModelStatusIsFault(NvsModelStatus s) {
  return s == NVS_MODEL_BAD_MAGIC || s == NVS_MODEL_BAD_SIZE ||
         s == NVS_MODEL_BAD_SCHEMA || s == NVS_MODEL_BAD_CRC ||
         s == NVS_MODEL_ERR_OPEN || s == NVS_MODEL_ERR_WRITE;
}

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Opens the namespace read-write and starts the throttle clock. The clock
// starts here rather than at the first change so that a unit stuck in a reset
// loop cannot write once per boot.
bool nvsModelBegin();

// Reads the stored record and, only if every check passes, hands it to
// FloorMonitor::restore(). On any other status the monitor is not touched.
NvsModelStatus nvsModelLoad(FloorMonitor *fm);

// Writes now, regardless of the throttle. For the paths that know the next
// reboot is imminent - a console command, a brown-out warning - rather than
// for the update loop, which should call nvsModelMaybeSave().
NvsModelStatus nvsModelSave(const FloorMonitor *fm);

// "Something changed, consider saving." Call it wherever the model learns
// something worth keeping - a confirmed stop - and it costs a flag set. The
// actual write happens in nvsModelMaybeSave() once the interval has elapsed.
void nvsModelMarkDirty();

// The update-loop entry point: writes only if something has been marked dirty
// and NVS_SAVE_INTERVAL_MS has passed since the last write. Returns
// NVS_MODEL_SKIPPED when it did nothing, which is the common case.
//
// nowMs is millis(). The comparison is an unsigned difference, so the 49.7-day
// millis() wrap costs at most one late save rather than stalling the throttle
// for the rest of the run - which on a node meant to last years matters.
NvsModelStatus nvsModelMaybeSave(uint32_t nowMs, const FloorMonitor *fm);

// True once nvsModelLoad() has applied a record. This is what sets
// ELEV_FLAG_NVS_RESTORED, so a display can tell a restored model from one
// learned during this run.
bool nvsModelRestored();

// Successful writes since boot, and the status of the last save or load. Both
// go in the console output; the write count is the direct check on whether the
// throttle is doing what section 5.1 claims.
uint32_t nvsModelWriteCount();
NvsModelStatus nvsModelLastStatus();

// Erases the stored record. A deliberate cold boot, for a unit moved to
// another building where the learned pitch and ladder are wrong rather than
// stale.
NvsModelStatus nvsModelErase();
