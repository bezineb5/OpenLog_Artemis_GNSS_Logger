/*
  health.h - Declarations for the reliability telemetry in health.ino

  The Arduino builder concatenates the main .ino first and the other .ino
  tabs after it in alphabetical order, and only auto-generates function
  prototypes - not enums or variables. Anything used across tabs (the phase
  enum, the breadcrumb struct and its instance) therefore has to live in a
  header included from the main sketch.
*/

#ifndef HEALTH_H
#define HEALTH_H

#include <stdint.h>

enum healthPhaseId : uint32_t
{
  HEALTH_PHASE_NONE = 0,
  HEALTH_PHASE_SETUP_START,
  HEALTH_PHASE_SD_BEGIN,
  HEALTH_PHASE_SETTINGS_LOADED,
  HEALTH_PHASE_BOOT_RECORDED,
  HEALTH_PHASE_DATA_LOGGING,
  HEALTH_PHASE_SENSORS,
  HEALTH_PHASE_READY,
  HEALTH_PHASE_LOOP,
  HEALTH_PHASE_SD_WRITE,
  HEALTH_PHASE_ROTATE,
  HEALTH_PHASE_POWER_DOWN,
  HEALTH_PHASE_SLEEP,
};

struct struct_healthBreadcrumb
{
  uint32_t magic;          // HEALTH_BREADCRUMB_MAGIC when the contents are meaningful
  uint32_t phase;          // Last healthPhase reached (see healthPhaseName)
  uint32_t beats;          // Heartbeat counter (increments once per loop)
  uint32_t lastBeatMs;     // millis() at the last heartbeat
  uint32_t maxWriteUs;     // Worst single gnssDataFile.write() in this session
  uint32_t writes;         // Number of gnssDataFile.write() calls this session
  uint32_t feeds;          // Watchdog feeds this session
  uint16_t bufferHighWater;// Worst GPS file-buffer fill seen
  uint16_t reserved;
};

// Defined in health.ino in a section that survives resets (not power cycles).
extern struct_healthBreadcrumb healthBreadcrumb;

void healthPhase(uint32_t phase);
void healthArmEarlyWatchdog(void);
void healthOnBoot(void);
void healthTick(void);
void healthNoteWrite(uint32_t elapsedUs, uint16_t bytes);
void healthNoteSync(uint32_t elapsedUs);
void healthNoteBuffer(uint16_t fill);
void healthNoteFeed(void);
void healthMarkSessionOpen(const char *fileName);
void healthMarkSessionClean(void);
uint32_t healthResetStatus(void);
bool healthBreadcrumbValid(uint32_t resetStatus);
const char *healthPhaseName(uint32_t phase);
uint32_t healthFreeRam(void);

#endif // HEALTH_H
