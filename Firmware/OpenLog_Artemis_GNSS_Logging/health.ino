/*
  health.ino - Reliability telemetry for the OpenLog Artemis GNSS Logger

  Purpose: make resets, hangs and SD stalls *observable*.

  Three artefacts are produced on the SD card:

    BOOT.CSV    - one line per boot: reset reason, boot counter, VIN, RTC,
                  whether the previous session closed its log file cleanly,
                  and the last breadcrumb the previous run managed to leave.
    HEALTH.CSV  - one line per minute while logging: worst SD write latency,
                  GPS buffer high-water mark, free RAM, loop/feed counts.
    (breadcrumb) - a small structure kept in non-initialised SRAM. It survives
                  watchdog / software / external resets (it does NOT survive a
                  power cycle), so the boot record can say where the previous
                  run died and how stale its heartbeat was.

  Nothing in here is on the critical logging path: every function is safe to
  call when the SD card or the GNSS are offline.

  SparkFun code, firmware, and software is released under the MIT License.
*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Tunables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#define HEALTH_BOOT_FILE        "BOOT.CSV"
#define HEALTH_TELEMETRY_FILE   "HEALTH.CSV"
#define HEALTH_INTERVAL_MS      60000UL  // How often a HEALTH.CSV line is written
#define HEALTH_EARLY_WDT_SECONDS 45      // Watchdog timeout used before settings are loaded

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Crash-safe (across reset) breadcrumb in non-initialised SRAM
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#define HEALTH_BREADCRUMB_MAGIC 0x42434F4EUL // 'BCON'

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

// .noinit is not cleared by the C runtime, so a watchdog reset leaves it intact.
// (On the Apollo3 (ELF) this lands in an uninitialised RAM section. Guarded so the
// file can also be syntax-checked on a host compiler.)
#if defined(__arm__)
#define HEALTH_NOINIT __attribute__((section(".noinit"), used))
#else
#define HEALTH_NOINIT
#endif

HEALTH_NOINIT struct_healthBreadcrumb healthBreadcrumb;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// NVM record (one EEPROM write per boot, plus one per clean close)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#define HEALTH_NVM_MAGIC 0x4845414CUL // 'HEAL'

struct struct_healthNVM
{
  uint32_t magic;
  uint32_t bootCount;
  uint32_t sessionOpen;      // 1 = a log file was open the last time we ran
  uint32_t lastResetStatus;  // RSTGEN->STAT captured at the start of the session
  uint32_t cleanCloses;
  char     lastFileName[32];
};

static struct_healthNVM healthNVM;

// Placed immediately after the settings block so it always follows it around.
static int healthNVMOffset(void)
{
  return (int)((sizeof(settings) + 16) & ~15u);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Session state
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

static uint32_t healthLoops = 0;
static uint32_t healthLastTelemetryMs = 0;
static uint32_t healthSessionMaxWriteUs = 0;
static uint32_t healthSessionWrites = 0;
static uint32_t healthSessionSyncs = 0;
static uint16_t healthSessionBufferHighWater = 0;
static bool     healthTelemetryFileHasHeader = false;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Reset reason
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Raw RSTGEN status. 0xFFFFFFFF means "this core does not expose RSTGEN to us".
uint32_t healthResetStatus(void)
{
#ifdef RSTGEN_BASE
  return RSTGEN->STAT;
#else
  return 0xFFFFFFFFUL;
#endif
}

// Human readable decode of the raw status. Every flag is #ifdef'd so that the
// sketch still compiles (and simply prints fewer names) if the core's CMSIS
// header spells a bit differently.
const char *healthResetFlags(uint32_t status)
{
  static char flags[48];
  flags[0] = 0;

  if (status == 0xFFFFFFFFUL)
  {
    strcpy(flags, "n/a");
    return flags;
  }

  if (status == 0)
  {
    strcpy(flags, "none");
    return flags;
  }

  struct { uint32_t mask; const char *name; } table[] = {
#ifdef RSTGEN_STAT_PORSTAT_Msk
    { RSTGEN_STAT_PORSTAT_Msk,  "POR" },
#endif
#ifdef RSTGEN_STAT_BORSTAT_Msk
    { RSTGEN_STAT_BORSTAT_Msk,  "BOR" },
#endif
#ifdef RSTGEN_STAT_BURSTAT_Msk
    { RSTGEN_STAT_BURSTAT_Msk,  "BUR" },
#endif
#ifdef RSTGEN_STAT_POIRSTAT_Msk
    { RSTGEN_STAT_POIRSTAT_Msk, "POI" },
#endif
#ifdef RSTGEN_STAT_SWRSTAT_Msk
    { RSTGEN_STAT_SWRSTAT_Msk,  "SW" },
#endif
#ifdef RSTGEN_STAT_EXRSTAT_Msk
    { RSTGEN_STAT_EXRSTAT_Msk,  "EXT" },
#endif
#ifdef RSTGEN_STAT_WDRSTAT_Msk
    { RSTGEN_STAT_WDRSTAT_Msk,  "WDT" },
#endif
    { 0, 0 }
  };

  for (int i = 0; table[i].name != 0; i++)
  {
    if (status & table[i].mask)
    {
      if (flags[0]) strncat(flags, "|", sizeof(flags) - strlen(flags) - 1);
      strncat(flags, table[i].name, sizeof(flags) - strlen(flags) - 1);
    }
  }

  if (!flags[0]) strcpy(flags, "unknown");

  return flags;
}

const char *healthPhaseName(uint32_t phase)
{
  switch (phase)
  {
    case HEALTH_PHASE_NONE:            return "none";
    case HEALTH_PHASE_SETUP_START:     return "setup-start";
    case HEALTH_PHASE_SD_BEGIN:        return "sd-begin";
    case HEALTH_PHASE_SETTINGS_LOADED: return "settings-loaded";
    case HEALTH_PHASE_BOOT_RECORDED:   return "boot-recorded";
    case HEALTH_PHASE_DATA_LOGGING:    return "data-logging";
    case HEALTH_PHASE_SENSORS:         return "sensors";
    case HEALTH_PHASE_READY:           return "ready";
    case HEALTH_PHASE_LOOP:            return "loop";
    case HEALTH_PHASE_SD_WRITE:        return "sd-write";
    case HEALTH_PHASE_ROTATE:          return "rotate";
    case HEALTH_PHASE_POWER_DOWN:      return "power-down";
    case HEALTH_PHASE_SLEEP:           return "sleep";
    default:                           return "?";
  }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Breadcrumb access
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Record where we are. Cheap (a few stores) so it can be called freely.
void healthPhase(uint32_t phase)
{
  healthBreadcrumb.magic = HEALTH_BREADCRUMB_MAGIC;
  healthBreadcrumb.phase = phase;
  healthBreadcrumb.beats++;
  healthBreadcrumb.lastBeatMs = millis();
}

// Was the breadcrumb left behind by a previous run (rather than being stale
// garbage from a power cycle)? Only meaningful when the reset was *not* a POR.
bool healthBreadcrumbValid(uint32_t resetStatus)
{
  if (healthBreadcrumb.magic != HEALTH_BREADCRUMB_MAGIC)
    return false;

  // If the supply cycled, SRAM content is meaningless.
  if (resetStatus == 0xFFFFFFFFUL)
    return true; // We cannot tell; trust the magic.

#ifdef RSTGEN_STAT_PORSTAT_Msk
  if (resetStatus & RSTGEN_STAT_PORSTAT_Msk)
    return false;
#endif
#ifdef RSTGEN_STAT_BORSTAT_Msk
  if (resetStatus & RSTGEN_STAT_BORSTAT_Msk)
    return false;
#endif

  return true;
}

// Called from storeData() around every SD write.
void healthNoteWrite(uint32_t elapsedUs, uint16_t bytes)
{
  (void)bytes;
  healthSessionWrites++;
  if (elapsedUs > healthSessionMaxWriteUs)
    healthSessionMaxWriteUs = elapsedUs;

  // Keep the crash-surviving copy somewhere near current, but this is a few
  // stores - no SD, no flash.
  healthBreadcrumb.magic = HEALTH_BREADCRUMB_MAGIC;
  healthBreadcrumb.writes = healthSessionWrites;
  if (elapsedUs > healthBreadcrumb.maxWriteUs)
    healthBreadcrumb.maxWriteUs = elapsedUs;
}

// Called around gnssDataFile.sync() / close(): the classic long-tail SD stall.
void healthNoteSync(uint32_t elapsedUs)
{
  healthSessionSyncs++;
  if (elapsedUs > healthSessionMaxWriteUs)
    healthSessionMaxWriteUs = elapsedUs;

  healthBreadcrumb.magic = HEALTH_BREADCRUMB_MAGIC;
  if (elapsedUs > healthBreadcrumb.maxWriteUs)
    healthBreadcrumb.maxWriteUs = elapsedUs;
}

void healthNoteBuffer(uint16_t fill)
{
  if (fill > healthSessionBufferHighWater)
    healthSessionBufferHighWater = fill;
  if (fill > healthBreadcrumb.bufferHighWater)
    healthBreadcrumb.bufferHighWater = fill;
}

void healthNoteFeed(void)
{
  healthBreadcrumb.magic = HEALTH_BREADCRUMB_MAGIC;
  healthBreadcrumb.feeds++;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Free RAM (heap-to-stack gap)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

extern "C" void *sbrk(intptr_t i);

uint32_t healthFreeRam(void)
{
  char marker;
  char *heapEnd = (char *)sbrk(0);
  if (heapEnd == NULL)
    return 0;
  int32_t freeBytes = (int32_t)((char *)&marker - heapEnd);
  return (freeBytes > 0) ? (uint32_t)freeBytes : 0;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// CSV helpers
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#if SD_FAT_TYPE == 1
typedef File32 HealthFile;
#elif SD_FAT_TYPE == 2
typedef ExFile HealthFile;
#elif SD_FAT_TYPE == 3
typedef FsFile HealthFile;
#else
typedef File HealthFile;
#endif

static void healthRtcTimestamp(char *buf, size_t len)
{
  myRTC.getTime();
  snprintf(buf, len, "%04d-%02d-%02dT%02d:%02d:%02dZ",
           myRTC.year + 2000, myRTC.month, myRTC.dayOfMonth,
           myRTC.hour, myRTC.minute, myRTC.seconds);
}

// Append one line. The file is opened, written and closed every time so that a
// reset never leaves a half-flushed telemetry file that upsets the FAT.
#define HEALTH_MAX_FILE_BYTES (512UL * 1024UL)

static bool healthAppendLine(const char *fileName, const char *header, const char *line)
{
  if (online.microSD == false)
    return false;

  bool fileExisted = sd.exists(fileName);

  HealthFile f;
  if (f.open(fileName, O_CREAT | O_APPEND | O_WRITE) == false)
    return false;

  // Keep the telemetry files bounded even if the logger is reset-looping.
  if (f.fileSize() > HEALTH_MAX_FILE_BYTES)
  {
    f.close();
    sd.remove(fileName);
    fileExisted = false;
    if (f.open(fileName, O_CREAT | O_APPEND | O_WRITE) == false)
      return false;
  }

  bool ok = true;

  if (!fileExisted && header != NULL)
  {
    size_t headerLen = strlen(header);
    ok = (f.write(header, headerLen) == headerLen) && (f.write("\r\n", 2) == 2);
  }

  size_t lineLen = strlen(line);
  if (ok)
    ok = (f.write(line, lineLen) == lineLen) && (f.write("\r\n", 2) == 2);

  f.sync();
  f.close();

  return ok;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// NVM
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void healthLoadNVM(void)
{
  EEPROM.get(healthNVMOffset(), healthNVM);

  if (healthNVM.magic != HEALTH_NVM_MAGIC)
  {
    memset(&healthNVM, 0, sizeof(healthNVM));
    healthNVM.magic = HEALTH_NVM_MAGIC;
  }

  healthNVM.lastFileName[sizeof(healthNVM.lastFileName) - 1] = 0;
}

static void healthSaveNVM(void)
{
  healthNVM.magic = HEALTH_NVM_MAGIC;
  EEPROM.put(healthNVMOffset(), healthNVM);
}

// Mark that a log file is open. Two flash cycles per session.
void healthMarkSessionOpen(const char *fileName)
{
  healthNVM.sessionOpen = 1;
  healthNVM.lastResetStatus = healthResetStatus();
  strncpy(healthNVM.lastFileName, fileName, sizeof(healthNVM.lastFileName) - 1);
  healthNVM.lastFileName[sizeof(healthNVM.lastFileName) - 1] = 0;
  healthSaveNVM();
}

// Mark that the log file was closed properly before the reset/power-off.
void healthMarkSessionClean(void)
{
  if (healthNVM.sessionOpen == 0)
    return;

  healthNVM.sessionOpen = 0;
  healthNVM.cleanCloses++;
  healthSaveNVM();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// BOOT.CSV
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

static const char healthBootHeader[] =
  "#boot,rtc_utc,vin_v,reset_raw,reset_flags,prev_log,prev_clean,prev_phase,"
  "bc_valid,bc_phase,bc_beats,bc_age_ms,bc_max_write_us,bc_buffer_hw,bc_feeds,fw";

void healthOnBoot(void)
{
  healthLoadNVM();

  uint32_t resetStatus = healthResetStatus();
  bool bcValid = healthBreadcrumbValid(resetStatus);

  uint32_t bootNumber = healthNVM.bootCount + 1;
  uint32_t prevPhase = bcValid ? healthBreadcrumb.phase : 0;
  uint32_t prevBeats = bcValid ? healthBreadcrumb.beats : 0;
  uint32_t prevAgeMs = bcValid ? (millis() - healthBreadcrumb.lastBeatMs) : 0;
  uint32_t prevMaxWriteUs = bcValid ? healthBreadcrumb.maxWriteUs : 0;
  uint32_t prevBufHw = bcValid ? healthBreadcrumb.bufferHighWater : 0;
  uint32_t prevFeeds = bcValid ? healthBreadcrumb.feeds : 0;

  char rtc[32];
  healthRtcTimestamp(rtc, sizeof(rtc));

  // %f is not enabled in newlib-nano, so format the voltage by hand.
  char vinStr[12];
  olaftoa(readVIN(), vinStr, 2, sizeof(vinStr));

  char line[320];
  snprintf(line, sizeof(line),
           "%lu,%s,%s,0x%08lX,%s,%s,%d,%s,%d,%s,%lu,%lu,%lu,%lu,%lu,%d.%d",
           (unsigned long)bootNumber,
           rtc,
           vinStr,
           (unsigned long)resetStatus,
           healthResetFlags(resetStatus),
           (healthNVM.lastFileName[0] == 0) ? "-" : healthNVM.lastFileName,
           (int)healthNVM.sessionOpen,                  // 1 = previous run did NOT close cleanly
           bcValid ? healthPhaseName(prevPhase) : "stale",
           (int)bcValid,
           healthPhaseName(prevPhase),
           (unsigned long)prevBeats,
           (unsigned long)prevAgeMs,
           (unsigned long)prevMaxWriteUs,
           (unsigned long)prevBufHw,
           (unsigned long)prevFeeds,
           FIRMWARE_VERSION_MAJOR,
           FIRMWARE_VERSION_MINOR);

  healthAppendLine(HEALTH_BOOT_FILE, healthBootHeader, line);

  healthNVM.bootCount = bootNumber;
  healthSaveNVM();

  // Start this session with a clean breadcrumb.
  memset(&healthBreadcrumb, 0, sizeof(healthBreadcrumb));
  healthPhase(HEALTH_PHASE_BOOT_RECORDED);

  if (settings.printMajorDebugMessages == true)
  {
    Serial.print(F("BOOT.CSV: "));
    Serial.println(line);
  }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// HEALTH.CSV
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

static const char healthTelemetryHeader[] =
  "#uptime_ms,rtc_utc,log_file,gps_buf_now,gps_buf_max,free_ram,"
  "sd_writes,sd_syncs,sd_max_stall_us,loops,wdt_feeds,reset_raw";

static void healthWriteTelemetry(void)
{
  uint16_t bufferNow = 0;
  if (qwiicOnline.uBlox && qwiicAvailable.uBlox)
    bufferNow = gpsSensor_ublox.fileBufferAvailable();

  char rtc[32];
  healthRtcTimestamp(rtc, sizeof(rtc));

  char line[256];
  snprintf(line, sizeof(line),
           "%lu,%s,%s,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,0x%08lX",
           (unsigned long)millis(),
           rtc,
           (gnssDataFileName[0] == 0) ? "-" : gnssDataFileName,
           (unsigned)bufferNow,
           (unsigned)healthSessionBufferHighWater,
           (unsigned long)healthFreeRam(),
           (unsigned long)healthSessionWrites,
           (unsigned long)healthSessionSyncs,
           (unsigned long)healthSessionMaxWriteUs,
           (unsigned long)healthLoops,
           (unsigned long)healthBreadcrumb.feeds,
           (unsigned long)healthResetStatus());

  healthAppendLine(HEALTH_TELEMETRY_FILE, healthTelemetryHeader, line);

  // Next line reports a fresh one-minute window
  healthSessionMaxWriteUs = 0;
  healthSessionWrites = 0;
  healthSessionSyncs = 0;
}

// Called once per loop() iteration.
void healthTick(void)
{
  healthLoops++;

  healthBreadcrumb.magic = HEALTH_BREADCRUMB_MAGIC;
  healthBreadcrumb.phase = HEALTH_PHASE_LOOP;
  healthBreadcrumb.beats++;
  healthBreadcrumb.lastBeatMs = millis();

  if ((uint32_t)(millis() - healthLastTelemetryMs) >= HEALTH_INTERVAL_MS)
  {
    healthLastTelemetryMs = millis();
    healthWriteTelemetry();
  }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Early watchdog
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Arm the watchdog as early as possible in setup(). The configured watchdog is
// re-initialised later (after settings are loaded); this one only exists to
// cover the boot path, where the logger previously had no watchdog at all.
void healthArmEarlyWatchdog(void)
{
  initWatchdog(HEALTH_EARLY_WDT_SECONDS);
  feedWatchdog();
}
