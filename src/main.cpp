/*
 * Twente Cough Machine Control Firmware (main.cpp)
 *
 * Controls valves for atomisation experiments with precise timing;
 * reads pressure, temperature, and humidity sensors; manages flow curve
 * datasets; and logs execution data to flash.

 * Responsibilities:
 * - Hardware setup (valves, laser/PDA, pressure + T/H sensors, QSPI flash)
 * - Runtime mode/state machine
 *   (Idle, DropletDetect, DelayBeforeRun, ExecutingRun, LaserTest)
 * - Serial command parsing and execution (see README for command protocol)
 *
 * File layout:
 * 1) Constants/config + global state
 * 2) Hardware and persistence helpers
 * 3) loop() mode processing + command dispatch
 */

// TODO: Fix first run log of run still containing old data
#include "Adafruit_SPIFlash.h"
#include "DvG_StreamCommand.h"
#include "MIKROE_4_20mA_RT_Click.h"
#include "SdFat.h"
#include <Adafruit_DotStar.h>
#include <Adafruit_SHT4x.h>
#include <Arduino.h>
#include <stdlib.h>

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void printError(const char *message);
template <typename T> void printError(const char *message, T value);
void resetDataArrays();

// ============================================================================
// CONFIGURATION
// ============================================================================
// Keep fixed hardware, protocol, timing, calibration, and buffer values here.
// Runtime state and hardware objects begin in the next section.

// Protocol and serial communication
const uint8_t TCM_PROTOCOL_VERSION = 6;
const uint32_t SERIAL_BAUD_RATE = 115200;
const uint32_t SERIAL_TIMEOUT_MS = 10;

// Hardware pin mapping for the ItsyBitsy M4
const int PIN_VALVE = 7;           // MOSFET gate pin for solenoid valve control
const int PIN_PROP_VALVE = 11;     // Chip select for proportional valve
const int PIN_TANK_CS_TCLICK = 10; // Chip select for tank pressure regulator
const int PIN_TANK_CS_RCLICK = 2;  // Chip select for tank pressure sensor (SPI)
const int PIN_NEB_CS_TCLICK = 4; // Chip select for nebuliser pressure regulator
const int PIN_NEB_CS_RCLICK =
    13;                   // Chip select for nebuliser pressure sensor (SPI)
const int PIN_TRIG = 9;   // Trigger output for peripheral synchronization
const int PIN_LASER = 12; // Laser MOSFET gate pin for droplet detection
const int PIN_LIGHT = 5;  // Light output pin (PWM brightness)
const int PIN_FAN = 3;    // Fan speed control pin (PWM, not yet implemented)
const int PIN_NEB = A3;   // Nebuliser control pin (digital)
const int PIN_PDA = A2;   // Analog input from photodetector
// PIN_DOTSTAR_DATA and PIN_DOTSTAR_CLK are defined in variant.h.

// Buffer, file, and dataset persistence limits
const int MAX_DATA_LENGTH = 2000;
const uint16_t CMD_BUF_LEN = 32000;
const int MAX_RECORDS = 2000;
const size_t PERSISTENT_STATE_LINE_LENGTH = 64;
const size_t FLASH_ENTRY_NAME_LENGTH = 64;
const size_t RUN_LOG_FILENAME_LENGTH = 32;
const char *STATE_FILE = "run_state.txt";
const char *DATASET_FILE = "dataset_state.bin";
const char *RUN_LOG_PREFIX = "experiment_log_";
const char *RUN_LOG_FILENAME_FORMAT = "experiment_log_%04lu.csv";
const uint32_t DATASET_MAGIC = 0x54434D46; // "TCMF"
const uint8_t DATASET_FORMAT_VERSION = 2;

// Run, trigger, and user-interface timing
const uint32_t TRIGGER_WIDTH = 10000; // Trigger pulse width [us]
const uint32_t DEFAULT_PDA_DELAY_US = 10000;
const uint32_t DATASET_TIME_SCALE_US_PER_MS = 1000;
const uint32_t LASER_TEST_STREAM_INTERVAL_MS = 50;
const uint8_t ERROR_LED_FLASH_COUNT = 5;
const uint32_t ERROR_LED_FLASH_INTERVAL_MS = 300;

// Sensor acquisition and ADC conversion
const uint32_t RCLICK_EMA_INTERVAL = 10; // Sampling interval for EMA [us]
const float RCLICK_EMA_LP_FREQ =
    500.0f; // Low-pass filter cutoff frequency [Hz]
const uint8_t ADC_RESOLUTION_BITS = 12;
const float ADC_MAX_VALUE = 4095.0f;
const float ADC_REFERENCE_VOLTAGE = 3.3f;
const uint8_t SIGNAL_PRINT_DECIMAL_PLACES = 3;
const float PDA_R1 = 6710.0f;     // Voltage divider resistor [Ohm]
const float PDA_R2 = 3260.0f;     // Voltage divider resistor [Ohm]
const float PDA_THR = 4.8f;       // Droplet detection threshold [V]
const float PDA_MIN_VALID = 0.1f; // Minimum valid signal [V]

// Pressure conversion and 4-20 mA calibration
struct PressureCalibration {
  float set_bar_per_mA;
  float set_bar_offset;
  float read_bar_per_mA;
  float read_bar_offset;
};

const PressureCalibration TANK_PRESS_CALIBRATION{
    0.62242857f, 2.48821429f, 0.6255112463192659f, -2.534598501736508f};
// TODO: Replace with calibration values for the nebuliser pressure loop.
const PressureCalibration NEB_PRESS_CALIBRATION{
    0.62242857f, 2.48821429f, 0.6255112463192659f, -2.534598501736508f};
const RT_Click_Calibration R_CLICK_CALIBRATION{4.04, 10.98, 806, 2191};
const RT_Click_Calibration T_CLICK_CALIBRATION{3.97, 19.90, 796, 3982};

// Actuator and display defaults
const float MAX_CURR_MA = 20.0f;
const float MIN_CURR_VALVE_MA = 12.0f;
const float MIN_CURR_PRESS_REG_MA = 3.9f;
const float DEF_CURR_VALVE_MA = 12.0f;
const float DEF_PRESSURE = 4.0f;
const uint8_t LED_BRIGHTNESS = 255;
const float LIGHT_LEVEL_MIN = 0.0f;
const float LIGHT_LEVEL_MAX = 1.0f;
const float PWM_MAX_DUTY = 255.0f;

// DotStar colors use BGR format and avoid pure red for laser goggles.
const uint32_t COLOR_IDLE = 0x001000;
const uint32_t COLOR_VALVE_OPEN = 0x00FF00;
const uint32_t COLOR_ERROR = 0xFF4000;
const uint32_t COLOR_READING = 0xFF0040;
const uint32_t COLOR_LASER = 0x100000;
const uint32_t COLOR_DROPLET = 0xFF0000;
const uint32_t COLOR_WAITING = 0x400040;
const uint32_t COLOR_RECEIVING = 0x100000;
const uint32_t COLOR_EXECUTING = 0xFF0000;
const uint32_t COLOR_OFF = 0x000000;

// ============================================================================
// SERIAL COMMAND DEFINITIONS
// ============================================================================
// Incoming serial lines are first mapped to a CommandId by matching this table,
// then the command dispatcher performs the corresponding action. Commands with
// shared prefixes (for example P? and P) rely on the most specific entry first.
enum class CommandId : uint8_t {
  IdQuery,
  ProtocolVersionQuery,
  DebugToggle,
  StatusQuery,
  Help,
  SetValve,
  SetTankPressure,
  SetNebPressure,
  OpenSolenoid,
  CloseSolenoid,
  Quit,
  TriggerOnce,
  LaserTestToggle,
  LightToggle,
  FanSpeed,
  NebuliserToggle,
  ReadTankPressure,
  ReadNebPressure,
  ReadTempHumidity,
  WaitSet,
  WaitQuery,
  ClearMemory,
  ClearLogs,
  LoadDataset,
  DatasetStatus,
  Run,
  DropletRun,
  DropletDetect,
  Other
};

struct CommandDefinition {
  const char *prefix;
  CommandId id;
};

// Order matters: more specific command prefixes must appear first.
const CommandDefinition COMMAND_DEFINITIONS[] = {
    {"id?", CommandId::IdQuery},
    {"ver?", CommandId::ProtocolVersionQuery},
    {"S?", CommandId::StatusQuery},
    {"P?", CommandId::ReadTankPressure},
    {"M?", CommandId::ReadNebPressure},
    {"T?", CommandId::ReadTempHumidity},
    {"W?", CommandId::WaitQuery},
    {"X!", CommandId::ClearMemory},
    {"L?", CommandId::DatasetStatus},
    {"D!", CommandId::DropletRun},
    {"B", CommandId::DebugToggle},
    {"V", CommandId::SetValve},
    {"P", CommandId::SetTankPressure},
    {"M", CommandId::SetNebPressure},
    {"O", CommandId::OpenSolenoid},
    {"C", CommandId::CloseSolenoid},
    {"Q", CommandId::Quit},
    {"G", CommandId::TriggerOnce},
    {"A", CommandId::LaserTestToggle},
    {"I", CommandId::LightToggle},
    {"F", CommandId::FanSpeed},
    {"N", CommandId::NebuliserToggle},
    {"W", CommandId::WaitSet},
    {"X", CommandId::ClearLogs},
    {"L", CommandId::LoadDataset},
    {"R", CommandId::Run},
    {"D", CommandId::DropletDetect},
    {"?", CommandId::Help},
};

CommandId parseCommandId(const char *command) {
  for (const CommandDefinition &definition : COMMAND_DEFINITIONS) {
    if (strncmp(command, definition.prefix, strlen(definition.prefix)) == 0) {
      return definition.id;
    }
  }
  return CommandId::Other;
}

bool parseDropletRunCount(const char *command, bool runAfterDetection,
                          int32_t &requestedCount) {
  // No count selects continuous mode; explicit counts must be positive.
  requestedCount = -1;
  const size_t prefixLength = runAfterDetection ? 2 : 1;
  if (strlen(command) <= prefixLength) {
    return true;
  }

  requestedCount = parseIntInString(command, prefixLength);
  if (requestedCount > 0) {
    return true;
  }

  if (runAfterDetection) {
    printError("D! count must be >= 1! To run indefinitely, send D! without a "
               "number.");
  } else {
    printError("D count must be >= 1! To run indefinitely, send D without a "
               "number.");
  }
  return false;
}

// ============================================================================
// CONTROLLER STATE
// ============================================================================
// State shared by the run, droplet-detection, trigger, and laser-test flows.
enum class LoopMode : uint8_t {
  Idle,
  DropletDetect,
  DelayBeforeRun,
  ExecutingRun,
  LaserTest
};

struct ControllerState {
  LoopMode mode = LoopMode::Idle;
  bool solValveOpen = false;
  bool performingTrigger = false;
  bool belowThreshold = false;
  bool detectionBaselineReady = false;
  uint32_t delayedRunStartTime = 0;
  uint32_t detectionStartTime = 0;
  bool runAfterDropletDetection = false;
  bool pressureConfigured = false;
  uint32_t laserTestLastPrint = 0;
};

ControllerState controllerState;

// ============================================================================
// RUNTIME STATE AND HARDWARE
// ============================================================================
// Debug output is disabled by default and can be enabled via the B command.
bool debug_enabled = false;

// Convenience macros to keep debug logging cheap when disabled
#define DEBUG_PRINT(x)                                                         \
  do {                                                                         \
    if (debug_enabled) {                                                       \
      Serial.print(x);                                                         \
    }                                                                          \
  } while (0)
#define DEBUG_PRINTLN(x)                                                       \
  do {                                                                         \
    if (debug_enabled) {                                                       \
      Serial.println(x);                                                       \
    }                                                                          \
  } while (0)

// ============================================================================
// FLOW-CURVE DATASET BUFFERS
// ============================================================================
// Buffers used for receiving and storing the uploaded dataset
int incomingCount = 0;           // Declare incoming dataset length globally
char cmd_buf[CMD_BUF_LEN]{'\0'}; // Instantiate empty Serial buffer
uint32_t time_array[MAX_DATA_LENGTH];       // Time dataset
float value_array[MAX_DATA_LENGTH];         // mA dataset
uint8_t sol_enable_array[MAX_DATA_LENGTH];  // 0/1: solenoid enable
uint8_t trig_enable_array[MAX_DATA_LENGTH]; // 0/1: trigger pulse event
// Create DvG_StreamCommand object on Serial stream
DvG_StreamCommand sc(Serial, cmd_buf, CMD_BUF_LEN);

// ============================================================================
// FLOW-CURVE EXECUTION RUNTIME STATE
// ============================================================================
// Indices and counters used during flow curve playback
int sequenceIndex = 0;     // Index of dataset to execute on time
int dataIndex = 0;         // Number of datapoints of dataset stored
int datasetDuration = 0.0; // Duration of the uploaded flow profile

// ============================================================================
// QSPI FLASH FILESYSTEM INTERFACE
// ============================================================================
// QSPI flash storage for logged flow curve datasets
// Setup for the ItsyBitsy M4 internal QSPI flash
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
// The filesystem object
FatFileSystem fatfs;

// ============================================================================
// FLOW-CURVE EXECUTION LOGGING (RAM)
// ============================================================================
// Log format stored in RAM before it is saved to flash
struct __attribute__((__packed__)) LogEntry {
  uint32_t timestamp; // 4 bytes (micros)
  int8_t valve1;      // 1 byte (1, 0, -1)
  float valve2_mA;    // 4 bytes
  float pressure;     // 4 bytes
};

// Initialize logging array in RAM
LogEntry logs[MAX_RECORDS];
int currentCount = 0;
bool runLogActive = false;      // True only while a dataset run is active
uint32_t runLogTriggerUs = 0;   // First trigger timestamp for current run [us]
bool runLogTriggerSeen = false; // True once the run-local trigger has fired
uint32_t runCounter = 0;        // Per-session run index (starts at 0 each boot)
char lastSavedFilename[RUN_LOG_FILENAME_LENGTH] = ""; // Latest saved file
float lastPressure_bar = 0.0f; // Persisted pressure regulator setting [bar]
bool pressureInitializedFromFlash = false; // Valid persisted pressure loaded
bool waitInitializedFromFlash = false;     // Valid persisted wait loaded

// Timing state for trigger, droplet detection, and run scheduling
uint32_t tick = 0; // Timestamp for timing events [µs]

uint32_t pre_trigger_delay_us =
    0; // Delay between droplet detection/RUN command and opening valve [µs] ->
       // example value 59500 (outdated)
uint32_t pda_delay =
    DEFAULT_PDA_DELAY_US; // Delay before photodiode starts detecting again [µs]

int32_t dropletRunsRemaining = 0; // -1 = continuous, 0 = idle, >0 = remaining

// This is just another ticker
uint32_t runCallTime = 0; // Time elapsed since "RUN" command [µs]

// Persistent state and session tracking
uint32_t lastSessionCount = 0;
// Pressure sensor (4-20mA R-Click) with exponential moving average filtering
R_Click tank_RClick(PIN_TANK_CS_RCLICK, R_CLICK_CALIBRATION,
                    RCLICK_EMA_INTERVAL, RCLICK_EMA_LP_FREQ);

// TODO: Replace with calibration values for the nebuliser R-Click.
R_Click neb_RClick(PIN_NEB_CS_RCLICK, R_CLICK_CALIBRATION, RCLICK_EMA_INTERVAL,
                   RCLICK_EMA_LP_FREQ);

// Temperature & humidity sensor (SHT4x I2C)
Adafruit_SHT4x sht4;

// Proportional valve and pressure regulator interfaces
T_Click valve(PIN_PROP_VALVE, T_CLICK_CALIBRATION);
T_Click pressure(PIN_TANK_CS_TCLICK, T_CLICK_CALIBRATION);

// TODO: Replace with calibration values for the nebuliser T-Click.
T_Click neb_pressure(PIN_NEB_CS_TCLICK, T_CLICK_CALIBRATION);

// DotStar RGB LED (using board's built-in DotStar on pins 8 and 6)
Adafruit_DotStar led(1, PIN_DOTSTAR_DATA, PIN_DOTSTAR_CLK, DOTSTAR_BGR);

// ============================================================================
// LED HELPER FUNCTION
// ============================================================================
void setLedColor(uint32_t color) {
  // Set the DotStar LED to a specific color
  led.setPixelColor(0, color);
  led.show();
}

void beginRunLog() {
  // Start a fresh per-run log and ignore any earlier housekeeping events.
  currentCount = 0;
  runLogActive = true;
  runLogTriggerUs = micros();
  runLogTriggerSeen = false;
}

void markRunTrigger() {
  // Keep the first trigger timestamp as the time origin for this run log.
  if (!runLogActive || runLogTriggerSeen) {
    return;
  }

  runLogTriggerUs = micros();
  runLogTriggerSeen = true;
}

void endRunLog() { runLogActive = false; }
// ============================================================================
// FUNCTION TO STORE EXECUTION EVENTS IN RAM
// ============================================================================
void recordEvent(int8_t v1, float v2, float press) {
  // Only record events that belong to an actively executing flow-curve run.
  if (!runLogActive) {
    return;
  }

  // Append a log entry if there is space
  if (currentCount < MAX_RECORDS) {
    logs[currentCount] = {micros(), v1, v2, press};
    currentCount++;
  }
}

float pressureBarToCurrent(float bar, const PressureCalibration &calibration) {
  return (bar + calibration.set_bar_offset) / calibration.set_bar_per_mA;
}

float pressureCurrentToBar(float current_mA,
                           const PressureCalibration &calibration) {
  return calibration.read_bar_per_mA * current_mA + calibration.read_bar_offset;
}

// ============================================================================
// FUNCTION TO STORE DATA TO FLASH INSTEAD OF RAM
// ============================================================================
void savePersistentState() {
  // Store selected settings so they survive power cycles
  if (fatfs.exists(STATE_FILE)) {
    fatfs.remove(STATE_FILE);
  }

  File file = fatfs.open(STATE_FILE, FILE_WRITE);
  if (!file) {
    DEBUG_PRINTLN("Error opening state file for writing!");
    return;
  }

  // Only persist pressure when it is known to be valid
  if (pressureInitializedFromFlash) {
    file.printf("lastPressure_bar=%.3f\n", lastPressure_bar);
  }
  if (waitInitializedFromFlash) {
    file.printf("preTriggerDelay_us=%lu\n",
                static_cast<unsigned long>(pre_trigger_delay_us));
  }
  file.close();
}

void loadPersistentState() {
  // Restore settings if the state file exists
  pressureInitializedFromFlash = false;
  waitInitializedFromFlash = false;
  lastPressure_bar = 0.0f;
  if (!fatfs.exists(STATE_FILE)) {
    return;
  }

  File file = fatfs.open(STATE_FILE, FILE_READ);
  if (!file) {
    DEBUG_PRINTLN("Error opening state file for reading!");
    return;
  }

  char line[PERSISTENT_STATE_LINE_LENGTH];
  while (file.available()) {
    size_t len = file.readBytesUntil('\n', line, sizeof(line) - 1);
    line[len] = '\0';

    // Parse known keys line-by-line (avoid sscanf float parsing)
    const char *pressureKey = "lastPressure_bar=";
    if (strncmp(line, pressureKey, strlen(pressureKey)) == 0) {
      const char *value = line + strlen(pressureKey);
      lastPressure_bar = atof(value);
      float current =
          pressureBarToCurrent(lastPressure_bar, TANK_PRESS_CALIBRATION);
      // Validate range before applying
      if (current >= MIN_CURR_PRESS_REG_MA && current <= MAX_CURR_MA) {
        pressureInitializedFromFlash = true;
      }
      continue;
    }

    const char *waitKey = "preTriggerDelay_us=";
    if (strncmp(line, waitKey, strlen(waitKey)) == 0) {
      const char *value = line + strlen(waitKey);
      unsigned long delayValue = strtoul(value, nullptr, 10);
      pre_trigger_delay_us = static_cast<uint32_t>(delayValue);
      waitInitializedFromFlash = true;
    }
  }
  file.close();
}

// ============================================================================
// FLOW CURVE DATASET PERSISTENCE (FLASH)
// ============================================================================
struct __attribute__((__packed__)) DatasetHeader {
  uint32_t magic;
  uint32_t count;
  uint32_t duration_ms;
  uint8_t format_version;
};

struct __attribute__((__packed__)) DatasetRow {
  uint32_t time_ms;
  float value_mA;
  uint8_t enable;
  uint8_t trigger;
};

bool saveDatasetToFlash() {
  if (dataIndex <= 0) {
    return false;
  }

  if (fatfs.exists(DATASET_FILE)) {
    fatfs.remove(DATASET_FILE);
  }

  File file = fatfs.open(DATASET_FILE, FILE_WRITE);
  if (!file) {
    printError("Error opening dataset file for writing!");
    return false;
  }

  DatasetHeader header{DATASET_MAGIC, static_cast<uint32_t>(dataIndex),
                       static_cast<uint32_t>(datasetDuration),
                       DATASET_FORMAT_VERSION};
  if (file.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header)) !=
      sizeof(header)) {
    printError("Error writing dataset header!");
    file.close();
    return false;
  }

  for (int i = 0; i < dataIndex; i++) {
    DatasetRow row{time_array[i], value_array[i], sol_enable_array[i],
                   trig_enable_array[i]};
    if (file.write(reinterpret_cast<const uint8_t *>(&row), sizeof(row)) !=
        sizeof(row)) {
      printError("Error writing dataset row!");
      file.close();
      return false;
    }
  }

  file.close();
  return true;
}

bool loadDatasetFromFlash() {
  if (!fatfs.exists(DATASET_FILE)) {
    return false;
  }

  File file = fatfs.open(DATASET_FILE, FILE_READ);
  if (!file) {
    printError("Error opening dataset file for reading!");
    return false;
  }

  DatasetHeader header{};
  if (file.read(reinterpret_cast<uint8_t *>(&header), sizeof(header)) !=
      sizeof(header)) {
    printError("Error reading dataset header!");
    file.close();
    return false;
  }

  if (header.count == 0 || header.count > MAX_DATA_LENGTH) {
    printError("Dataset header invalid!");
    file.close();
    return false;
  }

  // Reject datasets that are not ours (magic mismatch) or that were written
  // with a different row/header schema (version mismatch).
  // This prevents mis-parsing stale/corrupt files after protocol changes.
  if (header.magic != DATASET_MAGIC ||
      header.format_version != DATASET_FORMAT_VERSION) {
    printError("Dataset format mismatch! Re-upload flow curve.");
    file.close();
    return false;
  }

  for (uint32_t i = 0; i < header.count; i++) {
    DatasetRow row{};
    if (file.read(reinterpret_cast<uint8_t *>(&row), sizeof(row)) !=
        sizeof(row)) {
      printError("Error reading dataset row!");
      file.close();
      return false;
    }
    time_array[i] = row.time_ms;
    value_array[i] = row.value_mA;
    sol_enable_array[i] = row.enable;
    trig_enable_array[i] = row.trigger;
  }

  incomingCount = static_cast<int>(header.count);
  dataIndex = static_cast<int>(header.count);
  datasetDuration = static_cast<int>(header.duration_ms);
  file.close();
  return true;
}

bool restorePressureFromFlash() {
  loadPersistentState();
  if (pressureInitializedFromFlash) {
    float current =
        pressureBarToCurrent(lastPressure_bar, TANK_PRESS_CALIBRATION);
    pressure.set_mA(current);
    return true;
  }
  return false;
}

void startRunSession() {
  // Clean up old session files so new runs can overwrite them
  for (uint32_t i = 1; i <= lastSessionCount; i++) {
    char filename[RUN_LOG_FILENAME_LENGTH];
    snprintf(filename, sizeof(filename), RUN_LOG_FILENAME_FORMAT,
             static_cast<unsigned long>(i));
    if (fatfs.exists(filename)) {
      fatfs.remove(filename);
    }
  }
  // Reset per-session counters and filename tracking
  runCounter = 0;
  lastSavedFilename[0] = '\0';
  lastSessionCount = 0;
  savePersistentState();
}

void saveToFlash() {
  // Advance run index and build the filename for this run
  runCounter++;
  snprintf(lastSavedFilename, sizeof(lastSavedFilename),
           RUN_LOG_FILENAME_FORMAT, static_cast<unsigned long>(runCounter));
  // Track how many files exist in the current session
  if (runCounter > lastSessionCount) {
    lastSessionCount = runCounter;
  }
  savePersistentState();

  File file = fatfs.open(lastSavedFilename, FILE_WRITE);

  if (file) {
    // Add run metadata and logged data rows
    file.printf("run_nr,%lu\n", static_cast<unsigned long>(runCounter));
    file.printf("protocol_version,%u\n", TCM_PROTOCOL_VERSION);
    file.printf("trigger_t0_us,%lu\n",
                static_cast<unsigned long>(runLogTriggerUs));
    // file.println("us,v1 action,v2 set mA,bar"); // Header
    file.println("time_us,sol_valve_action,prop_valve_ma,press_bar"); // Header
    for (int i = 0; i < currentCount; i++) {
      file.printf("%lu,%d,%.2f,%.2f\n", logs[i].timestamp, logs[i].valve1,
                  logs[i].valve2_mA, logs[i].pressure);
    }
    file.close();
    Serial.println("SAVED_TO_FLASH");
  } else {
    printError("Unable to open file for writing!");
  }
  endRunLog();
  currentCount = 0; // Reset RAM log count after saving
}

void dumpToSerial() {
  // Stream the latest run file over serial
  if (strlen(lastSavedFilename) == 0) {
    printError("No flow curve dataset saved yet!");
    return;
  }

  if (!fatfs.exists(lastSavedFilename)) {
    printError("Last flow curve dataset file not found in flash!");
    return;
  }

  File file = fatfs.open(lastSavedFilename, FILE_READ);
  if (file) {
    // Announce the file for host parsing
    Serial.print("START_OF_FILE ");
    Serial.println(lastSavedFilename);

    // Print file contents line by line
    while (file.available()) {
      Serial.write(file.read());
    }

    // Announce end of file for host parsing
    Serial.println("END_OF_FILE");
    file.close();
  } else {
    printError("Unable to open file for reading!");
  }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void setup() {
  // Basic pin modes and safe default states
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_VALVE, OUTPUT);
  pinMode(PIN_TANK_CS_RCLICK, INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_LASER, OUTPUT);
  pinMode(PIN_LIGHT, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_NEB, OUTPUT);
  pinMode(PIN_PDA, INPUT);

  // Set all outputs to safe initial state (off)
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_VALVE, LOW);
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LASER, LOW);
  digitalWrite(PIN_LIGHT, LOW);
  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_NEB, LOW);

  // Initialize T Clicks (proportional valve and pressure regulator)
  valve.begin();
  valve.set_mA(DEF_CURR_VALVE_MA);

  pressure.begin();

  // Initialize DotStar LED
  led.begin();
  led.setBrightness(LED_BRIGHTNESS);
  led.show(); // Initialize all pixels to 'off'

  // Initialize serial communication at 115200 baud
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT_MS);

  // Set ADC resolution for photodetector
  analogReadResolution(ADC_RESOLUTION_BITS);

  // Initialize pressure sensor
  tank_RClick.begin();
  neb_RClick.begin();

  // Initialize the SHT4x temperature & humidity sensor
  if (!sht4.begin()) {
    printError("Failed to find SHT4x sensor!");
  }

  // Configure SHT4x for high precision, no heater
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);

  // Show idle color to indicate system is ready
  setLedColor(COLOR_IDLE);

  // Initialize flash and filesystem
  if (!flash.begin()) {
    printError("Flash initialization failed!");
  }
  DEBUG_PRINTLN("Flash initialized.");

  // Mount the filesystem
  if (!fatfs.begin(&flash)) {
    DEBUG_PRINTLN("Flash chip could also not be mounted, trying to format...");

    if (!flash.eraseChip()) {
      printError("Failed to erase chip!");
    }

    // Try mounting again
    if (!fatfs.begin(&flash)) {
      printError("Still cannot mount filesystem!");
    }
  }
  DEBUG_PRINTLN("Flash filesystem mounted successfully.");

  // Reset run counters on boot; pressure persists
  runCounter = 0;
  lastSessionCount = 0;
  lastSavedFilename[0] = '\0';
  if (!restorePressureFromFlash()) {
    // Fall back to default only when no persisted value was loaded
    pressure.set_mA(DEF_PRESSURE);
  }
  controllerState.pressureConfigured = pressureInitializedFromFlash;

  // Restore last loaded dataset if available
  if (!loadDatasetFromFlash()) {
    resetDataArrays();
  }
}

// ============================================================================
// VALVE AND SENSOR FUNCTIONS
// ============================================================================

void openSolValve() {
  // Open solenoid valve using direct PORT register access for speed
  // Equivalent to digitalWrite(PIN_VALVE, HIGH);
  PORT->Group[g_APinDescription[PIN_VALVE].ulPort].OUTSET.reg =
      (1 << g_APinDescription[PIN_VALVE].ulPin);

  recordEvent(
      1, -1,
      pressureCurrentToBar(tank_RClick.get_EMA_mA(), TANK_PRESS_CALIBRATION));
  // Log valve open event
}

void trigOut() {
  // Send trigger pulse using direct PORT register access for speed
  // Equivalent to digitalWrite(PIN_TRIG, HIGH);
  PORT->Group[g_APinDescription[PIN_TRIG].ulPort].OUTSET.reg =
      (1 << g_APinDescription[PIN_TRIG].ulPin);
}

void closeSolValve() {
  // Close valve using direct PORT register access for speed
  // Equivalent to digitalWrite(PIN_VALVE, LOW);
  PORT->Group[g_APinDescription[PIN_VALVE].ulPort].OUTCLR.reg =
      (1 << g_APinDescription[PIN_VALVE].ulPin);

  recordEvent(
      0, -1,
      pressureCurrentToBar(tank_RClick.get_EMA_mA(), TANK_PRESS_CALIBRATION));
  // Log valve close event

  DEBUG_PRINTLN("SOLENOID_VALVE_CLOSED"); // Valve closed confirmation (debug
                                          // only for speed)
}

void startLaser() {
  // Turn on laser using direct PORT register access for speed
  // Equivalent to digitalWrite(PIN_LASER, HIGH);
  PORT->Group[g_APinDescription[PIN_LASER].ulPort].OUTSET.reg =
      (1 << g_APinDescription[PIN_LASER].ulPin);

  DEBUG_PRINTLN("LASER_ON");
}

void stopLaser() {
  // Turn off laser using direct PORT register access for speed
  // Equivalent to digitalWrite(PIN_LASER, LOW);
  PORT->Group[g_APinDescription[PIN_LASER].ulPort].OUTCLR.reg =
      (1 << g_APinDescription[PIN_LASER].ulPin);

  DEBUG_PRINTLN("LASER_OFF");
}

void stopTrigger() {
  // Turn off trigger using direct PORT register access for speed
  // Equivalent to digitalWrite(PIN_TRIG, LOW);
  PORT->Group[g_APinDescription[PIN_TRIG].ulPort].OUTCLR.reg =
      (1 << g_APinDescription[PIN_TRIG].ulPin);
}

void flashErrorLed() {
  // Flash orange briefly to indicate error
  for (int i = 0; i < ERROR_LED_FLASH_COUNT; i++) {
    setLedColor(COLOR_ERROR);
    delay(ERROR_LED_FLASH_INTERVAL_MS);
    setLedColor(COLOR_OFF);
    delay(ERROR_LED_FLASH_INTERVAL_MS);
  }
  setLedColor(COLOR_IDLE);
}

void printError(const char *message) {
  // Print error message to serial for debugging
  Serial.print("ERROR: ");
  Serial.println(message);
  flashErrorLed();
}

template <typename T> void printError(const char *message, T value) {
  // Print error message with a variable suffix
  Serial.print("ERROR: ");
  Serial.print(message);
  Serial.print(' ');
  Serial.println(value);
  flashErrorLed();
}

void readPressure() {
  // Read current pressure from R-Click sensor and send over serial
  // Conversion formula: Pressure = 0.6249 * I[mA] - 2.4882
  // where I is the 4-20mA current output
  Serial.print("P");
  Serial.println(
      pressureCurrentToBar(tank_RClick.get_EMA_mA(), TANK_PRESS_CALIBRATION));

  DEBUG_PRINT("R Click bitvalue: ");
  DEBUG_PRINTLN(tank_RClick.get_EMA_bitval());
  DEBUG_PRINT("R Click mA: ");
  DEBUG_PRINTLN(tank_RClick.get_EMA_mA());
}

void readTemperatureHumidity(bool valveOpen) {
  // Read temperature and relative humidity from SHT4x sensor
  setLedColor(COLOR_READING); // Show color during reading
  sensors_event_t humidity, temp;
  sht4.getEvent(&humidity, &temp);

  // Send temperature reading
  Serial.print("T");
  Serial.print(temp.temperature);

  // Send humidity reading
  Serial.print(" H");
  Serial.println(humidity.relative_humidity);

  // Restore LED color based on valve state
  setLedColor(valveOpen ? COLOR_VALVE_OPEN : COLOR_IDLE);
}

float readPhotodetector() {
  // Read photodetector voltage with resistor divider compensation
  int adcValue = analogRead(PIN_PDA);
  float voltage = (adcValue / ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE;
  float signalVoltage = voltage * ((PDA_R1 + PDA_R2) / PDA_R2);

  // DEBUG_PRINTLN(signalVoltage);

  return signalVoltage;
}

void resetDataArrays() {
  // Clear all flow curve dataset buffers and indices
  memset(time_array, 0, sizeof(time_array));
  memset(value_array, 0, sizeof(value_array));
  memset(sol_enable_array, 0, sizeof(sol_enable_array));
  memset(trig_enable_array, 0, sizeof(trig_enable_array));
  incomingCount = 0;
  // Keep metadata and execution index in sync with cleared buffers.
  // Prevents stale dataset status after parse failures/clear operations.
  dataIndex = 0;
  sequenceIndex = 0;
  datasetDuration = 0;
}

void clearRunCsvFiles() {
  // Remove experiment CSV files from flash
  File root = fatfs.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      if (!entry.isDirectory()) {
        char name[FLASH_ENTRY_NAME_LENGTH];
        if (entry.getName(name, sizeof(name))) {
          size_t nameLen = strlen(name);
          bool isCsv =
              (nameLen >= 4 && strncmp(name + nameLen - 4, ".csv", 4) == 0);
          bool matchesCurrentPrefix =
              (strncmp(name, RUN_LOG_PREFIX, strlen(RUN_LOG_PREFIX)) == 0);
          if (isCsv && matchesCurrentPrefix) {
            entry.close();
            fatfs.remove(name);
          } else {
            entry.close();
          }
        } else {
          entry.close();
        }
      } else {
        entry.close();
      }
      entry = root.openNextFile();
    }
    root.close();
  }
}

void printStoredFilesDebug() {
  DEBUG_PRINTLN("Stored files:");

  File root = fatfs.open("/");
  if (!root) {
    DEBUG_PRINTLN("  <unable to open root>");
    return;
  }

  bool foundFile = false;
  File entry = root.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      char name[FLASH_ENTRY_NAME_LENGTH];
      if (entry.getName(name, sizeof(name))) {
        foundFile = true;
        DEBUG_PRINT("  ");
        DEBUG_PRINT(name);
        DEBUG_PRINT(" (");
        DEBUG_PRINT((unsigned long)entry.size());
        DEBUG_PRINTLN(" B)");
      }
    }
    entry.close();
    entry = root.openNextFile();
  }

  if (!foundFile) {
    DEBUG_PRINTLN("  <none>");
  }

  root.close();
}

void clearSessionTracking() {
  // Reset session counters and latest filename tracking
  runCounter = 0;
  lastSessionCount = 0;
  lastSavedFilename[0] = '\0';
  currentCount = 0;
  runLogActive = false;
  runLogTriggerUs = 0;
  runLogTriggerSeen = false;
}

void clearPersistentStateAndDataset() {
  // Remove persisted state and dataset files from flash
  if (fatfs.exists(STATE_FILE)) {
    fatfs.remove(STATE_FILE);
  }
  if (fatfs.exists(DATASET_FILE)) {
    fatfs.remove(DATASET_FILE);
  }

  // Reset in-memory persistence tracking (do not alter live outputs)
  pressureInitializedFromFlash = false;
  waitInitializedFromFlash = false;
  lastPressure_bar = 0.0f;
  resetDataArrays();
}

void loadDataset(char *command) {
  // Command: L <N> <duration_ms> <ms>,<mA>,<enable>,<trigger>,...

  // 1. Indicate that the complete command line is being processed.
  setLedColor(COLOR_RECEIVING);

  // 2. Reject a command that has no header or CSV data.
  if (strlen(command) < 3) {
    printError("\"L\" command is not followed by dataset!");
    return;
  }

  // 3. Split the header into the number of rows and total duration in ms.
  char *token = strtok(command + 2, " ");
  if (token == nullptr) {
    printError("L command is missing the dataset row count!");
    resetDataArrays();
    return;
  }
  incomingCount = atoi(token);

  token = strtok(NULL, " ");
  if (token == nullptr) {
    printError("L command is missing the dataset duration!");
    resetDataArrays();
    return;
  }
  datasetDuration = atoi(token);

  // The fixed-size arrays must be able to hold every requested row.
  if (incomingCount > MAX_DATA_LENGTH || incomingCount <= 0) {
    printError("Data length is not allowed: 0 < N <", MAX_DATA_LENGTH);
    resetDataArrays();
    return;
  }

  // 4. Parse each CSV row into the four arrays used during execution.
  dataIndex = 0;
  for (int index = 0; index < incomingCount; index++) {
    // Field 1: timestamp relative to the start of the dataset [ms].
    token = strtok(NULL, ",");
    if (token == NULL) {
      printError("Token was NULL, breaking CSV parsing. Upload new dataset! "
                 "Error at data index ",
                 dataIndex);
      resetDataArrays();
      break;
    }
    time_array[index] = atoi(token);

    // Field 2: proportional-valve current [mA].
    token = strtok(NULL, ",");
    if (token == NULL) {
      printError("Token was NULL, breaking CSV parsing. Upload new dataset! "
                 "Error at data index",
                 dataIndex);
      resetDataArrays();
      break;
    }
    value_array[index] = parseFloatInString(token, 0);

    // Field 3: solenoid state, restricted to the binary values 0 and 1.
    token = strtok(NULL, ",");
    if (token == NULL) {
      printError("Token was NULL, breaking CSV parsing. Upload new dataset! "
                 "Error at data index",
                 dataIndex);
      resetDataArrays();
      break;
    }
    int enable = strcmp(token, "0") == 0 ? 0 : strcmp(token, "1") == 0 ? 1 : -1;
    if (enable == -1) {
      printError("Enable flag must be 0 or 1. Upload new dataset! Error at "
                 "data index",
                 dataIndex);
      resetDataArrays();
      break;
    }
    sol_enable_array[index] = static_cast<uint8_t>(enable);

    // Field 4: trigger event, also restricted to the binary values 0 and 1.
    token = strtok(NULL, ",");
    if (token == NULL) {
      printError("Token was NULL, breaking CSV parsing. Upload new dataset! "
                 "Error at data index",
                 dataIndex);
      resetDataArrays();
      break;
    }
    int trigger = strcmp(token, "0") == 0   ? 0
                  : strcmp(token, "1") == 0 ? 1
                                            : -1;
    if (trigger == -1) {
      printError("Trigger flag must be 0 or 1. Upload new dataset! Error at "
                 "data index",
                 dataIndex);
      resetDataArrays();
      break;
    }
    trig_enable_array[index] = static_cast<uint8_t>(trigger);

    // Emit the accepted row only when debug output is enabled.
    DEBUG_PRINT("Timestamp: ");
    DEBUG_PRINT(time_array[index]);
    DEBUG_PRINT(", mA: ");
    DEBUG_PRINT(value_array[index]);
    DEBUG_PRINT(", enable: ");
    DEBUG_PRINT(sol_enable_array[index]);
    DEBUG_PRINT(", trigger: ");
    DEBUG_PRINTLN(trig_enable_array[index]);
    dataIndex++;
  }

  // 5. Tell the host parsing finished, then persist valid parsed rows for boot
  // recovery. A failed parse leaves dataIndex at zero, so nothing is saved.
  Serial.println("DATASET_RECEIVED");
  if (!saveDatasetToFlash()) {
    printError("Failed to persist dataset to flash!");
  } else {
    Serial.println("DATASET_SAVED");
  }
  // 6. Restore the normal LED indication after success or failure.
  setLedColor(COLOR_OFF);
}

// Enter flow-curve execution from either the immediate R path or the delayed
// droplet-detection path. This resets per-run timing and actuator state so a
// new run cannot inherit an open valve or an active trigger pulse.
// statusMessage is the protocol reply specific to the calling path.
void beginDatasetExecution(const char *statusMessage) {
  controllerState.mode = LoopMode::ExecutingRun;
  beginRunLog();
  runCallTime = micros();
  sequenceIndex = 0;
  controllerState.solValveOpen = false;
  valve.set_mA(DEF_CURR_VALVE_MA);

  if (controllerState.performingTrigger) {
    stopTrigger();
    controllerState.performingTrigger = false;
  }

  setLedColor(COLOR_EXECUTING);
  Serial.println(statusMessage);
}

// Complete an active trigger pulse after its configured width has elapsed.
// This must be serviced every loop iteration to guarantee that the output pin
// is returned LOW even when no dataset row is currently due.
void serviceTriggerPulse() {
  if (controllerState.performingTrigger && micros() - tick >= TRIGGER_WIDTH) {
    stopTrigger();
    controllerState.performingTrigger = false;
  }
}

// Update both R-Click exponential moving averages. The readings are consumed
// by pressure queries and event logging, so this is called every iteration.
void pollPressureSensors() {
  tank_RClick.poll_EMA();
  neb_RClick.poll_EMA();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  /*
   * LOOP ROADMAP
   * -------------------------------------------------------------------------
   * 1) Persistent state and local helpers (lambdas)
   * 2) Fast periodic service (trigger timeout, sensor EMA)
   * 3) Mode processors (laser test, droplet detection, delayed run start)
   * 4) Active flow curve execution
   * 5) Serial command dispatch
   */

  // LoopMode transition guide:
  // - Idle -> DropletDetect: D / D! commands via startDropletDetection(...)
  // - Idle -> DelayBeforeRun: R when pre_trigger_delay_us > 0
  // - Idle -> ExecutingRun: R when pre_trigger_delay_us == 0
  // - Idle -> LaserTest: A 1
  // - DropletDetect -> DelayBeforeRun: droplet detected in D! mode
  // - DropletDetect -> Idle: droplet detected in D mode, PDA fault, or stop
  // - DelayBeforeRun -> ExecutingRun: processDelayedRunStart() after delay
  // - ExecutingRun -> Idle: processDatasetExecution() on run completion
  // - LaserTest -> Idle: A 0 or stopActiveModes(...)
  // - Any mode -> Idle: stopActiveModes(...)

  // Local aliases keep this incremental refactor readable while the helpers
  // below are still lambdas. They will be replaced by named functions next.
  LoopMode &mode = controllerState.mode;
  bool &solValveOpen = controllerState.solValveOpen;
  bool &performingTrigger = controllerState.performingTrigger;
  bool &belowThreshold = controllerState.belowThreshold;
  bool &detectionBaselineReady = controllerState.detectionBaselineReady;
  uint32_t &delayedRunStartTime = controllerState.delayedRunStartTime;
  uint32_t &detectionStartTime = controllerState.detectionStartTime;
  bool &runAfterDropletDetection = controllerState.runAfterDropletDetection;
  bool &setPressure = controllerState.pressureConfigured;
  uint32_t &laserTestLastPrint = controllerState.laserTestLastPrint;

  /* ----------------------------------------------------------------------- */
  /* [HELPER] Arm droplet detection mode                                     */
  /* ----------------------------------------------------------------------- */
  auto startDropletDetection = [&](bool runAfterDetection) -> bool {
    // Responsibility:
    // - Validate prerequisites (dataset + pressure only when runAfterDetection)
    // - Reset run-time actuator state relevant to droplet arming
    // - Set LoopMode::DropletDetect and baseline-tracking flags

    // Validate prerequisites before arming detection
    if (runAfterDetection && dataIndex == 0) {
      printError("Flow curve dataset is empty! Upload first using L command.");
      return false;
    }
    if (runAfterDetection && !setPressure) {
      printError("Pressure regulator not set! Set it first using P command.");
      return false;
    }

    // Ensure clean state before arming detection
    if (solValveOpen) {
      closeSolValve();
      solValveOpen = false;
    }
    valve.set_mA(DEF_CURR_VALVE_MA);
    sequenceIndex = 0;
    runAfterDropletDetection = runAfterDetection;

    // Turn on laser and start timing for detection
    startLaser();
    setLedColor(COLOR_LASER);
    mode = LoopMode::DropletDetect;
    belowThreshold = false;
    detectionBaselineReady = false;
    detectionStartTime = micros();
    DEBUG_PRINTLN("Detecting droplets (primed to cough)");
    return true;
  };

  /* ----------------------------------------------------------------------- */
  /* [HELPER] Stop/clear all active operation modes                          */
  /* ----------------------------------------------------------------------- */
  auto stopActiveModes = [&](bool setIdleLed = true) {
    // Responsibility:
    // - Bring outputs to safe defaults (trigger low, valve default)
    // - Turn off laser when relevant
    // - Reset mode/counters/edge flags that coordinate loop state machines
    // - Optionally restore idle LED indication

    // Centralized teardown for all active operation modes.
    // Used before mode switches and by emergency/stop commands.

    // Ensure trigger pulse is not left active.
    if (performingTrigger) {
      stopTrigger();
      performingTrigger = false;
    }

    // Ensure solenoid and proportional valve return to safe defaults.
    if (solValveOpen) {
      closeSolValve();
      solValveOpen = false;
    }
    valve.set_mA(DEF_CURR_VALVE_MA);

    // Stop any active run/detection/test modes.
    if (mode == LoopMode::DropletDetect || mode == LoopMode::LaserTest) {
      stopLaser();
    }
    mode = LoopMode::Idle;

    // Reset mode-specific control flags/counters.
    sequenceIndex = 0;
    dropletRunsRemaining = 0;
    runAfterDropletDetection = false;
    belowThreshold = false;
    detectionBaselineReady = false;

    if (setIdleLed) {
      setLedColor(COLOR_IDLE);
    }
  };

  /* ----------------------------------------------------------------------- */
  /* [HELPER] Process droplet detection state machine                         */
  /* ----------------------------------------------------------------------- */
  auto processDropletDetection = [&]() -> bool {
    // Responsibility:
    // - Enforce pda_delay dead-time
    // - Detect falling-edge droplet events with baseline initialization
    // - Transition to DelayBeforeRun or re-arm/idle depending on mode/count

    // Returns true when the current loop iteration should exit early
    // (used after hard PDA fault handling).
    if (mode != LoopMode::DropletDetect) {
      return false;
    }

    // Step 1: enforce dead-time after arming/re-arming before reading PDA.
    uint32_t elapsedSinceStart = micros() - detectionStartTime;
    if (elapsedSinceStart < pda_delay) {
      return false;
    }

    // Step 2: acquire the current photodetector voltage.
    float signalVoltage = readPhotodetector();

    // Step 3: fail-safe check. Near-zero voltage means detection is unreliable.
    if (signalVoltage <= PDA_MIN_VALID) {
      printError("PDA signal too low! Check photodetector or laser power.");
      stopLaser();
      mode = LoopMode::Idle;
      belowThreshold = false;
      detectionBaselineReady = false;
      dropletRunsRemaining = 0;
      setLedColor(COLOR_IDLE);
      return true;
    }

    bool signalBelowThreshold = (signalVoltage < PDA_THR);

    // Step 4: initialize edge-tracking baseline once after each arm/re-arm.
    // This prevents immediate retrigger if a droplet is still blocking light.
    if (!detectionBaselineReady) {
      belowThreshold = signalBelowThreshold;
      detectionBaselineReady = true;
      return false;
    }

    // Step 5: detect only a true falling edge (high -> low).
    bool fallingEdgeDetected = (!belowThreshold && signalBelowThreshold);
    if (!fallingEdgeDetected) {
      // Keep edge state synchronized while waiting for next droplet.
      belowThreshold = signalBelowThreshold;
      return false;
    }

    // Step 6: droplet event accepted. Stop laser and publish event.
    belowThreshold = true;
    if (dropletRunsRemaining > 0) {
      dropletRunsRemaining--;
    }

    stopLaser();
    mode = LoopMode::Idle;

    setLedColor(COLOR_DROPLET);
    Serial.println("DROPLET_DETECTED");

    // Step 7: choose post-detection action based on selected mode.
    if (runAfterDropletDetection) {
      // Detect-and-run mode: start pre-run wait timer.
      mode = LoopMode::DelayBeforeRun;
      delayedRunStartTime = micros();
    } else if (dropletRunsRemaining != 0) {
      // Detect-only multi mode: immediately re-arm; pda_delay still applies
      // because processDropletDetection() always waits before sampling.
      if (!startDropletDetection(false)) {
        dropletRunsRemaining = 0;
        setLedColor(COLOR_IDLE);
      }
    } else {
      // Detect-only single/final event: return to idle indication.
      setLedColor(COLOR_IDLE);
    }

    return false;
  };

  /* ----------------------------------------------------------------------- */
  /* [HELPER] Process laser test mode                                        */
  /* ----------------------------------------------------------------------- */
  auto processLaserTest = [&]() {
    if (mode != LoopMode::LaserTest) {
      return;
    }

    uint32_t now_ms = millis();
    if (now_ms - laserTestLastPrint >= LASER_TEST_STREAM_INTERVAL_MS) {
      float signalVoltage = readPhotodetector();
      Serial.print("A");
      Serial.println(signalVoltage, SIGNAL_PRINT_DECIMAL_PLACES);

      if (signalVoltage <= PDA_THR) {
        Serial.println("LASER_TEST_BELOW_THRESHOLD");
      }

      laserTestLastPrint = now_ms;
    }
  };

  /* ----------------------------------------------------------------------- */
  /* [HELPER] Process delayed flow-curve run start                           */
  /* ----------------------------------------------------------------------- */
  auto processDelayedRunStart = [&]() {
    // Responsibility:
    // - Hold execution in DelayBeforeRun until pre-trigger delay has elapsed
    // - Transition mode to ExecutingRun when delay is complete
    // - Initialize run timing/indices and output state for execution start

    // Wait for configured pre-trigger delay before starting flow curve
    if (mode != LoopMode::DelayBeforeRun) {
      return;
    }

    uint32_t elapsed = micros() - delayedRunStartTime;

    if (elapsed < pre_trigger_delay_us) {
      setLedColor(COLOR_WAITING);
      return;
    }

    beginDatasetExecution("EXECUTING_DATASET");
  };

  /* ----------------------------------------------------------------------- */
  /* [HELPER] Process active flow-curve execution                            */
  /* ----------------------------------------------------------------------- */
  auto processDatasetExecution = [&]() -> bool {
    // Responsibility:
    // - Apply due flow-curve points based on elapsed run time
    // - Drive proportional + solenoid valve outputs and trigger pulse state
    // - Finalize run (save + stream logs) and transition back to Idle

    // Returns true when run completion should end this loop iteration.
    if (mode != LoopMode::ExecutingRun) {
      return false;
    }

    // Calculate time since start execution
    uint32_t now = (micros() - runCallTime); // Time since RUN is called [µs]
    uint32_t now_ms = now / DATASET_TIME_SCALE_US_PER_MS;

    // Apply all dataset points that are due
    while (sequenceIndex < dataIndex && now_ms >= time_array[sequenceIndex]) {
      uint8_t enable = sol_enable_array[sequenceIndex];
      uint8_t trigger = trig_enable_array[sequenceIndex];

      // Proportional valve follows mA column regardless of solenoid enable
      valve.set_mA(value_array[sequenceIndex]);
      recordEvent(-1, value_array[sequenceIndex],
                  pressureCurrentToBar(tank_RClick.get_EMA_mA(),
                                       TANK_PRESS_CALIBRATION));

      // Solenoid enable controls only the solenoid valve state
      if (enable && !solValveOpen) {
        openSolValve();
        solValveOpen = true;
        setLedColor(COLOR_VALVE_OPEN);
      } else if (!enable && solValveOpen) {
        closeSolValve();
        solValveOpen = false;
      }

      // Trigger column controls trigger pulses independently of solenoid state
      if (trigger) {
        trigOut();
        markRunTrigger();
        performingTrigger = true;
        tick = micros();
      }

      sequenceIndex++;
    }

    // End condition: dataset duration elapsed AND all points processed
    if (now_ms >= (uint32_t)datasetDuration && sequenceIndex >= dataIndex) {
      valve.set_mA(DEF_CURR_VALVE_MA);
      if (solValveOpen) {
        closeSolValve();
        solValveOpen = false;
      }

      mode = LoopMode::Idle;
      sequenceIndex = 0;
      setLedColor(COLOR_OFF);
      Serial.println("FINISHED");
      // Persist and stream the log for this run
      saveToFlash();
      dumpToSerial();

      // If in multi-run mode, re-arm detection for the next droplet
      if (dropletRunsRemaining != 0) {
        if (!startDropletDetection(true)) {
          dropletRunsRemaining = 0;
        }
      }
      return true;
    }

    return false;
  };

  // =======================================================================
  // LOOP PHASE 1/4: Fast periodic service
  // =======================================================================

  serviceTriggerPulse();
  pollPressureSensors();

  // =======================================================================
  // LOOP PHASE 2/4: Mode processors
  // =======================================================================

  // Laser test mode: keep laser on and stream photodiode voltage
  processLaserTest();

  // Droplet detection monitoring
  if (processDropletDetection()) {
    return;
  }

  // Delayed flow-curve start after droplet detection / R command
  processDelayedRunStart();

  // =======================================================================
  // LOOP PHASE 3/4: Active flow-curve execution
  // =======================================================================

  // Drives the proportional valve, solenoid, and trigger based on the dataset
  if (processDatasetExecution()) {
    return;
  }

  // =======================================================================
  // LOOP PHASE 4/4: Serial command dispatch
  // =======================================================================
  if (sc.available()) {
    // Fetch and decode the latest command line
    char *command =
        sc.getCommand(); // Pointer to memory location of serial buffer contents

    auto armDropletMode = [&](bool runAfterDetection, int32_t requestedCount,
                              bool resetRunSessionFiles) {
      // Step 1: force a clean mode boundary (resets mode flags and outputs).
      stopActiveModes(false);

      // Step 2: persist requested run count in global control state.
      // -1 => continuous, >0 => finite number of droplet triggers.
      dropletRunsRemaining = requestedCount;

      // Step 3: optionally reset per-session run files/counters for modes
      // that execute and log flow-curve runs (D! variants).
      if (resetRunSessionFiles) {
        startRunSession();
      }

      // Step 4: arm droplet detection state machine.
      if (!startDropletDetection(runAfterDetection)) {
        dropletRunsRemaining = 0;
      } else {
        Serial.println("DROPLET_ARMED");
      }
    };

    CommandId commandId = parseCommandId(command);
    switch (commandId) {
    case CommandId::IdQuery:
      Serial.println("TCM_control");
      break;

    case CommandId::ProtocolVersionQuery:
      Serial.print("PROTO ");
      Serial.println(TCM_PROTOCOL_VERSION);
      break;

    case CommandId::DebugToggle: {
      int enable = parseIntInString(command, 1);
      if (enable != 0 && enable != 1) {
        printError("B expects 0 or 1!");
        return;
      }
      debug_enabled = (enable == 1);
      Serial.println(debug_enabled ? "DEBUG_ON" : "DEBUG_OFF");
      break;
    }

    case CommandId::StatusQuery:
      DEBUG_PRINTLN("\n=== System Status ===");
      DEBUG_PRINT("Solenoid valve: ");
      DEBUG_PRINTLN(solValveOpen ? "OPEN" : "CLOSED");
      DEBUG_PRINT("Dataset in memory: ");
      DEBUG_PRINTLN((dataIndex == 0) ? "FALSE" : "TRUE");
      DEBUG_PRINT("Executing dataset: ");
      DEBUG_PRINTLN(mode == LoopMode::ExecutingRun ? "TRUE" : "FALSE");
      DEBUG_PRINT("Trigger: ");
      DEBUG_PRINTLN(performingTrigger ? "ACTIVE" : "IDLE");
      DEBUG_PRINT("Droplet detection: ");
      DEBUG_PRINTLN(mode == LoopMode::DropletDetect ? "ACTIVE" : "IDLE");
      if (mode == LoopMode::DropletDetect) {
        DEBUG_PRINT("Photodetector: ");
        DEBUG_PRINT(readPhotodetector());
        DEBUG_PRINTLN(" V");
      }
      DEBUG_PRINT("Wait before RUN: ");
      DEBUG_PRINT(pre_trigger_delay_us);
      DEBUG_PRINTLN(" µs");
      DEBUG_PRINT("Photodiode detection delay: ");
      DEBUG_PRINT(pda_delay);
      DEBUG_PRINTLN(" µs");
      DEBUG_PRINT("Droplet runs remaining: ");
      DEBUG_PRINTLN(dropletRunsRemaining);
      DEBUG_PRINT("Pressure (raw): ");
      DEBUG_PRINT(tank_RClick.get_EMA_mA());
      DEBUG_PRINT("Pressure (bar): ");
      DEBUG_PRINTLN(pressureCurrentToBar(tank_RClick.get_EMA_mA(),
                                         TANK_PRESS_CALIBRATION));
      DEBUG_PRINTLN(" mA");
      DEBUG_PRINT("Uptime: ");
      DEBUG_PRINT(millis() / 1000);
      DEBUG_PRINTLN(" s");
      printStoredFilesDebug();
      break;

    case CommandId::Help:
      // If debug is off, print an error
      if (!debug_enabled) {
        printError("Help menu is only available when debug output is enabled! "
                   "Enable with B 1 command.");
        return;
      }

      DEBUG_PRINTLN("\n=== Available Commands ===");
      DEBUG_PRINTLN("[Connection & Debugging]");
      DEBUG_PRINTLN("id?     - Show device ID for auto serial connection");
      DEBUG_PRINTLN("ver?    - Show protocol version");
      DEBUG_PRINTLN("B <0|1> - Toggle debug output");
      DEBUG_PRINTLN("S?      - Show system status (debug only)");
      DEBUG_PRINTLN("?       - Show the on-device help menu");
      DEBUG_PRINTLN("[Control Hardware]");
      DEBUG_PRINTLN("V <mA>  - Set proportional valve current in mA");
      DEBUG_PRINTLN("P <bar> - Set tank pressure in bar");
      DEBUG_PRINTLN("M <bar> - Set nebuliser pressure in bar");
      DEBUG_PRINTLN("O       - Open solenoid valve");
      DEBUG_PRINTLN("C       - Close solenoid valve");
      DEBUG_PRINTLN("I <0..1> - Set light level (pin 5, normalized PWM)");
      DEBUG_PRINTLN("G       - Send one trigger pulse now");
      DEBUG_PRINTLN("A <0|1> - Laser test mode off/on (streams photodiode "
                    "readings when on)");
      DEBUG_PRINTLN("F <val> - Set fan speed (pin 3, not yet implemented)");
      DEBUG_PRINTLN("N <0|1> - Nebuliser off/on (pin A3)");
      DEBUG_PRINTLN("Q       - Quit active modes and return to idle");
      DEBUG_PRINTLN("[Read Out Sensors]");
      DEBUG_PRINTLN("P?      - Read current pressure (bar)");
      DEBUG_PRINTLN("M?      - Read current nebuliser pressure (bar)");
      DEBUG_PRINTLN("T?      - Read temperature & humidity");
      DEBUG_PRINTLN("[Configuration]");
      DEBUG_PRINTLN("W <us>  - Set wait before run in microseconds");
      DEBUG_PRINTLN("W?      - Read current wait before run in microseconds");
      DEBUG_PRINTLN("X       - Delete logged CSV files (experiment_log_*.csv)");
      DEBUG_PRINTLN("X!      - X + clear persisted state and dataset");
      DEBUG_PRINTLN("[Flow curve dataset Handling]");
      DEBUG_PRINTLN("L <N> <duration_ms> <csv> - Load flow curve. CSV format: "
                    "<ms0>,<mA0>,<e0>,<t0>,<ms1>,<mA1>,<e1>,<t1>,...,<msN>,<"
                    "mAN>,<eN>,<tN>");
      DEBUG_PRINTLN("         where e=solenoid enable (0/1), t=trigger event "
                    "(0/1), and trigger pulse width is fixed in firmware");
      DEBUG_PRINTLN("L?      - Show loaded flow curve status");
      DEBUG_PRINTLN("[Cough]");
      DEBUG_PRINTLN("R       - Run the loaded flow curve dataset");
      DEBUG_PRINTLN("D       - Droplet-detect only (cont.)");
      DEBUG_PRINTLN("D <n>   - Droplet-detect only n times then stop");
      DEBUG_PRINTLN("D!      - Droplet-detect then run flow curve (cont.)");
      DEBUG_PRINTLN("D! <n>  - Droplet-detect then run flow curve n times");
      break;

    case CommandId::WaitSet:
      pre_trigger_delay_us = parseIntInString(command, 1);
      waitInitializedFromFlash = true;
      savePersistentState();
      DEBUG_PRINT("Pre-trigger wait: ");
      DEBUG_PRINT(pre_trigger_delay_us);
      DEBUG_PRINTLN(" µs");
      Serial.print("SET_WAIT ");
      Serial.println(pre_trigger_delay_us);
      break;

    case CommandId::WaitQuery:
      Serial.print("W");
      Serial.println(pre_trigger_delay_us);
      break;

    case CommandId::ClearMemory:
      clearRunCsvFiles();
      clearPersistentStateAndDataset();
      clearSessionTracking();
      Serial.println("MEMORY_CLEARED");
      break;

    case CommandId::ClearLogs:
      clearRunCsvFiles();
      clearSessionTracking();
      Serial.println("LOGS_CLEARED");
      break;

    case CommandId::SetValve: {
      // Command: V <mA>
      float current = parseFloatInString(command, 1);
      if (!current || current < MIN_CURR_VALVE_MA || current > MAX_CURR_MA) {
        printError("Valve mA input out of range!");
      } else {
        valve.set_mA(current);
        DEBUG_PRINT("Last set bitvalue of proportional valve: ");
        DEBUG_PRINTLN(valve.get_last_set_bitval());
        Serial.print("SET_VALVE ");
        Serial.println(current, 2);
      }
      break;
    }

    case CommandId::SetTankPressure: {
      // Command: P <bar>
      // Step 1: mark pressure as user-configured so runs can proceed.
      if (!setPressure) {
        setPressure = true;
      }

      // Step 2: parse user target and convert to regulator current.
      float bar = parseFloatInString(command, 1);
      float current = pressureBarToCurrent(bar, TANK_PRESS_CALIBRATION);

      // Step 3: validate current range before touching hardware.
      if (!current || current < MIN_CURR_PRESS_REG_MA ||
          current > MAX_CURR_MA) {
        printError("Pressure input out of range!");
      } else {
        // Step 4: apply output and persist equivalent bar setpoint.
        pressure.set_mA(current);
        lastPressure_bar = bar;
        pressureInitializedFromFlash = true;
        savePersistentState();
        DEBUG_PRINT("Last set bitvalue of pressure regulator: ");
        DEBUG_PRINTLN(pressure.get_last_set_bitval());
        Serial.print("SET_PRESSURE ");
        Serial.println(lastPressure_bar, 2);
      }
      break;
    }

    case CommandId::SetNebPressure: {
      // Command: M <bar>
      float bar = parseFloatInString(command, 1);
      float current = pressureBarToCurrent(bar, NEB_PRESS_CALIBRATION);

      if (!current || current < MIN_CURR_PRESS_REG_MA ||
          current > MAX_CURR_MA) {
        printError("Nebuliser pressure input out of range!");
      } else {
        neb_pressure.set_mA(current);
        Serial.print("SET_NEB_PRESSURE ");
        Serial.println(bar, 2);
      }
      break;
    }

    case CommandId::OpenSolenoid:
      if (!solValveOpen) {
        openSolValve();
        solValveOpen = true;
      }
      setLedColor(COLOR_VALVE_OPEN);
      Serial.println("SOLENOID_OPENED");
      break;

    case CommandId::CloseSolenoid:
      if (solValveOpen) {
        closeSolValve();
        solValveOpen = false;
      }
      setLedColor(mode == LoopMode::ExecutingRun ? COLOR_EXECUTING
                                                 : COLOR_IDLE);
      Serial.println("SOLENOID_CLOSED");
      break;

    case CommandId::Quit:
      stopActiveModes(true);
      Serial.println("RETURNED_TO_IDLE");
      break;

    case CommandId::TriggerOnce:
      trigOut();
      tick = micros();
      performingTrigger = true;
      Serial.println("TRIGGER_PULSE_SENT");
      break;

    case CommandId::LaserTestToggle: {
      // Step 1: parse desired laser-test state.
      int enable = parseIntInString(command, 1);
      if (enable != 0 && enable != 1) {
        printError("A expects 0 or 1!");
        return;
      }

      bool enableLaser = (enable == 1);
      if (enableLaser && mode != LoopMode::LaserTest) {
        // Step 2a: entering laser-test mode from another mode:
        // - clear all active modes/flags
        // - turn laser on
        // - switch loop mode + reset print ticker
        stopActiveModes(false);
        startLaser();
        setLedColor(COLOR_LASER);
        mode = LoopMode::LaserTest;
        laserTestLastPrint = 0;
        Serial.println("LASER_TEST_ON");
      } else if (!enableLaser && mode == LoopMode::LaserTest) {
        // Step 2b: leaving laser-test mode:
        // - turn laser off
        // - return to idle mode/LED
        stopLaser();
        setLedColor(COLOR_IDLE);
        mode = LoopMode::Idle;
        Serial.println("LASER_TEST_OFF");
      } else {
        // Step 2c: no state change requested; report current state.
        Serial.println(mode == LoopMode::LaserTest ? "LASER_TEST_ON"
                                                   : "LASER_TEST_OFF");
      }
      break;
    }

    case CommandId::FanSpeed: {
      // TODO: Implement fan speed control on PIN_FAN (pin 3).
      // Hardware not yet finalised — likely PWM.
      // Placeholder: accepts a speed value but does nothing.
      Serial.println("FAN_SPEED_SET");
      break;
    }

    case CommandId::NebuliserToggle: {
      int enable = parseIntInString(command, 1);
      if (enable != 0 && enable != 1) {
        printError("N expects 0 or 1!");
        return;
      }
      digitalWrite(PIN_NEB, enable == 1 ? HIGH : LOW);
      Serial.println(enable == 1 ? "NEBULISER_ON" : "NEBULISER_OFF");
      break;
    }

    case CommandId::LightToggle: {
      // Command: I <level>
      // level is normalized PWM duty in [0.0, 1.0].
      // Backward compatible: legacy I 0 and I 1 are still valid.
      char *valueStart = command + 1;
      while (*valueStart == ' ') {
        valueStart++;
      }

      if (*valueStart == '\0') {
        printError("I expects a value in [0.0, 1.0]!");
        return;
      }

      char *endPtr = nullptr;
      double normalized = strtod(valueStart, &endPtr);
      while (*endPtr == ' ') {
        endPtr++;
      }

      if (endPtr == valueStart || *endPtr != '\0') {
        printError("I expects a numeric value in [0.0, 1.0]!");
        return;
      }

      if (normalized < LIGHT_LEVEL_MIN || normalized > LIGHT_LEVEL_MAX) {
        printError("Light level out of range! Use 0.0 to 1.0.");
        return;
      }

      uint8_t duty = (uint8_t)(normalized * PWM_MAX_DUTY + 0.5);
      analogWrite(PIN_LIGHT, duty);

      Serial.print("SET_LIGHT ");
      Serial.print((float)normalized, 3);
      Serial.print(" DUTY ");
      Serial.println((int)duty);
      break;
    }

    case CommandId::ReadTankPressure:
      readPressure();
      break;

    case CommandId::ReadNebPressure:
      Serial.print("M");
      Serial.println(
          pressureCurrentToBar(neb_RClick.get_EMA_mA(), NEB_PRESS_CALIBRATION));

      DEBUG_PRINT("R Click bitvalue: ");
      DEBUG_PRINTLN(neb_RClick.get_EMA_bitval());
      DEBUG_PRINT("R Click mA: ");
      DEBUG_PRINTLN(neb_RClick.get_EMA_mA());
      break;

    case CommandId::ReadTempHumidity:
      readTemperatureHumidity(solValveOpen);
      break;

    case CommandId::LoadDataset:
      loadDataset(command);
      break;

    case CommandId::DatasetStatus:
      if (dataIndex == 0) {
        Serial.println("NO_DATASET");
      } else {
        Serial.print("DATASET: ");
        Serial.print(incomingCount);
        Serial.print(" LINES AND ");
        Serial.print(datasetDuration);
        Serial.println(" MS");
      }
      break;

    case CommandId::Run: {
      // Start a single run using the loaded dataset
      if (dataIndex == 0) {
        printError("Dataset is empty! Upload first using L command.");
        setLedColor(COLOR_ERROR);
        delay(300);
        setLedColor(COLOR_OFF);
      } else if (!setPressure) {
        printError("Pressure regulator not set! Set it first using P command.");
      } else {
        // Step 1: clear other active modes and reset mode flags/outputs.
        stopActiveModes(false);

        // Step 2: start fresh run session (file/counter reset).
        startRunSession();

        // Step 3: branch run mode based on pre-trigger delay setting.
        if (pre_trigger_delay_us > 0) {
          // Delayed execution path: remember delay start timestamp.
          mode = LoopMode::DelayBeforeRun;
          delayedRunStartTime = micros();
        } else {
          // Immediate execution path: initialize runtime indices and outputs.
          beginDatasetExecution("STARTING_RUN");
        }
      }
      break;
    }

    case CommandId::DropletRun: {
      // Command: D!
      // Arm droplet detection; upon droplet detection wait W delay and run
      // the currently loaded dataset.
      // Optional count: D! <n>. Without a number: run indefinitely.

      int32_t requestedCount = -1;
      if (!parseDropletRunCount(command, true, requestedCount)) {
        return;
      }

      // For D!: reset run-session files/counters because runs will be logged.
      armDropletMode(true, requestedCount, true);
      break;
    }

    case CommandId::DropletDetect: {
      // Command: D
      // Arm droplet detection only (no dataset run).
      // Optional count: D <n>. Without a number: run indefinitely.

      int32_t requestedCount = -1;
      if (!parseDropletRunCount(command, false, requestedCount)) {
        return;
      }

      // For D: no run-session reset needed because this mode does not execute
      // and log the flow-curve by itself.
      armDropletMode(false, requestedCount, false);

      break;
    }

    case CommandId::Other:
      printError("Unknown command:", command);
      break;
    }
  }
}
