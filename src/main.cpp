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

#include "Adafruit_SPIFlash.h"
#include "DvG_StreamCommand.h"
#include "MIKROE_4_20mA_RT_Click.h"
#include "SdFat.h"
#include <Adafruit_DotStar.h>
#include <Adafruit_SHT4x.h>
#include <Arduino.h>

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void printError(const char *message);
template <typename T> void printError(const char *message, T value);
void resetDataArrays();

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================
// Debug output is disabled by default and can be enabled via the B command.
bool debug_enabled = false;

// Convenience macros to keep debug logging cheap when disabled
#define DEBUG_PRINT(x)                                                         \
  do {                                                                         \
    if (debug_enabled) {                                                       \
      Serial.print("[");                                                       \
      Serial.print(x);                                                         \
      Serial.print("]");                                                       \
    }                                                                          \
  } while (0)
#define DEBUG_PRINTLN(x)                                                       \
  do {                                                                         \
    if (debug_enabled) {                                                       \
      Serial.print("[");                                                       \
      Serial.print(x);                                                         \
      Serial.println("]");                                                     \
    }                                                                          \
  } while (0)

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
// Hardware pin mapping for the ItsyBitsy M4
const int PIN_VALVE = 7;       // MOSFET gate pin for solenoid valve control
const int PIN_PROP_VALVE = 11; // Chip select for proportional valve
const int PIN_PRES_REG = 10;   // Chip select for pressure regulator
const int PIN_CS_RCLICK = 2;   // Chip select for R-Click pressure sensor (SPI)
const int PIN_TRIG = 9; // Trigger output for peripheral devices synchronization
const int PIN_LASER = 12; // Laser MOSFET gate pin for droplet detection
const int PIN_PDA = A2;   // Analog input from photodetector
// Note: PIN_DOTSTAR_DATA and PIN_DOTSTAR_CLK are already defined in variant.h

// ============================================================================
// FLOW-CURVE DATASET BUFFERS
// ============================================================================
// Buffers used for receiving and storing the uploaded dataset
const int MAX_DATA_LENGTH = 2000; // Max serial dataset size
const uint16_t CMD_BUF_LEN =
    32000;                       // RAM size allocation for Serial buffer size
int incomingCount = 0;           // Declare incoming dataset length globally
char cmd_buf[CMD_BUF_LEN]{'\0'}; // Instantiate empty Serial buffer
uint32_t time_array[MAX_DATA_LENGTH];      // Time dataset
float value_array[MAX_DATA_LENGTH];        // mA dataset
uint8_t sol_enable_array[MAX_DATA_LENGTH]; // 0/1: solenoid enable
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

// INITIALIZE LOGGING ARRAY IN RAM
#define MAX_RECORDS 2000
LogEntry logs[MAX_RECORDS];
int currentCount = 0;
uint32_t runCounter = 0; // Per-session run index (starts at 0 each boot)
char lastSavedFilename[32] = ""; // Latest file saved during this session
float lastPressure_bar = 0.0f;   // Persisted pressure regulator setting [bar]
bool pressureInitializedFromFlash = false; // Valid persisted pressure loaded
bool waitInitializedFromFlash = false;     // Valid persisted wait loaded

// ============================================================================
// TIMING PARAMETERS
// ============================================================================
// Timing constants for trigger, droplet detection, and run scheduling
const uint32_t TRIGGER_WIDTH = 10000; // Trigger pulse width [µs] (10ms)
uint32_t tick = 0;                    // Timestamp for timing events [µs]

uint32_t pre_trigger_delay_us =
    0; // Delay between droplet detection/RUN command and opening valve [µs] ->
       // example value 59500 (outdated)
uint32_t pda_delay =
    10000; // Delay before photodiode starts detecting again [µs]

int32_t dropletRunsRemaining = 0; // -1 = continuous, 0 = idle, >0 = remaining

// This is just another ticker
uint32_t runCallTime = 0; // Time elapsed since "RUN" command [µs]

// ============================================================================
// PRESSURE CONVERSION CALIBRATION
// ============================================================================
// Setpoint conversion (requested pressure [bar] -> regulator current [mA])
const float PRESSURE_SETPOINT_BAR_OFFSET = 2.48821429f;
const float PRESSURE_SETPOINT_BAR_PER_mA = 0.62242857f;

// Sensor readback conversion (measured current [mA] -> pressure [bar])
const float PRESSURE_READBACK_BAR_PER_mA = 0.6151645155919202f;
const float PRESSURE_READBACK_BAR_OFFSET = -2.5128908254187095f;

// ============================================================================
// PERSISTENCE FILE KEYS + SESSION TRACKING
// ============================================================================
uint32_t lastSessionCount = 0;
const char *STATE_FILE = "run_state.txt";       // Stores persistent settings
const char *DATASET_FILE = "dataset_state.bin"; // Stores last loaded flow curve

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================
// Pressure sensor (4-20mA R-Click) with exponential moving average filtering
const uint32_t EMA_INTERVAL = 500; // Sampling interval for EMA [µs]
const float EMA_LP_FREQ = 200.;    // Low-pass filter cutoff frequency [Hz]
// Initialize with calibration values: p1_mA, p2_mA, p1_bitval, p2_bitval
R_Click R_click(PIN_CS_RCLICK, RT_Click_Calibration{4.04, 10.98, 806, 2191});

// Temperature & humidity sensor (SHT4x I2C)
Adafruit_SHT4x sht4;

// Photodetector configuration for droplet detection
const float PDA_R1 = 6710.0;     // Voltage divider resistor [Ohm]
const float PDA_R2 = 3260.0;     // Voltage divider resistor [Ohm]
const float PDA_THR = 4.5;       // Droplet detection threshold [V]
const float PDA_MIN_VALID = 0.1; // Minimum valid signal [V] (detect PSU off)

// ============================================================================
// T CLICK CONFIGURATION (proportional valve and pressure regulator)
// ============================================================================
// Proportional valve and pressure regulator interfaces
T_Click valve(PIN_PROP_VALVE, RT_Click_Calibration{3.97, 19.90, 796, 3982});
T_Click pressure(PIN_PRES_REG, RT_Click_Calibration{3.97, 19.90, 796, 3982});

// Define default T Click values
const float max_mA = 20.0;
const float min_mA_valve = 12.0;
const float min_mA_pres_reg = 3.9;
const float default_valve = 12.0;   // mA
const float default_pressure = 4.0; // mA

// ============================================================================
// LED CONFIGURATION
// ============================================================================
// DotStar RGB LED (using board's built-in DotStar on pins 8 and 6)
Adafruit_DotStar led(1, PIN_DOTSTAR_DATA, PIN_DOTSTAR_CLK, DOTSTAR_BGR);

// LED color definitions (avoiding pure red for laser goggle compatibility)
// Colors use BGR format: Blue, Green, Red
const uint32_t COLOR_IDLE = 0x001000;       // Dim green - system ready
const uint32_t COLOR_VALVE_OPEN = 0x00FF00; // Bright green - valve active
const uint32_t COLOR_ERROR = 0xFF4000;      // Orange - error state
const uint32_t COLOR_READING = 0xFF0040;    // Cyan - taking measurement
const uint32_t COLOR_LASER = 0x100000;      // Dim blue - started detection
const uint32_t COLOR_DROPLET = 0xFF0000;    // Bright blue - droplet detected
const uint32_t COLOR_WAITING = 0x400040;   // Purple - waiting for valve opening
const uint32_t COLOR_RECEIVING = 0x100000; // Dim red - receiving flow curve
const uint32_t COLOR_EXECUTING =
    0xFF0000;                        // Bright red - executing loaded flow curve
const uint32_t COLOR_OFF = 0x000000; // Off

// ============================================================================
// LED HELPER FUNCTION
// ============================================================================
void setLedColor(uint32_t color) {
  // Set the DotStar LED to a specific color
  led.setPixelColor(0, color);
  led.show();
}
// ============================================================================
// FUNCTION TO STORE EXECUTION EVENTS IN RAM
// ============================================================================
void recordEvent(int8_t v1, float v2, float press) {
  // Append a log entry if there is space
  if (currentCount < MAX_RECORDS) {
    logs[currentCount] = {micros(), v1, v2, press};
    currentCount++;
  }
}

float pressureBarToCurrent(float bar) {
  // Convert requested pressure [bar] to regulator current [mA]
  // Uses the pressure-regulator setpoint calibration.
  return (bar + PRESSURE_SETPOINT_BAR_OFFSET) / PRESSURE_SETPOINT_BAR_PER_mA;
}

float pressureCurrentToBar(float current_mA) {
  // Convert measured sensor current [mA] to pressure [bar]
  // Uses the pressure-sensor readback calibration.
  return PRESSURE_READBACK_BAR_PER_mA * current_mA +
         PRESSURE_READBACK_BAR_OFFSET;
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

  char line[64];
  while (file.available()) {
    size_t len = file.readBytesUntil('\n', line, sizeof(line) - 1);
    line[len] = '\0';

    // Parse known keys line-by-line (avoid sscanf float parsing)
    const char *pressureKey = "lastPressure_bar=";
    if (strncmp(line, pressureKey, strlen(pressureKey)) == 0) {
      const char *value = line + strlen(pressureKey);
      lastPressure_bar = atof(value);
      float current = pressureBarToCurrent(lastPressure_bar);
      // Validate range before applying
      if (current >= min_mA_pres_reg && current <= max_mA) {
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
  uint32_t count;
  uint32_t duration_ms;
};

struct __attribute__((__packed__)) DatasetRow {
  uint32_t time_ms;
  float value_mA;
  uint8_t enable;
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

  DatasetHeader header{static_cast<uint32_t>(dataIndex),
                       static_cast<uint32_t>(datasetDuration)};
  if (file.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header)) !=
      sizeof(header)) {
    printError("Error writing dataset header!");
    file.close();
    return false;
  }

  for (int i = 0; i < dataIndex; i++) {
    DatasetRow row{time_array[i], value_array[i], sol_enable_array[i]};
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
    float current = pressureBarToCurrent(lastPressure_bar);
    pressure.set_mA(current);
    return true;
  }
  return false;
}

void startRunSession() {
  // Clean up old session files so new runs can overwrite them
  for (uint32_t i = 1; i <= lastSessionCount; i++) {
    char filename[32];
    snprintf(filename, sizeof(filename), "experiment_log_%04lu.csv",
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
           "experiment_log_%04lu.csv", static_cast<unsigned long>(runCounter));
  // Track how many files exist in the current session
  if (runCounter > lastSessionCount) {
    lastSessionCount = runCounter;
  }
  savePersistentState();

  File file = fatfs.open(lastSavedFilename, FILE_WRITE);

  if (file) {
    // Add run metadata and logged data rows
    file.printf("run_nr,%lu\n", static_cast<unsigned long>(runCounter));
    file.printf("trigger_t0_us,%lu\n", tick);
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
  pinMode(PIN_CS_RCLICK, INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_LASER, OUTPUT);
  pinMode(PIN_PDA, INPUT);

  // Set all outputs to safe initial state (off)
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_VALVE, LOW);
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LASER, LOW);

  // Initialize T Clicks (proportional valve and pressure regulator)
  valve.begin();
  valve.set_mA(default_valve);

  pressure.begin();

  // Initialize DotStar LED
  led.begin();
  led.setBrightness(255); // Set brightness to full
  led.show();             // Initialize all pixels to 'off'

  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  Serial.setTimeout(10); // Set timeout to 10ms instead of default 1000ms

  // Set ADC resolution for photodetector
  analogReadResolution(12); // 12-bit ADC (0-4095)

  // Initialize pressure sensor
  R_click.begin();

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
    pressure.set_mA(default_pressure);
  }
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

  recordEvent(1, -1, pressureCurrentToBar(R_click.get_EMA_mA()));
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

  recordEvent(0, -1, pressureCurrentToBar(R_click.get_EMA_mA()));
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
  for (int i = 0; i < 5; i++) {
    setLedColor(COLOR_ERROR);
    delay(300);
    setLedColor(COLOR_OFF);
    delay(300);
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

void readPressure(bool valveOpen) {
  // Read current pressure from R-Click sensor and send over serial
  // Conversion formula: Pressure = 0.6249 * I[mA] - 2.4882
  // where I is the 4-20mA current output
  setLedColor(COLOR_READING); // Show color during reading
  Serial.print("P");
  Serial.println(pressureCurrentToBar(R_click.get_EMA_mA()));

  // Restore LED color based on valve state
  setLedColor(valveOpen ? COLOR_VALVE_OPEN : COLOR_IDLE);
  DEBUG_PRINT("R Click bitvalue: ");
  DEBUG_PRINTLN(R_click.get_EMA_bitval());
  DEBUG_PRINT("R Click mA: ");
  DEBUG_PRINTLN(R_click.get_EMA_mA());
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
  int adcValue = analogRead(PIN_PDA);        // 0-4095 (12-bit)
  float voltage = (adcValue / 4095.0) * 3.3; // Convert to voltage
  float signalVoltage = voltage * ((PDA_R1 + PDA_R2) / PDA_R2);

  // DEBUG_PRINTLN(signalVoltage);

  return signalVoltage;
}

void resetDataArrays() {
  // Clear all flow curve dataset buffers and indices
  memset(time_array, 0, sizeof(time_array));
  memset(value_array, 0, sizeof(value_array));
  memset(sol_enable_array, 0, sizeof(sol_enable_array));
  incomingCount = 0;
  // Todo: Review if these resets are necessary
  // Added these three resets after testing, need reviewing!
  dataIndex = 0;
  sequenceIndex = 0;
  datasetDuration = 0;
}

void clearRunCsvFiles() {
  // Remove experiment CSV files from flash
  const char *csvPrefix = "experiment_dataset_";
  File root = fatfs.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      if (!entry.isDirectory()) {
        char name[64];
        if (entry.getName(name, sizeof(name))) {
          size_t nameLen = strlen(name);
          if (strncmp(name, csvPrefix, strlen(csvPrefix)) == 0 &&
              nameLen >= 4 && strncmp(name + nameLen - 4, ".csv", 4) == 0) {
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

void clearSessionTracking() {
  // Reset session counters and latest filename tracking
  runCounter = 0;
  lastSessionCount = 0;
  lastSavedFilename[0] = '\0';
  currentCount = 0;
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

  // Persistent loop state (retained across iterations)
  enum class LoopMode : uint8_t {
    Idle,
    DropletDetect,
    DelayBeforeRun,
    ExecutingRun,
    LaserTest
  };

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

  static LoopMode mode = LoopMode::Idle;
  static bool solValveOpen = false;      // Tracks if valve is currently open
  static bool performingTrigger = false; // Tracks if trigger pulse is active
  static bool belowThreshold = false;    // Tracks if signal is below threshold
  static bool detectionBaselineReady =
      false; // True after first valid post-delay sample is captured
  static uint32_t delayedRunStartTime = 0; // When delay waiting started [µs]
  static uint32_t detectionStartTime =
      0; // When laser/detection was started [µs]
  static bool runAfterDropletDetection =
      false; // True: detect then run flow curve
  static bool setPressure =
      pressureInitializedFromFlash; // Tracks if pressure regulator has been set
  static uint32_t laserTestLastPrint = 0; // Last photodiode print time [ms]

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
    valve.set_mA(default_valve);
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
    valve.set_mA(default_valve);

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
    if (now_ms - laserTestLastPrint >= 50) {
      float signalVoltage = readPhotodetector();
      Serial.print("A");
      Serial.println(signalVoltage, 3);

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

    mode = LoopMode::ExecutingRun;
    runCallTime = micros();

    // Initialise dataset execution variables
    sequenceIndex = 0;
    solValveOpen = false;
    valve.set_mA(default_valve);

    setLedColor(COLOR_EXECUTING);
    Serial.println("EXECUTING_DATASET");
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
    uint32_t now_ms = now / 1000;

    // Apply all dataset points that are due
    while (sequenceIndex < dataIndex && now_ms >= time_array[sequenceIndex]) {
      uint8_t enable = sol_enable_array[sequenceIndex];

      // Proportional valve follows mA column regardless of solenoid enable
      valve.set_mA(value_array[sequenceIndex]);
      recordEvent(-1, value_array[sequenceIndex],
                  pressureCurrentToBar(R_click.get_EMA_mA()));

      // Solenoid enable controls solenoid and trigger
      if (enable && !solValveOpen) {
        trigOut();
        openSolValve(); // This gives ~5 us delay
        performingTrigger = true;
        solValveOpen = true;
        tick = micros();
        setLedColor(COLOR_VALVE_OPEN);
      } else if (!enable && solValveOpen) {
        closeSolValve();
        solValveOpen = false;
      }

      sequenceIndex++;
    }

    // End condition: dataset duration elapsed AND all points processed
    if (now_ms >= (uint32_t)datasetDuration && sequenceIndex >= dataIndex) {
      valve.set_mA(default_valve);
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

  // Handle trigger pulse timing
  // The trigger pulse is a short signal sent to peripheral devices when the
  // valve opens. It turns off after TRIGGER_WIDTH microseconds.
  if (performingTrigger && (micros() - tick >= TRIGGER_WIDTH)) {
    stopTrigger();
    performingTrigger = false;
  }

  // Pressure sensor must be called regularly to maintain exp. moving average
  R_click.poll_EMA();

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

    enum class CommandId : uint8_t {
      IdQuery,
      DebugToggle,
      StatusQuery,
      Help,
      SetValve,
      SetPressure,
      OpenSolenoid,
      CloseSolenoid,
      LaserTestToggle,
      ReadPressure,
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

    auto classifyCommand = [&](const char *cmd) -> CommandId {
      // Preserve matching precedence for overlapping prefixes
      if (strncmp(cmd, "id?", 3) == 0)
        return CommandId::IdQuery;
      if (strncmp(cmd, "S?", 2) == 0)
        return CommandId::StatusQuery;
      if (strncmp(cmd, "P?", 2) == 0)
        return CommandId::ReadPressure;
      if (strncmp(cmd, "T?", 2) == 0)
        return CommandId::ReadTempHumidity;
      if (strncmp(cmd, "W?", 2) == 0)
        return CommandId::WaitQuery;
      if (strncmp(cmd, "Q!", 2) == 0)
        return CommandId::ClearMemory;
      if (strncmp(cmd, "L?", 2) == 0)
        return CommandId::DatasetStatus;
      if (strncmp(cmd, "D!", 2) == 0)
        return CommandId::DropletRun;
      if (strncmp(cmd, "B", 1) == 0)
        return CommandId::DebugToggle;
      if (strncmp(cmd, "V", 1) == 0)
        return CommandId::SetValve;
      if (strncmp(cmd, "P", 1) == 0)
        return CommandId::SetPressure;
      if (strncmp(cmd, "O", 1) == 0)
        return CommandId::OpenSolenoid;
      if (strncmp(cmd, "C", 1) == 0)
        return CommandId::CloseSolenoid;
      if (strncmp(cmd, "A", 1) == 0)
        return CommandId::LaserTestToggle;
      if (strncmp(cmd, "W", 1) == 0)
        return CommandId::WaitSet;
      if (strncmp(cmd, "Q", 1) == 0)
        return CommandId::ClearLogs;
      if (strncmp(cmd, "L", 1) == 0)
        return CommandId::LoadDataset;
      if (strncmp(cmd, "R", 1) == 0)
        return CommandId::Run;
      if (strncmp(cmd, "D", 1) == 0)
        return CommandId::DropletDetect;
      if (strncmp(cmd, "?", 1) == 0)
        return CommandId::Help;
      return CommandId::Other;
    };

    auto parseDropletRunCount = [&](const char *cmd, bool runAfterDetection,
                                    int32_t &requestedCount) -> bool {
      // Shared parser for D / D! count semantics:
      // - No explicit number => continuous mode (-1)
      // - Explicit number must be >= 1
      requestedCount = -1;

      // D has 1-char prefix, D! has 2-char prefix
      const size_t prefixLen = runAfterDetection ? 2 : 1;
      if (strlen(cmd) > prefixLen) {
        requestedCount = parseIntInString(cmd, prefixLen);
        if (requestedCount <= 0) {
          if (runAfterDetection) {
            printError("D! count must be >= 1! To run indefinitely, send D! "
                       "without a number.");
          } else {
            printError("D count must be >= 1! To run indefinitely, send D "
                       "without a number.");
          }
          return false;
        }
      }
      return true;
    };

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

    CommandId commandId = classifyCommand(command);
    switch (commandId) {
    case CommandId::IdQuery:
      Serial.println("TCM_control");
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
      DEBUG_PRINT(R_click.get_EMA_mA());
      DEBUG_PRINT("Pressure (bar): ");
      DEBUG_PRINTLN(pressureCurrentToBar(R_click.get_EMA_mA()));
      DEBUG_PRINTLN(" mA");
      DEBUG_PRINT("Uptime: ");
      DEBUG_PRINT(millis() / 1000);
      DEBUG_PRINTLN(" s");
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
      DEBUG_PRINTLN("B <0|1> - Toggle debug output");
      DEBUG_PRINTLN("S?      - Show system status (debug only)");
      DEBUG_PRINTLN("?       - Show the on-device help menu");
      DEBUG_PRINTLN("[Control Hardware]");
      DEBUG_PRINTLN("V <mA>  - Set proportional valve current in mA");
      DEBUG_PRINTLN("P <bar> - Set pressure regulator in bar");
      DEBUG_PRINTLN("O       - Open solenoid valve");
      DEBUG_PRINTLN("C       - Close solenoid valve (and stop any run)");
      DEBUG_PRINTLN("A <0|1> - Laser test mode off/on (streams photodiode "
                    "readings when on)");
      DEBUG_PRINTLN("[Read Out Sensors]");
      DEBUG_PRINTLN("P?      - Read current pressure (bar)");
      DEBUG_PRINTLN("T?      - Read temperature & humidity");
      DEBUG_PRINTLN("[Configuration]");
      DEBUG_PRINTLN("W <us>  - Set wait before run in microseconds");
      DEBUG_PRINTLN("W?      - Read current wait before run in microseconds");
      DEBUG_PRINTLN(
          "Q       - Delete logged CSV files (experiment_dataset_*.csv)");
      DEBUG_PRINTLN("Q!      - Q + clear persisted state and dataset");
      DEBUG_PRINTLN("[Flow curve dataset Handling]");
      DEBUG_PRINTLN("L <N> <duration_ms> <csv> - Load flow curve. CSV format: "
                    "<ms0>,<mA0>,<e0>,<ms1>,<mA1>,<e1>,...,<msN>,<mAN>,<eN>");
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
      if (!current || current < min_mA_valve || current > max_mA) {
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

    case CommandId::SetPressure: {
      // Command: P <bar>
      // Step 1: mark pressure as user-configured so runs can proceed.
      if (!setPressure) {
        setPressure = true;
      }

      // Step 2: parse user target and convert to regulator current.
      float bar = parseFloatInString(command, 1);
      float current = pressureBarToCurrent(bar);

      // Step 3: validate current range before touching hardware.
      if (!current || current < min_mA_pres_reg || current > max_mA) {
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

    case CommandId::OpenSolenoid:
      if (!solValveOpen) {
        openSolValve();
        solValveOpen = true;
      }
      setLedColor(COLOR_VALVE_OPEN);
      Serial.println("SOLENOID_OPENED");
      break;

    case CommandId::CloseSolenoid:
      stopActiveModes(true);
      Serial.println("SOLENOID_CLOSED");
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

    case CommandId::ReadPressure:
      readPressure(solValveOpen);
      break;

    case CommandId::ReadTempHumidity:
      readTemperatureHumidity(solValveOpen);
      break;

    case CommandId::LoadDataset: {
      // Parse incoming dataset. Command: "L <N_datapoints>
      // <Time0>,<mA0>,<E0>,<Time1>,<mA1>,<E1>,...,<TimeN>,<mAN>,<EN>"
      // where E is 0/1 solenoid enable.

      setLedColor(COLOR_RECEIVING);

      const char *delim = ","; // Serial dataset delimiter

      if (strlen(command) < 3) {
        printError("\"L\" command is not followed by dataset!");
        return;
      }

      // Step 1: parse header metadata (line count + total duration).
      // read dataset length from char 2 until space (_)
      // "L_<length>_<dataset>" and instantialize position to start reading
      // data from in strtok
      char *idx = strtok(command + 2, " ");
      incomingCount = atoi(idx); // Dataset length (int)

      idx = strtok(NULL, " ");
      datasetDuration = atoi(idx); // Dataset duration

      // Check if data length is acceptable
      if (incomingCount > MAX_DATA_LENGTH || incomingCount <= 0) {
        printError("Data length is not allowed: 0 < N <", MAX_DATA_LENGTH);
        resetDataArrays();
        return;
      }

      dataIndex = 0; // Used later to only read valuable data from data arrays

      // Step 2: parse all CSV triples into runtime arrays.
      // Parsing rest of the dataset after handshake
      for (int i = 0; i < incomingCount; i++) {

        idx = strtok(NULL, delim); // Get next item from buffer (str_cmd). This
                                   // item is the timestamp
        // If the item is NULL, break
        if (idx == NULL) {
          printError("Token was NULL, breaking CSV parsing. Upload new "
                     "dataset! Error at data index ",
                     dataIndex);
          resetDataArrays();
          break;
        }
        // Convert incoming csv buffer index from string to int and add to time
        // array
        time_array[i] = atoi(idx);

        idx = strtok(
            NULL,
            delim); // Get next csv buffer index. This item is the mA value
        // Check again if item is not NULL
        if (idx == NULL) {
          printError("Token was NULL, breaking CSV parsing. Upload new "
                     "dataset! Error at data index",
                     dataIndex);
          resetDataArrays();
          break;
        }
        // Convert incoming csv buffer index from string to float and add to
        // value array
        value_array[i] = parseFloatInString(idx, 0);

        idx = strtok(NULL, delim); // Get next csv buffer index: enable flag
        if (idx == NULL) {
          printError("Token was NULL, breaking CSV parsing. Upload new "
                     "dataset! Error at data index",
                     dataIndex);
          resetDataArrays();
          break;
        }

        int enableInt = atoi(idx);
        if (enableInt != 0 && enableInt != 1) {
          printError("Enable flag must be 0 or 1. Upload new dataset! Error at "
                     "data index",
                     dataIndex);
          resetDataArrays();
          break;
        }
        sol_enable_array[i] = (uint8_t)enableInt;

        // Debug print whole received dataset
        DEBUG_PRINT("Timestamp: ");
        DEBUG_PRINT(time_array[i]);
        DEBUG_PRINT(", mA: ");
        DEBUG_PRINT(value_array[i]);
        DEBUG_PRINT(", enable: ");
        DEBUG_PRINTLN(sol_enable_array[i]);

        // Increase working index, used later to only read valuable data from
        // data arrays
        dataIndex++;
      }

      Serial.println("DATASET_RECEIVED");

      // Step 3: persist parsed dataset to flash for reboot recovery.
      if (!saveDatasetToFlash()) {
        DEBUG_PRINTLN("Failed to persist dataset to flash!");
      } else {
        Serial.println("DATASET_SAVED");
      }

      // LED color off when whole dataset is received
      setLedColor(COLOR_OFF);
      break;
    }

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
          // Immediate execution path: initialize runtime indices/outputs.
          mode = LoopMode::ExecutingRun;
          runCallTime = micros();
          sequenceIndex = 0;
          solValveOpen = false;
          valve.set_mA(default_valve);
          setLedColor(COLOR_EXECUTING);
          Serial.println("STARTING_RUN");
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
