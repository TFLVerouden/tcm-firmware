/*
 * Cough Machine Control System
 *
 * Controls a solenoid valve for atomisation experiments with precise timing.
 * Monitors pressure and environmental conditions.
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
      Serial.println(x);                                                       \
      Serial.print("]");                                                       \
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
// INITIALIZE DVG_STREAMCOMMAND AND FLOW CURVE DATASETS
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
// DATASET PROCESSING & EXECUTION VARIABLES
// ============================================================================
// Indices and counters used during dataset playback
int sequenceIndex = 0;     // Index of dataset to execute on time
int dataIndex = 0;         // Number of datapoints of dataset stored
int datasetDuration = 0.0; // Duration of the uploaded flow profile

// ============================================================================
// SETUP FOR QSPI FLASH FILESYSTEM
// ============================================================================
// QSPI flash storage for logged datasets
// Setup for the ItsyBitsy M4 internal QSPI flash
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
// The filesystem object
FatFileSystem fatfs;

// ============================================================================
// DATASET EXECUTION LOGGING STRUCTURE (IN RAM)
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

// Session tracking for saved files
uint32_t lastSessionCount = 0;
const char *STATE_FILE = "run_state.txt";       // Stores persistent settings
const char *DATASET_FILE = "dataset_state.bin"; // Stores last loaded dataset

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
const float min_mA_pres_reg = 4.0;
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
const uint32_t COLOR_RECEIVING = 0x100000; // Dim red - receiving dataset
const uint32_t COLOR_EXECUTING =
    0xFF0000;                        // Bright red - executing loaded dataset
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
  return (bar + 2.48821429f) / 0.62242857f;
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
// DATASET PERSISTENCE (FLASH)
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
    snprintf(filename, sizeof(filename), "experiment_dataset_%04lu.csv",
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
           "experiment_dataset_%04lu.csv",
           static_cast<unsigned long>(runCounter));
  // Track how many files exist in the current session
  if (runCounter > lastSessionCount) {
    lastSessionCount = runCounter;
  }
  savePersistentState();

  File file = fatfs.open(lastSavedFilename, FILE_WRITE);

  if (file) {
    // Add run metadata and logged data rows
    file.printf("Run,%lu\n", static_cast<unsigned long>(runCounter));
    file.printf("Trigger T0 (us),%lu\n", tick);
    file.println("us,v1 action,v2 set mA,bar"); // Header
    for (int i = 0; i < currentCount; i++) {
      file.printf("%lu,%d,%.2f,%.2f\n", logs[i].timestamp, logs[i].valve1,
                  logs[i].valve2_mA, logs[i].pressure);
    }
    file.close();
    Serial.println("SAVED_TO_FLASH");
  } else {
    DEBUG_PRINTLN("Error opening file for writing!");
  }
  currentCount = 0; // Reset RAM log count after saving
}

void dumpToSerial() {
  // Stream the latest run file over serial
  if (strlen(lastSavedFilename) == 0) {
    DEBUG_PRINTLN("No dataset saved yet!");
    return;
  }

  if (!fatfs.exists(lastSavedFilename)) {
    DEBUG_PRINTLN("Last dataset file not found in flash!");
    return;
  }

  File file = fatfs.open(lastSavedFilename, FILE_READ);
  if (file) {
    // Announce the file and run index for host parsing
    Serial.print("FILE: ");
    Serial.println(lastSavedFilename);
    Serial.print("RUN: ");
    Serial.println(static_cast<unsigned long>(runCounter));
    Serial.println("START_OF_FILE");

    while (file.available()) {
      Serial.write(file.read());
    }

    Serial.println("END_OF_FILE");
    file.close();
  } else {
    DEBUG_PRINTLN("Error opening file for reading!");
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
    printError("Failed to find SHT4x sensor");
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

  recordEvent(1, -1,
              0.62350602 * R_click.get_EMA_mA() -
                  2.51344790); // Log valve open event
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

  recordEvent(0, -1,
              0.62350602 * R_click.get_EMA_mA() -
                  2.51344790); // Log valve close event

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
  Serial.println(0.62350602 * R_click.get_EMA_mA() - 2.51344790);

  // Restore LED color based on valve state
  setLedColor(valveOpen ? COLOR_VALVE_OPEN : COLOR_IDLE);
  DEBUG_PRINT("R Click bitvalue: ");
  DEBUG_PRINTLN(R_click.get_EMA_bitval());
}

void readTemperatureHumidity(bool valveOpen) {
  // Read temperature and relative humidity from SHT4x sensor
  setLedColor(COLOR_READING); // Show color during reading
  sensors_event_t humidity, temp;
  sht4.getEvent(&humidity, &temp);

  // Send temperature reading
  Serial.print("T");
  Serial.println(temp.temperature);

  // Send humidity reading
  Serial.print("H");
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
  // Clear all dataset buffers and indices
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

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Static variables persist across loop iterations
  static bool solValveOpen = false;      // Tracks if valve is currently open
  static bool performingTrigger = false; // Tracks if trigger pulse is active
  static bool detectingDroplet = false;  // Tracks if in droplet detection mode
  static bool belowThreshold = false;    // Tracks if signal is below threshold
  static bool delayedRunPending = false; // Waiting pre-trigger delay before RUN
  static uint32_t delayedRunStartTime = 0; // When delay waiting started [µs]
  static uint32_t detectionStartTime =
      0;                           // When laser/detection was started [µs]
  static bool isExecuting = false; // Tracks if waiting to run loaded sequence
  static bool setPressure =
      pressureInitializedFromFlash; // Tracks if pressure regulator has been set
  static bool laserTestActive = false;    // Laser test mode on/off
  static uint32_t laserTestLastPrint = 0; // Last photodiode print time [ms]

  auto startDropletDetection = [&]() -> bool {
    // Validate prerequisites before arming detection
    if (dataIndex == 0) {
      printError("Dataset is empty! Upload first using L command.");
      return false;
    }
    if (!setPressure) {
      printError("Pressure regulator not set! Set it first using P command.");
      return false;
    }

    // Ensure clean state before arming detection
    if (solValveOpen) {
      closeSolValve();
      solValveOpen = false;
    }
    valve.set_mA(default_valve);
    isExecuting = false;
    sequenceIndex = 0;
    delayedRunPending = false;

    // Turn on laser and start timing for detection
    startLaser();
    setLedColor(COLOR_LASER);
    detectingDroplet = true;
    belowThreshold = false;
    detectionStartTime = micros();
    DEBUG_PRINTLN("Detecting droplets (primed to cough)");
    return true;
  };

  // -------------------------------------------------------------------------
  // Handle trigger pulse timing
  // -------------------------------------------------------------------------
  // The trigger pulse is a short signal sent to peripheral devices when the
  // valve opens. It turns off after TRIGGER_WIDTH microseconds.
  if (performingTrigger && (micros() - tick >= TRIGGER_WIDTH)) {
    stopTrigger();
    performingTrigger = false;
  }

  // -------------------------------------------------------------------------
  // Update pressure sensor
  // -------------------------------------------------------------------------
  // Must be called regularly to maintain the exponential moving average
  R_click.poll_EMA();

  // -------------------------------------------------------------------------
  // Laser test mode: keep laser on and stream photodiode voltage
  // -------------------------------------------------------------------------
  if (laserTestActive) {
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
  }

  // -------------------------------------------------------------------------
  // Droplet detection monitoring
  // -------------------------------------------------------------------------
  // When armed, wait for photodetector delay and watch for threshold crossing
  if (detectingDroplet) {
    uint32_t elapsedSinceStart = micros() - detectionStartTime;

    // Only start checking photodiode after the configured delay
    if (elapsedSinceStart >= pda_delay) {
      float signalVoltage = readPhotodetector();

      // If signal is near zero, assume PDA power is off -> abort detection
      if (signalVoltage <= PDA_MIN_VALID) {
        printError("PDA signal too low; check photodetector or laser power.");
        stopLaser();
        detectingDroplet = false;
        belowThreshold = false;
        delayedRunPending = false;
        dropletRunsRemaining = 0;
        setLedColor(COLOR_IDLE);
        return;
      }

      // Falling edge: droplet detected (signal drops below threshold)
      if (!belowThreshold && signalVoltage < PDA_THR) {
        belowThreshold = true;
        delayedRunPending = true;
        delayedRunStartTime = micros();
        if (dropletRunsRemaining > 0) {
          dropletRunsRemaining--;
        }

        // Turn off laser immediately when droplet is detected
        stopLaser();
        detectingDroplet = false;

        setLedColor(COLOR_DROPLET);
        Serial.println("DROPLET_DETECTED");
      }
    }
  }

  // -------------------------------------------------------------------------
  // Handle delayed dataset start after droplet detection/R command
  // -------------------------------------------------------------------------
  // Wait for the configured pre-trigger delay before starting the dataset
  if (delayedRunPending) {
    uint32_t elapsed = micros() - delayedRunStartTime;

    if (elapsed < pre_trigger_delay_us) {
      setLedColor(COLOR_WAITING);
    } else {
      // Start executing the already-loaded dataset now
      delayedRunPending = false;
      isExecuting = true;
      runCallTime = micros();

      // Initialise dataset execution variables
      sequenceIndex = 0;
      solValveOpen = false;
      valve.set_mA(default_valve);

      setLedColor(COLOR_EXECUTING);
      Serial.println("EXECUTING_DATASET");
    }
  }

  // Drives the proportional valve, solenoid, and trigger based on the dataset
  if (isExecuting) {
    // Calculate time since start execution
    uint32_t now = (micros() - runCallTime); // Time since RUN is called [µs]

    uint32_t now_ms = now / 1000;

    // Apply all dataset points that are due
    while (sequenceIndex < dataIndex && now_ms >= time_array[sequenceIndex]) {
      uint8_t enable = sol_enable_array[sequenceIndex];

      // Proportional valve follows mA column regardless of solenoid enable
      valve.set_mA(value_array[sequenceIndex]);
      recordEvent(-1, value_array[sequenceIndex],
                  0.62350602 * R_click.get_EMA_mA() - 2.51344790);

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

      isExecuting = false;
      sequenceIndex = 0;
      setLedColor(COLOR_OFF);
      Serial.println("FINISHED");
      // Persist and stream the log for this run
      saveToFlash();
      dumpToSerial();

      // If in multi-run mode, re-arm detection for the next droplet
      if (dropletRunsRemaining != 0) {
        if (!startDropletDetection()) {
          dropletRunsRemaining = 0;
        }
      }
      return;
    }
  }

  // =========================================================================
  // Process serial commands
  // =========================================================================
  if (sc.available()) {
    // Fetch and decode the latest command line
    char *command =
        sc.getCommand(); // Pointer to memory location of serial buffer contents

    DEBUG_PRINT("CMD: ");
    DEBUG_PRINTLN(command);

    if (strncmp(command, "B", 1) == 0) {
      // Command: B <0|1>
      // Enable (1) or disable (0) debug output
      int enable = parseIntInString(command, 1);
      debug_enabled = (enable == 1);
      Serial.println(debug_enabled ? "DEBUG_ON" : "DEBUG_OFF");

    } else if (strncmp(command, "V", 1) == 0) {
      // Command: V <mA>
      // Set milli amps of proportional valve to <mA>

      float current = parseFloatInString(
          command, 1); // Parse float from char array 'command'

      // Handle out of allowable range inputs, defaults to specified value
      if (!current || current < min_mA_valve || current > max_mA) {
        printError("Valve mA input out of range!");

        // Set T_Click to input mA
      } else {
        valve.set_mA(current);
        DEBUG_PRINT("Last set bitvalue of proportional valve: ");
        DEBUG_PRINTLN(valve.get_last_set_bitval());
      }

      // ---------------------------------------------------------------------
      // Pressure regulator: P? (read) or P <bar> (set)
      // ---------------------------------------------------------------------
    } else if (strncmp(command, "P?", 2) == 0) {
      // Command: P?
      // Read and return current pressure
      readPressure(solValveOpen);

    } else if (strncmp(command, "P", 1) == 0) {
      // Command: P <bar>
      // Set pressure regulator to <bar>

      // Mark that pressure has been explicitly set
      if (!setPressure) {
        setPressure = true;
      }

      float bar =
          parseFloatInString(command, 1); // Parse float from char array command
      float current = pressureBarToCurrent(bar);

      // Handle out of allowable range inputs, defaults to specified value
      if (!current || current < min_mA_pres_reg || current > max_mA) {
        printError("Pressure input out of range!");

        // Set T_Click to input mA
      } else {
        // Apply pressure, store it, and persist to flash
        pressure.set_mA(current);
        lastPressure_bar = bar;
        pressureInitializedFromFlash = true;
        savePersistentState();
        DEBUG_PRINT("Last set bitvalue of pressure regulator: ");
        DEBUG_PRINTLN(pressure.get_last_set_bitval());
      }

      // ---------------------------------------------------------------------
      // Dataset: L? / L / R / F
      // ---------------------------------------------------------------------
    } else if (strncmp(command, "L?", 2) == 0) {

      if (dataIndex == 0) {
        Serial.println("NO_DATASET");
      } else {
        Serial.print("DATASET: ");
        Serial.print(incomingCount);
        Serial.print(" LINES AND ");
        Serial.print(datasetDuration);
        Serial.println(" MS");
      }

    } else if (strncmp(command, "L", 1) == 0) {
      // Parse incomming dataset. Command: "L <N_datapoints>
      // <Time0>,<mA0>,<E0>,<Time1>,<mA1>,<E1>,...,<TimeN>,<mAN>,<EN>"
      // where E is 0/1 solenoid enable.

      setLedColor(COLOR_RECEIVING);

      const char *delim = ","; // Serial dataset delimiter

      if (strlen(command) < 3) {
        printError("\"L\" command is not followed by dataset");
        return;
      }

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
                     "dataset! Error at data index ",
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
                     "dataset! Error at data index ",
                     dataIndex);
          resetDataArrays();
          break;
        }

        int enableInt = atoi(idx);
        if (enableInt != 0 && enableInt != 1) {
          printError("Enable flag must be 0 or 1. Upload new dataset! Error at "
                     "data index ",
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

      if (!saveDatasetToFlash()) {
        DEBUG_PRINTLN("Failed to persist dataset to flash!");
      }

      // LED color off when whole dataset is received
      setLedColor(COLOR_OFF);

    } else if (strncmp(command, "R", 1) == 0) {
      // Start a single run using the loaded dataset
      if (dataIndex == 0) {
        printError("Dataset is empty! Upload first using L command.");
        setLedColor(COLOR_ERROR);
        delay(300);
        setLedColor(COLOR_OFF);
      } else if (!setPressure) {
        printError("Pressure regulator not set! Set it first using P command.");
      } else {
        // New session: clear old files and reset counters
        startRunSession();
        delayedRunPending = (pre_trigger_delay_us > 0);
        delayedRunStartTime = micros();

        if (!delayedRunPending) {
          isExecuting = true;
          runCallTime = micros();
          sequenceIndex = 0;
          solValveOpen = false;
          valve.set_mA(default_valve);
          setLedColor(COLOR_EXECUTING);
          Serial.println("EXECUTING_DATASET");
        }
      }

      // ---------------------------------------------------------------------
      // Solenoid and droplet detection: O / C / D / W
      // ---------------------------------------------------------------------
    } else if (strncmp(command, "O", 1) == 0) {
      // Command: O
      // Manually open solenoid valve
      if (!solValveOpen) {
        openSolValve();
        solValveOpen = true;
      }
      setLedColor(COLOR_VALVE_OPEN);

    } else if (strncmp(command, "C", 1) == 0) {
      // Command: C
      // Manually close solenoid valve and stop any active run/detection
      if (solValveOpen) {
        closeSolValve();
        solValveOpen = false;
      }
      valve.set_mA(default_valve);
      if (isExecuting) {
        isExecuting = false;
        sequenceIndex = 0;
      }
      if (detectingDroplet) {
        stopLaser();
        detectingDroplet = false;
      }
      delayedRunPending = false;
      dropletRunsRemaining = 0;
      setLedColor(COLOR_IDLE);

    } else if (strncmp(command, "D", 1) == 0) {
      // Command: D
      // Arm droplet detection; upon droplet detection wait W delay and run
      // the currently loaded dataset.
      // Optional count: D <n>. Without a number: run indefinitely.

      int32_t requestedCount = -1;
      if (strlen(command) > 1) {
        requestedCount = parseIntInString(command, 1);
        if (requestedCount <= 0) {
          printError("D count must be >= 1");
          return;
        }
      }

      dropletRunsRemaining = requestedCount;
      // New session: clear old files and reset counters
      startRunSession();
      if (!startDropletDetection()) {
        dropletRunsRemaining = 0;
      }

    } else if (strncmp(command, "W?", 2) == 0) {
      // Command: W?
      // Read and return current pre-trigger wait time
      Serial.print("W");
      Serial.println(pre_trigger_delay_us);

    } else if (strncmp(command, "W", 1) == 0) {
      // Command: W <delay_us>
      // Set delay between droplet detection and starting dataset execution
      pre_trigger_delay_us = parseIntInString(command, 1);
      waitInitializedFromFlash = true;
      savePersistentState();
      DEBUG_PRINT("Pre-trigger wait: ");
      DEBUG_PRINT(pre_trigger_delay_us);
      DEBUG_PRINTLN(" µs");

    } else if (strncmp(command, "A", 1) == 0) {
      // Command: A <0|1>
      // Enable (1) or disable (0) laser test mode; no arg toggles.
      bool enableLaser = !laserTestActive;
      if (strlen(command) > 1) {
        int enable = parseIntInString(command, 1);
        if (enable == 0) {
          enableLaser = false;
        } else if (enable == 1) {
          enableLaser = true;
        } else {
          printError("A expects 0 or 1");
          return;
        }
      }

      if (enableLaser && !laserTestActive) {
        if (detectingDroplet) {
          detectingDroplet = false;
          belowThreshold = false;
          delayedRunPending = false;
        }
        dropletRunsRemaining = 0;
        startLaser();
        setLedColor(COLOR_LASER);
        laserTestActive = true;
        laserTestLastPrint = 0;
      } else if (!enableLaser && laserTestActive) {
        stopLaser();
        setLedColor(COLOR_IDLE);
        laserTestActive = false;
      }

      // ---------------------------------------------------------------------
      // Sensors: T? / S?
      // ---------------------------------------------------------------------
    } else if (strncmp(command, "T?", 2) == 0) {
      // Command: T?
      // Read and return temperature & humidity
      readTemperatureHumidity(solValveOpen);

    } else if (strncmp(command, "id?", 3) == 0) {
      // Command: id?
      // Return device ID for dvg-devices Arduino validation
      Serial.println("TCM_control");

    } else if (strncmp(command, "?", 1) == 0) {
      // Command: ?
      // Print help menu
      DEBUG_PRINTLN("\n=== Available Commands ===");
      DEBUG_PRINTLN("R       - RUN loaded dataset");
      DEBUG_PRINTLN("D       - DROPLET-detect then run dataset");
      DEBUG_PRINTLN("D <n>   - DROPLET-detect n times then stop");
      DEBUG_PRINTLN("W <us>  - Set WAIT before run (µs)");
      DEBUG_PRINTLN("W?      - Read WAIT before run (µs)");
      DEBUG_PRINTLN("P <bar> - Set PRESSURE on tank (bar)");
      DEBUG_PRINTLN("P?      - Read PRESSURE");
      DEBUG_PRINTLN("A <0|1> - LASER test mode off/on");
      DEBUG_PRINTLN("O       - OPEN solenoid valve");
      DEBUG_PRINTLN("C       - CLOSE solenoid valve (and stop any run)");
      DEBUG_PRINTLN("V <mA>  - Set proportional VALVE milliamps to <mA>");
      DEBUG_PRINTLN("L <N_datapoints> <dataset duration (ms)> <csv dataset> "
                    "- LOAD dataset, format: "
                    "<ms0>,<mA0>,<e0>,<ms1>,<mA1>,<e1>,...,<msN>,<mAN>,<eN>");
      DEBUG_PRINTLN("L?      - Show LOADED dataset status");
      DEBUG_PRINTLN("T?      - Read TEMPERATURE & humidity");
      DEBUG_PRINTLN("S?      - System STATUS");
      DEBUG_PRINTLN("B <0|1> - DeBUG output off/on");
      DEBUG_PRINTLN("id?     - Show device ID for auto serial connection");
      DEBUG_PRINTLN("?       - Show this help?");

    } else if (strncmp(command, "S?", 2) == 0) {
      // Command: S?
      // Print system status (debug only)
      DEBUG_PRINTLN("\n=== System Status ===");
      DEBUG_PRINT("Solenoid valve: ");
      DEBUG_PRINTLN(solValveOpen ? "OPEN" : "CLOSED");
      DEBUG_PRINT("Dataset in memory: ");
      DEBUG_PRINTLN((dataIndex == 0) ? "FALSE" : "TRUE");
      DEBUG_PRINT("Executing dataset: ");
      DEBUG_PRINTLN(isExecuting ? "TRUE" : "FALSE");
      DEBUG_PRINT("Trigger: ");
      DEBUG_PRINTLN(performingTrigger ? "ACTIVE" : "IDLE");
      DEBUG_PRINT("Droplet detection: ");
      DEBUG_PRINTLN(detectingDroplet ? "ACTIVE" : "IDLE");
      if (detectingDroplet) {
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
      DEBUG_PRINTLN(0.62350602 * R_click.get_EMA_mA() - 2.51344790);
      DEBUG_PRINTLN(" mA");
      DEBUG_PRINT("Uptime: ");
      DEBUG_PRINT(millis() / 1000);
      DEBUG_PRINTLN(" s");

    } else {
      // Unknown command
      printError("Unknown command");
    }
  }
}
