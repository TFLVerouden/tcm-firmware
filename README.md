# Twente Cough Machine Control (MCU)

Firmware for the cough machine controller running on the [Adafruit ItsyBitsy M4 Express](https://learn.adafruit.com/introducing-adafruit-itsybitsy-m4). It drives the solenoid valve and proportional valve, handles droplet detection, logs data to QSPI flash, and exposes a serial command interface for experiments.

## Hardware

See also the [Twente Cough Machine hardware documentation](https://github.com/TFLVerouden/tcm-hardware/tree/main).

| Function | Pin |
| --- | --- |
| Solenoid valve MOSFET gate | 7 |
| Proportional-valve T-Click chip select | 11 |
| Pressure-regulator T-Click chip select | 10 |
| Pressure-sensor R-Click chip select | 2 |
| Trigger output | 9 |
| Laser MOSFET gate | 12 |
| Light PWM output | 5 |
| Fan PWM output | 3 |
| Nebuliser enable | A3 |
| Photodetector input | A2 |

All controlled outputs start LOW at boot. The pressure regulator restores its last valid saved setting from QSPI flash; when no valid setting is available, it uses the firmware default of 4.0 mA. Pressure conversion and sensor calibration constants are defined in [src/main.cpp](src/main.cpp).

## Build & Upload

This is a PlatformIO project. Open the project in VS Code with PlatformIO installed, then build and upload using the PlatformIO UI or tasks.

## Serial Command Reference

Commands are ASCII lines terminated by newline (\n). Units are noted per command.

### Connection & Debugging

| Command | Description | Reply |
| --- | --- | --- |
| `id?` | Show the device identifier for automatic serial connection. | `TCM_control` |
| `ver?` | Show the serial protocol version. | `PROTO <integer>` |
| `B <0\|1>` | Disable or enable debug output. | `DEBUG_OFF` or `DEBUG_ON` |
| `S?` | Show system status when debug output is enabled. | Status lines |
| `?` | Show the on-device help menu. | Command list |

### Hardware

| Command | Description | Reply |
| --- | --- | --- |
| `V <mA>` | Set proportional-valve current in mA. | `SET_VALVE <mA>` |
| `P <bar>` | Set pressure-regulator target in bar. | `SET_PRESSURE <bar>` |
| `O` | Open the solenoid valve. | `SOLENOID_OPENED` |
| `C` | Close the solenoid valve. | `SOLENOID_CLOSED` |
| `I <level>` | Set light output on pin 5 with normalized PWM from `0.0` to `1.0`. | `SET_LIGHT <level> DUTY <0-255>` |
| `G` | Send one trigger pulse with the firmware-defined width. | `TRIGGER_PULSE_SENT` |
| `A <0\|1>` | Disable or enable laser test mode; enabled mode streams photodiode readings. | `LASER_TEST_OFF` or `LASER_TEST_ON` |
| `F <val>` | Set fan speed on pin 3. The hardware implementation is currently a placeholder. | `FAN_SPEED_SET` |
| `N <0\|1>` | Disable or enable the nebuliser on pin A3. | `NEBULISER_OFF` or `NEBULISER_ON` |
| `Q` | Quit active modes and return to idle. | `RETURNED_TO_IDLE` |

### Sensors

| Command | Description | Reply |
| --- | --- | --- |
| `P?` | Read current pressure in bar. | `P<bar>` |
| `T?` | Read temperature and relative humidity. | `T<degC> H<%RH>` |

### Configuration

| Command | Description | Reply |
| --- | --- | --- |
| `W <us>` | Set the wait before a run or after droplet detection, in microseconds. | `SET_WAIT <us>` |
| `W?` | Read the configured wait before run, in microseconds. | `W<us>` |
| `X` | Delete logged `experiment_log_*.csv` files. | `LOGS_CLEARED` |
| `X!` | Clear run logs plus persisted state and dataset files. | `MEMORY_CLEARED` |

### Dataset

| Command | Description | Reply |
| --- | --- | --- |
| `L <N> <duration_ms> <csv>` | Load a flow curve. Each CSV row is `<ms>,<mA>,<e>,<t>` where `e` is solenoid enable and `t` is a trigger event. | `DATASET_RECEIVED`, then `DATASET_SAVED` |
| `L?` | Show loaded flow-curve status. | `NO_DATASET` or `DATASET: <lines> LINES AND <duration_ms> MS` |

### Cough

| Command | Description | Reply |
| --- | --- | --- |
| `R` | Run the loaded flow curve. | `STARTING_RUN`, `FINISHED`, and log-transfer markers |
| `D` | Arm continuous droplet detection only. | `DROPLET_ARMED`, then `DROPLET_DETECTED` |
| `D <n>` | Detect `n` droplets, then stop. | `DROPLET_ARMED` |
| `D!` | Continuously detect droplets and run the loaded flow curve after each detection. | `DROPLET_ARMED` |
| `D! <n>` | Detect `n` droplets and run the loaded flow curve after each detection. | `DROPLET_ARMED` |

## Files

- [src/main.cpp](src/main.cpp) contains the firmware implementation and the on-device help menu.
- [platformio.ini](platformio.ini) contains the board and build configuration.

## Logs

Run logs can be stored in QSPI flash. Serial output can be captured to the [logs](logs/) folder. When logs are streamed, output is wrapped by `START_OF_FILE <filename>` and `END_OF_FILE` markers.

In the file header, the run number (only relevant when doing multi-droplet runs), protocol version, and trigger time (in us) are output. The body contains four columns:
1. Time stamp (us)
2. Solenoid valve action (0 if closed at that time stamp, 1 if opened at that time stamp, -1 if unchanged at that time stamp)
3. Proportional valve current (mA, in range 12-20, or -1 if unchanged at that time stamp)
4. Pressure sensor readout (bar)

Example run output:

```text
STARTING_RUN
FINISHED
SAVED_TO_FLASH
START_OF_FILE experiment_log_0001.csv
run_nr,1
protocol_version,5
trigger_t0_us,149580895
time_us,sol_valve_action,prop_valve_ma,press_bar
149580861,-1,0.00,1.49
149580890,-1,12.00,1.49
149580894,1,-1.00,1.49
149586728,-1,20.00,1.49
149786723,-1,12.00,1.38
149786727,0,-1.00,1.38
END_OF_FILE
```
