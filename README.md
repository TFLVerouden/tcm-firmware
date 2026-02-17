# Twente Cough Machine Control (MCU)

Firmware for the cough machine controller running on the ItsyBitsy M4. It drives the solenoid valve and proportional valve, handles droplet detection, logs data to QSPI flash, and exposes a serial command interface for experiments.

## Build & Upload

This is a PlatformIO project. Open the project in VS Code with PlatformIO installed, then build and upload using the PlatformIO UI or tasks.

## Serial Command Reference

Commands are ASCII lines terminated by newline (\n). Units are noted per command.

### Connection & Debugging

- `id?`: Show device ID for auto serial connection. Replies `TCM_control`.
- `B <0|1>`: Toggle debug output. Replies `DEBUG_ON` or `DEBUG_OFF`.
- `S?`: Show system status (debug only). Replies a block delimited by `STATUS_BEGIN` and `STATUS_END`.
- `?`: Show the on-device help menu.

### Control Hardware

- `V <mA>`: Set proportional valve current in mA. Replies `SET_VALVE <mA>`.
- `P <bar>`: Set pressure regulator in bar. Replies `SET_PRESSURE <bar>`.
- `O`: Open solenoid valve. Replies `SOLENOID_OPENED`.
- `C`: Close solenoid valve (and stop any run). Replies `SOLENOID_CLOSED`.
- `A <0|1>`: Laser test mode off/on (streams photodiode readings when on). Replies `LASER_TEST_ON` or `LASER_TEST_OFF`.

### Read Out Sensors

- `P?`: Read current pressure (bar). Replies `P<bar>`.
- `T?`: Read temperature & humidity. Replies `T<degC> H<%RH>`.

### Configuration

- `W <us>`: Set wait before run in microseconds. Replies `SET_WAIT <us>`.
- `W?`: Read current wait before run in microseconds. Replies `W<us>`.
- `Q`: Delete logged CSV files matching `experiment_dataset_*.csv`. Replies `LOGS_CLEARED`.
- `Q!`: `Q` plus clear persisted state/dataset files. Replies `MEMORY_CLEARED`.

### Dataset Handling

- `L <N> <duration_ms> <csv>`: Load dataset. CSV format: `<ms0>,<mA0>,<e0>,<ms1>,<mA1>,<e1>,...,<msN>,<mAN>,<eN>`. Replies `DATASET_RECEIVED` and `DATASET_SAVED`.
- `L?`: Show loaded dataset status. Replies `NO_DATASET` or `DATASET: <lines> LINES AND <duration_ms> MS`.

### Cough

- `R`: Run the loaded dataset. Replies `STARTING_RUN` (immediate run). Later replies `STARTING_RUN`, `FINISHED`, and file transfer markers when logs are streamed.
- `D`: Droplet-detect then run dataset once. Replies `DROPLET_ARMED` on success.
- `D <n>`: Droplet-detect `n` times then stop. Replies `DROPLET_ARMED` on success.

## Files

- [src/main.cpp](src/main.cpp) contains the firmware implementation and the on-device help menu.
- [platformio.ini](platformio.ini) contains the board and build configuration.

## Logs

Run logs can be stored in QSPI flash. Serial output can be captured to the [logs](logs/) folder. When logs are streamed, output is wrapped by `START_OF_FILE <filename>` and `END_OF_FILE` markers.

In the file header, the run number (only relevant when doing multi-droplet runs) and the trigger time (in us) are output. The body contains four columns:
1. Time stamp (us)
2. Solenoid valve action (0 if closed at that time stamp, 1 if opened at that time stamp, -1 if unchanged at that time stamp)
3. Proportional valve current (mA, in range 12-20, or -1 if unchanged at that time stamp)
4. Pressure sensor readout (bar)

Example run output:

```text
STARTING_RUN
FINISHED
SAVED_TO_FLASH
START_OF_FILE experiment_dataset_0001.csv
run_nr,1
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
