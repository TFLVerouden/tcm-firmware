# Twente Cough Machine Control (MCU)

Firmware for the cough machine controller running on the ItsyBitsy M4. It drives the solenoid valve and proportional valve, handles droplet detection, logs data to QSPI flash, and exposes a serial command interface for experiments.

## Build & Upload

This is a PlatformIO project. Open the project in VS Code with PlatformIO installed, then build and upload using the PlatformIO UI or tasks.

## Serial Command Reference

Commands are ASCII lines terminated by newline (\n). Units are noted per command.

- `B <0|1>`: Toggle debug output.
- `V <mA>`: Set proportional valve current in mA.
- `P <bar>`: Set pressure regulator in bar.
- `P?`: Read current pressure (bar).
- `W <us>`: Set wait before run in microseconds.
- `L <N> <duration_ms> <csv>`: Load dataset. CSV format: `<ms0>,<mA0>,<e0>,<ms1>,<mA1>,<e1>,...,<msN>,<mAN>,<eN>`.
- `L?`: Show loaded dataset status.
- `R`: Run the loaded dataset.
- `D`: Droplet-detect then run dataset once.
- `D <n>`: Droplet-detect `n` times then stop.
- `O`: Open solenoid valve.
- `C`: Close solenoid valve (and stop any run).
- `A`: Toggle laser test mode (stream photodiode readings).
- `T?`: Read temperature & humidity.
- `S?`: Show system status (debug output).
- `id?`: Show device ID for auto serial connection.
- `?`: Show the on-device help menu.

## Files

- [src/main.cpp](src/main.cpp) contains the firmware implementation and the on-device help menu.
- [platformio.ini](platformio.ini) contains the board and build configuration.

## Logs

Run logs can be stored in QSPI flash. Serial output can be captured to the [logs](logs/) folder.
