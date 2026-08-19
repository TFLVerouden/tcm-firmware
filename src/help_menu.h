#pragma once

#include <stddef.h>

// Ordered lines for the debug-only on-device help menu.
static const char *const COMMAND_HELP_LINES[] = {
    "\n=== Available Commands ===",
    "[Connection & Debugging]",
    "id?     - Show device ID for auto serial connection",
    "ver?    - Show protocol version",
    "B <0|1> - Toggle debug output",
    "S?      - Show system status (debug only)",
    "?       - Show the on-device help menu",
    "[Control Hardware]",
    "V <mA>  - Set proportional valve current in mA",
    "P <bar> - Set tank pressure in bar",
    "M <bar> - Set nebuliser pressure in bar",
    "O       - Open solenoid valve",
    "C       - Close solenoid valve",
    "I <0..1> - Set light level (pin 5, normalized PWM)",
    "G       - Send one trigger pulse now",
    "A <0|1> - Laser test mode off/on (streams photodiode readings when on)",
    "F <val> - Set fan speed (pin 3, not yet implemented)",
    "N <0|1> - Nebuliser off/on (pin A3)",
    "Q       - Quit active modes and return to idle",
    "[Read Out Sensors]",
    "P?      - Read current tank pressure (bar)",
    "M?      - Read current nebuliser pressure (bar)",
    "T?      - Read temperature & humidity",
    "[Configuration]",
    "W <us>  - Set wait before run in microseconds",
    "W?      - Read current wait before run in microseconds",
    "X       - Delete logged CSV files (experiment_log_*.csv)",
    "X!      - X + clear persisted state and dataset",
    "[Flow curve dataset Handling]",
    "L <N> <duration_ms> <csv> - Load flow curve. CSV format: "
    "<ms0>,<mA0>,<e0>,<t0>,<ms1>,<mA1>,<e1>,<t1>,...,<msN>,<mAN>,<eN>,<tN>",
    "         where e=solenoid enable (0/1), t=trigger event (0/1), and "
    "trigger pulse width is fixed in firmware",
    "L?      - Show loaded flow curve status",
    "[Cough]",
    "R       - Run the loaded flow curve dataset",
    "D       - Droplet-detect only (cont.)",
    "D <n>   - Droplet-detect only n times then stop",
    "D!      - Droplet-detect then run flow curve (cont.)",
    "D! <n>  - Droplet-detect then run flow curve n times",
};

static constexpr size_t COMMAND_HELP_LINE_COUNT =
    sizeof(COMMAND_HELP_LINES) / sizeof(COMMAND_HELP_LINES[0]);