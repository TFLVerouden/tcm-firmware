#pragma once

#include <Arduino.h>

// Print the complete on-device serial help menu.
inline void printCommandHelp() {
	Serial.println("\n=== Available Commands ===");
	Serial.println("[Connection & Debugging]");
	Serial.println("id?     - Show device ID for auto serial connection");
	Serial.println("ver?    - Show protocol version");
	Serial.println("B <0|1> - Toggle debug output");
	Serial.println("S?      - Show system status (debug only)");
	Serial.println("?       - Show the on-device help menu");
	Serial.println("[Control Hardware]");
	Serial.println("V <mA>  - Set proportional valve current in mA");
	Serial.println("P <bar> - Set tank pressure in bar");
	Serial.println("M <bar> - Set nebuliser pressure in bar");
	Serial.println("O       - Open solenoid valve");
	Serial.println("C       - Close solenoid valve");
	Serial.println("I <0..1> - Set light level (pin 5, normalized PWM)");
	Serial.println("G       - Send one trigger pulse now");
	Serial.println("A <0|1> - Laser test mode off/on (streams photodiode "
								 "readings when on)");
	Serial.println("F <val> - Set fan speed (pin 3, not yet implemented)");
	Serial.println("N <0|1> - Nebuliser off/on (pin A3)");
	Serial.println("Q       - Quit active modes and return to idle");
	Serial.println("[Read Out Sensors]");
	Serial.println("P?      - Read current pressure (bar)");
	Serial.println("M?      - Read current nebuliser pressure (bar)");
	Serial.println("T?      - Read temperature & humidity");
	Serial.println("[Configuration]");
	Serial.println("W <us>  - Set wait before run in microseconds");
	Serial.println("W?      - Read current wait before run in microseconds");
	Serial.println("X       - Delete logged CSV files (experiment_log_*.csv)");
	Serial.println("X!      - X + clear persisted state and dataset");
	Serial.println("[Flow curve dataset Handling]");
	Serial.println("L <N> <duration_ms> <csv> - Load flow curve. CSV format: "
								 "<ms0>,<mA0>,<e0>,<t0>,<ms1>,<mA1>,<e1>,<t1>,...,<msN>,<"
								 "mAN>,<eN>,<tN>");
	Serial.println("         where e=solenoid enable (0/1), t=trigger event "
								 "(0/1), and trigger pulse width is fixed in firmware");
	Serial.println("L?      - Show loaded flow curve status");
	Serial.println("[Cough]");
	Serial.println("R       - Run the loaded flow curve dataset");
	Serial.println("D       - Droplet-detect only (cont.)");
	Serial.println("D <n>   - Droplet-detect only n times then stop");
	Serial.println("D!      - Droplet-detect then run flow curve (cont.)");
	Serial.println("D! <n>  - Droplet-detect then run flow curve n times");
}