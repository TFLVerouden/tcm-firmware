import csv
from pathlib import Path

from tcm_utils.file_dialogs import ask_open_file, find_repo_root, repo_config_path
from dvg_devices.BaseDevice import SerialDevice

# Configure variables
baud = 115200
delimiter = ','

# =========================================================================================
# Extract time and value arrays from csv file. Input: "filename", "delimiter"
# =========================================================================================


def extract(filename, delimiter=','):
    # Define output variables
    time = []
    mA = []
    enable = []
    row_idx = 0

    with open(filename, 'r') as csvfile:
        # Reader object of the file
        csvreader = csv.reader(csvfile, delimiter=delimiter)

        # Extract data from the file and create time, value, enable arrays
        for rows in csvreader:
            if len(rows) < 3 or not rows[0] or not rows[1] or rows[2] == "":
                print(f"Encountered empty cell at row index {row_idx}!")
                time = []
                mA = []
                enable = []
                break
            else:
                # replace ',' with '.' depending on csv format (';' delim vs ',' delim)
                time.append(rows[0].replace(',', '.'))
                mA.append(rows[1].replace(',', '.'))
                enable.append(rows[2].strip())
                row_idx += 1

    return time, mA, enable

# =========================================================================================
# Format incomming time and value arrays according to serial protocol. Input: [time], [mA]
# =========================================================================================


def format(time_array, mA_array, enable_array, prefix="L", handshake_delim=" ", data_delim=",", line_feed='\n'):

    duration = time_array[-1]

    # Check for inconsistent dataset length
    if (
        len(time_array) != len(mA_array)
        or len(time_array) != len(enable_array)
        or len(time_array) == 0
        or len(mA_array) == 0
    ):
        print(
            f"Arrays are not compatible! Time length: {len(time_array)}, mA length: {len(mA_array)}, enable length: {len(enable_array)}")
        return
    else:
        # Create 'handshake' sequence
        header = [prefix, handshake_delim, str(
            len(time_array)), handshake_delim, duration, handshake_delim]

        # Append in order <time0, mA0, e0, time1, mA1, e1, ...>
        data = [
            str(val)
            for time, mA, e in zip(time_array, mA_array, enable_array)
            for val in (time, mA, e)
        ]

        # format into one string to send over serial
        output = "".join(header) + data_delim.join(data) + line_feed

    return output


# =========================================================================================
# Write dataset over serial using communication protocol
# =========================================================================================
# arduino = Arduino(
    # name="MCU_1", long_name="Adafruit ItsyBitsy M4 Feather Express", connect_to_specific_ID="TCM_control")
mcu_1 = SerialDevice(name="MCU_1", long_name="TCM_control")
mcu_1.serial_settings["baudrate"] = baud
mcu_1.serial_settings["timeout"] = 1


def id_query():
    _success, reply = mcu_1.query("id?")
    if isinstance(reply, str):
        reply_broad = reply.strip()
        reply_specific = None
    else:
        reply_broad = ""
        reply_specific = None
    return reply_broad, reply_specific


mcu_1.set_ID_validation_query(
    ID_validation_query=id_query,
    valid_ID_broad="TCM_control",
    valid_ID_specific=None,
)

repo_root = find_repo_root()
last_port_path = repo_config_path("MCU_1_port.txt")

if not mcu_1.auto_connect(filepath_last_known_port=str(last_port_path)):
    raise SystemError("Arduino not found via auto_connect")

flow_curve_path = ask_open_file(
    key="flow_curve_csv",
    title="Select flow curve CSV",
    filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
    default_dir=repo_root / "src_python",
    start=repo_root,
)

if flow_curve_path is None:
    raise SystemExit("No flow curve CSV selected")

# data[0] is time array, data[1] is mA array, data[2] is enable array
data = extract(str(flow_curve_path), delimiter)
serial_command = format(data[0], data[1], data[2])

print(serial_command)                       # Debug print

# Encode serial command to utf8 format for arduino.
mcu_1.write(serial_command.encode('utf-8'))

mcu_1.close()
