import argparse
import serial
import datetime
import os

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("-d", "--device", help="device to read from", default="/dev/ttyACM0")
parser.add_argument("-s", "--speed", help="speed in bps", default=9600, type=int)
args = parser.parse_args()

outputFilePath = os.path.join(
    os.path.dirname(__file__),
    datetime.datetime.now().strftime("%Y-%m-%dT%H.%M.%S") + ".csv"
)
outputFilePath = '/media/pi/DATAFLASH/data.csv'

with serial.Serial(args.device, args.speed, timeout=1) as ser, open(outputFilePath, mode='w') as outputFile:
    print("Logging started. Ctrl-C to stop.")
    # Write CSV header
    outputFile.write("Date,Time,Data\n")
    try:
        line_buffer = ""
        while True:
            data = ser.read(ser.inWaiting()).decode(errors='replace')
            if data:
                line_buffer += data
                while "\n" in line_buffer:
                    line, line_buffer = line_buffer.split("\n", 1)
                    timestamp = datetime.datetime.now()
                    date_str = timestamp.strftime("%Y-%m-%d")
                    time_str = timestamp.strftime("%H:%M:%S.%f")[:-3]  # Millisecond precision
                    csv_entry = f"{date_str},{time_str},{line.strip()}\n"
                    print(csv_entry)
                    outputFile.write(csv_entry)
                    outputFile.flush()
    except KeyboardInterrupt:
        print("Logging stopped")