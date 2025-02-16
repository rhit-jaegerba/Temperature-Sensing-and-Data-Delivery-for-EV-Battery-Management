import matplotlib.pyplot as plt
import matplotlib.animation as animation
import smbus
import time

# Define the slave I2C address and bus number
SLAVE_ADDRESS = 0x20
I2C_BUS = 1  # Change to 0 if using an older Raspberry Pi model

# Initialize the I2C bus
bus = smbus.SMBus(I2C_BUS)

# Store data for plotting
x_data = []
y_data_1 = []  # Data for message 0x73
y_data_2 = []  # Data for message 0x72
y_data_3 = []  # Data for message 0x71
time_counter = 0

# Function to read data from the slave
def read_from_slave(message):
    try:
        num_bytes = 2
        response = bus.read_i2c_block_data(SLAVE_ADDRESS, message, num_bytes)
        combined_value = 0
        for i in range(num_bytes):
            combined_value = (combined_value << 8) | response[i]

        # Convert to signed 16-bit integer (handle two's complement)
        if combined_value & 0x8000:  # If sign bit (bit 15) is set
            combined_value -= 0x10000  # Convert to negative value
        
        scaled_value = combined_value * 0.0625  # Scale if needed
        return scaled_value
    except IOError:
        print(f"Error: Unable to read from slave for message {hex(message)}")
        return None

# Function to update the plot
def update(frame):
    global time_counter

    # Read data for each message
    Min = read_from_slave(0x99)
    Max = read_from_slave(0xBB)
    Avg = read_from_slave(0xAA)

    if Min is not None and Max is not None and Avg is not None:
        x_data.append(time_counter)
        y_data_1.append(Min)
        y_data_2.append(Max)
        y_data_3.append(Avg)
        time_counter += 1

        # Keep the plot window from growing indefinitely
        if len(x_data) > 100:
            x_data.pop(0)
            y_data_1.pop(0)
            y_data_2.pop(0)
            y_data_3.pop(0)

    ax.clear()
    ax.plot(x_data, y_data_1, label="Sensors Min (0x99)", color='r')
    ax.plot(x_data, y_data_2, label="Sensors Max (0xBB)", color='g')
    ax.plot(x_data, y_data_3, label="Sensors Avg (0xAA)", color='b')

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Value")
    ax.set_title("Live Plot of Multiple Sensor Values")
    ax.legend()
    ax.grid()

# Set up the plot
fig, ax = plt.subplots()
ani = animation.FuncAnimation(fig, update, interval=500)  # Update every 500ms

plt.show()
