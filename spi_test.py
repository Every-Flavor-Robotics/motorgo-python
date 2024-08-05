import spidev
import time
import struct

# Constants
BUFFER_SIZE = 64  # Size of data_out_t (1 byte for bool + 8 * 4 bytes for floats)

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Open bus 0, device (CS) 0
spi.mode = 0
spi.max_speed_hz = 50000  # Set SPI speed

def decode_data(data):
    # Unpack the data based on the packed struct definition
    unpacked_data = struct.unpack_from('<?8f', data)
    valid = unpacked_data[0]
    channel_1_pos = unpacked_data[1]
    channel_1_vel = unpacked_data[2]
    channel_2_pos = unpacked_data[3]
    channel_2_vel = unpacked_data[4]
    channel_3_pos = unpacked_data[5]
    channel_3_vel = unpacked_data[6]
    channel_4_pos = unpacked_data[7]
    channel_4_vel = unpacked_data[8]

    return valid, channel_1_pos, channel_1_vel, channel_2_pos, channel_2_vel, channel_3_pos, channel_3_vel, channel_4_pos, channel_4_vel

BUFFER_IN_SIZE = 49

# Enums to match C++ definitions
class ControlMode:
    VELOCITY = 0
    POWER = 1

class BrakeMode:
    BRAKE = 0
    COAST = 1

def prepare_data(valid, channel_commands, control_modes, brake_modes):
    # Create the packed data structure
    data = struct.pack(
        '<?4f4B4B',
        valid,
        channel_commands[0],
        channel_commands[1],
        channel_commands[2],
        channel_commands[3],
        control_modes[0],
        control_modes[1],
        control_modes[2],
        control_modes[3],
        brake_modes[0],
        brake_modes[1],
        brake_modes[2],
        brake_modes[3]
    )

    # Ensure the data is the correct size
    if len(data) < BUFFER_IN_SIZE:
        data += bytes(BUFFER_IN_SIZE - len(data))

    return data

def send_message(data):
    # Ensure data is the correct size
    if len(data) < BUFFER_IN_SIZE:
        data += bytes(BUFFER_IN_SIZE - len(data))
    elif len(data) > BUFFER_IN_SIZE:
        data = data[:BUFFER_IN_SIZE]

    # Send the message over SPI and receive the response
    response = spi.xfer2(list(data))
    return response

try:
    while True:
        # Example data
        valid = True
        channel_commands = [1.0, 2.0, 3.0, 4.0]
        control_modes = [
            ControlMode.VELOCITY,
            ControlMode.POWER,
            ControlMode.VELOCITY,
            ControlMode.POWER
        ]
        brake_modes = [
            BrakeMode.BRAKE,
            BrakeMode.COAST,
            BrakeMode.BRAKE,
            BrakeMode.COAST
        ]

        # Prepare data
        data = prepare_data(valid, channel_commands, control_modes, brake_modes)

        # Send the data over SPI and receive a response
        response = send_message(data)

        # Convert response to byte array
        response_bytes = bytearray(response)

        # Decode the data
        valid, channel_1_pos, channel_1_vel, channel_2_pos, channel_2_vel, channel_3_pos, channel_3_vel, channel_4_pos, channel_4_vel = decode_data(response_bytes)

        print(f"Valid: {valid}")
        print(f"Channel 1 Pos: {channel_1_pos}, Vel: {channel_1_vel}")
        print(f"Channel 2 Pos: {channel_2_pos}, Vel: {channel_2_vel}")
        print(f"Channel 3 Pos: {channel_3_pos}, Vel: {channel_3_vel}")
        print(f"Channel 4 Pos: {channel_4_pos}, Vel: {channel_4_vel}")

        time.sleep(0.5)  # Wait for 1 second before sending the next message

except KeyboardInterrupt:
    # Clean up on Ctrl+C
    spi.close()
    print("SPI communication stopped.")