
#ifndef UTILS_H
#define UTILS_H

void freq_println(String str, int freq)
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time > 1000 / freq)
  {
    Serial.println(str);
    last_print_time = now;
  }
}

#define BUFFER_OUT_SIZE 57

union data_out_t
{
  struct __attribute__((packed))
  {
    // Whether the data is valid
    bool valid = false;

    // Encoder data from all 4 channels
    float channel_1_pos;
    float channel_1_vel;

    float channel_2_pos;
    float channel_2_vel;

    float channel_3_pos;
    float channel_3_vel;

    float channel_4_pos;
    float channel_4_vel;

    // IMU data
    float gyro_x;
    float gyro_y;
    float gyro_z;

    float accel_x;
    float accel_y;
    float accel_z;
  };

  uint8_t raw[BUFFER_OUT_SIZE];
};

enum class ControlMode : uint8_t
{
  VELOCITY,
  POWER
};

enum class BrakeMode : uint8_t
{
  BRAKE,
  COAST
};

#define BUFFER_IN_SIZE 25
union data_in_t
{
  struct __attribute__((packed))
  {
    // Whether the data is valid
    bool valid = false;

    // Motor data for all 4 channels
    float channel_1_command;
    float channel_2_command;
    float channel_3_command;
    float channel_4_command;

    // Control mode
    ControlMode channel_1_control_mode;
    ControlMode channel_2_control_mode;
    ControlMode channel_3_control_mode;
    ControlMode channel_4_control_mode;

    // Brake mode
    BrakeMode channel_1_brake_mode;
    BrakeMode channel_2_brake_mode;
    BrakeMode channel_3_brake_mode;
    BrakeMode channel_4_brake_mode;
  };

  uint8_t raw[BUFFER_IN_SIZE];
};

#define BUFFER_SIZE 64

void print_data_in(const data_in_t &data)
{
  String output = "";

  output += "Valid: " + String(data.valid) + "\n";

  output += "Channel 1 Command: " + String(data.channel_1_command) + "\n";
  output += "Channel 1 Control Mode: " +
            String(static_cast<int>(data.channel_1_control_mode)) + "\n";
  output += "Channel 1 Brake Mode: " +
            String(static_cast<int>(data.channel_1_brake_mode)) + "\n";

  output += "Channel 2 Command: " + String(data.channel_2_command) + "\n";
  output += "Channel 2 Control Mode: " +
            String(static_cast<int>(data.channel_2_control_mode)) + "\n";
  output += "Channel 2 Brake Mode: " +
            String(static_cast<int>(data.channel_2_brake_mode)) + "\n";

  output += "Channel 3 Command: " + String(data.channel_3_command) + "\n";
  output += "Channel 3 Control Mode: " +
            String(static_cast<int>(data.channel_3_control_mode)) + "\n";
  output += "Channel 3 Brake Mode: " +
            String(static_cast<int>(data.channel_3_brake_mode)) + "\n";

  output += "Channel 4 Command: " + String(data.channel_4_command) + "\n";
  output += "Channel 4 Control Mode: " +
            String(static_cast<int>(data.channel_4_control_mode)) + "\n";
  output += "Channel 4 Brake Mode: " +
            String(static_cast<int>(data.channel_4_brake_mode)) + "\n";

  freq_println(output, 1);  // Adjust frequency as needed
}

#endif  // UTILS_H