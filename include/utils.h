
#ifndef UTILS_H
#define UTILS_H

#define BUFFER_OUT_SIZE 33

union data_out_union_t
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
  };

  uint8_t raw[BUFFER_OUT_SIZE];
};

enum class ControlMode
{
  VELOCITY,
  POWER
};

enum class BrakeMode
{
  BRAKE,
  COAST
};

// struct __attribute__((packed)) data_in_t
// {
//   // Whether the data is valid
//   bool valid = false;

//   // Motor data for all 4 channels
//   float channel_1_command;
//   float channel_2_command;
//   float channel_3_command;
//   float channel_4_command;

//   // Control mode
//   ControlMode channel_1_control_mode;
//   ControlMode channel_2_control_mode;
//   ControlMode channel_3_control_mode;
//   ControlMode channel_4_control_mode;

//   // Brake mode
//   BrakeMode channel_1_brake_mode;
//   BrakeMode channel_2_brake_mode;
//   BrakeMode channel_3_brake_mode;
//   BrakeMode channel_4_brake_mode;
// };

// #define BUFFER_IN_SIZE sizeof(data_in_t)

// union data_in_union_t
// {
//   data_in_t data;
//   uint8_t raw[BUFFER_IN_SIZE];
// };

#endif  // UTILS_H