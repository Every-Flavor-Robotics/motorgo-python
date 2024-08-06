#include <ESP32SPISlave.h>

#include "SPI.h"
#include "helper.h"
#include "motorgo_plink.h"
#include "utils.h"

MotorGo::MotorGoPlink plink;
MotorGo::MotorChannel& ch1 = plink.ch1;
MotorGo::MotorChannel& ch2 = plink.ch2;
MotorGo::MotorChannel& ch3 = plink.ch3;
MotorGo::MotorChannel& ch4 = plink.ch4;

#define MOSI 42
#define MISO 41
#define SCK 40
#define SS 39

static constexpr size_t QUEUE_SIZE = 1;
static constexpr size_t BUFFER_SIZE =
    64;  // Increase buffer size to handle larger messages
uint8_t tx_buf[BUFFER_SIZE]{0};
uint8_t rx_buf[BUFFER_SIZE]{0};

data_out_t data_out;
data_in_t data_in;

void construct_data_out() {}

ESP32SPISlave slave;
void setup()
{
  Serial.begin(115200);

  delay(5000);

  slave.setDataMode(SPI_MODE0);    // default: SPI_MODE0
  slave.setQueueSize(QUEUE_SIZE);  // default: 1

  // begin() after setting
  slave.begin(HSPI, SCK, MISO, MOSI,
              SS);  // default: HSPI (please refer README for pin assignments)

  //   Serial.println(BUFFER_IN_SIZE);
  Serial.println(BUFFER_OUT_SIZE);

  Serial.println("start spi slave");

  data_out.valid = false;
  data_out.channel_1_pos = 1.0;
  data_out.channel_1_vel = 2.0;
  data_out.channel_2_pos = 3.0;
  data_out.channel_2_vel = 4.0;
  data_out.channel_3_pos = 5.0;
  data_out.channel_3_vel = 6.0;
  data_out.channel_4_pos = 7.0;
  data_out.channel_4_vel = 8.0;
}

void loop()
{
  data_out.channel_1_pos = ch1.get_position();
  data_out.channel_1_vel = ch1.get_velocity();

  data_out.channel_2_pos = ch2.get_position();
  data_out.channel_2_vel = ch2.get_velocity();

  data_out.channel_3_pos = ch3.get_position();
  data_out.channel_3_vel = ch3.get_velocity();

  data_out.channel_4_pos = ch4.get_position();
  data_out.channel_4_vel = ch4.get_velocity();
  data_out.valid = true;

  // Copy data to tx buffer
  memcpy(tx_buf, data_out.raw, BUFFER_OUT_SIZE);

  // start and wait to complete one BIG transaction (same data will be received
  // from slave)
  const size_t received_bytes =
      slave.transfer(tx_buf, rx_buf, BUFFER_SIZE, 100);

  //   Decode data into data_in_t
  //   Serial.println(received_bytes);
  memcpy(data_in.raw, rx_buf, BUFFER_SIZE);

  //   Print contents of data_in
  Serial.print("valid: ");
  Serial.println(data_in.valid);
  Serial.print("channel_1_brake_mode: ");
  Serial.println((int)data_in.channel_1_brake_mode);
  Serial.print("channel_2_brake_mode: ");
  Serial.println((int)data_in.channel_2_brake_mode);
  Serial.print("channel_3_brake_mode: ");
  Serial.println((int)data_in.channel_3_brake_mode);
  Serial.print("channel_4_brake_mode: ");
  Serial.println((int)data_in.channel_4_brake_mode);
  Serial.print("channel_1_command: ");
  Serial.println(data_in.channel_1_command);
  Serial.print("channel_2_command: ");
  Serial.println(data_in.channel_2_command);
  Serial.print("channel_3_command: ");
  Serial.println(data_in.channel_3_command);
  Serial.print("channel_4_command: ");
  Serial.println(data_in.channel_4_command);

  //   if (data_in.valid)
  //   {
  //     if (data_in.channel_1_brake_mode == BrakeMode::BRAKE)
  //     {
  //       ch1.set_brake();
  //     }
  //     else
  //     {
  //       ch1.set_coast();
  //     }

  //     if (data_in.channel_2_brake_mode == BrakeMode::BRAKE)
  //     {
  //       ch2.set_brake();
  //     }
  //     else
  //     {
  //       ch2.set_coast();
  //     }

  //     if (data_in.channel_3_brake_mode == BrakeMode::BRAKE)
  //     {
  //       ch3.set_brake();
  //     }
  //     else
  //     {
  //       ch3.set_coast();
  //     }

  //     if (data_in.channel_4_brake_mode == BrakeMode::BRAKE)
  //     {
  //       ch4.set_brake();
  //     }
  //     else
  //     {
  //       ch4.set_coast();
  //     }

  //     ch1.set_power(data_in.channel_1_command);
  //     ch2.set_power(data_in.channel_2_command);
  //     ch3.set_power(data_in.channel_3_command);
  //     ch4.set_power(data_in.channel_4_command);
}
}