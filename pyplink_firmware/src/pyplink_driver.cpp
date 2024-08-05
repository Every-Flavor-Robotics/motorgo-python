#include <ESP32SPISlave.h>

#include "SPI.h"
#include "helper.h"
#include "utils.h"
// MOSI - Purple
// MISO - WHITE
// SCK - GREY
// SS - BLACK

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
  // Copy data to tx buffer
  memcpy(tx_buf, data_out.raw, BUFFER_OUT_SIZE);

  // start and wait to complete one BIG transaction (same data will be received
  // from slave)
  const size_t received_bytes =
      slave.transfer(tx_buf, rx_buf, BUFFER_SIZE, 1000);

  //   Decode data into data_in_t
  //   Serial.println(received_bytes);
  memcpy(data_in.raw, rx_buf, BUFFER_SIZE);

  print_data_in(data_in);

  data_out.valid = !data_out.valid;
  //   Increase all channels by 1
  data_out.channel_1_pos += 1;
  data_out.channel_1_vel += 1;
  data_out.channel_2_pos += 1;
  data_out.channel_2_vel += 1;
  data_out.channel_3_pos += 1;
  data_out.channel_3_vel += 1;
  data_out.channel_4_pos += 1;
  data_out.channel_4_vel += 1;

  delay(1000);
}