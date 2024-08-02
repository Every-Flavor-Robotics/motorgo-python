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

data_out_union_t data_out_union;

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

  data_out_union.valid = false;
  data_out_union.channel_1_pos = 1.0;
  data_out_union.channel_1_vel = 2.0;
  data_out_union.channel_2_pos = 3.0;
  data_out_union.channel_2_vel = 4.0;
  data_out_union.channel_3_pos = 5.0;
  data_out_union.channel_3_vel = 6.0;
  data_out_union.channel_4_pos = 7.0;
  data_out_union.channel_4_vel = 8.0;
}

void loop()
{
  // Print data out
  Serial.println("data_out_union.valid: " + String(data_out_union.valid));
  Serial.println("data_out_union.channel_1_pos: " +
                 String(data_out_union.channel_1_pos));
  Serial.println("data_out_union.channel_1_vel: " +
                 String(data_out_union.channel_1_vel));
  Serial.println("data_out_union.channel_2_pos: " +
                 String(data_out_union.channel_2_pos));
  Serial.println("data_out_union.channel_2_vel: " +
                 String(data_out_union.channel_2_vel));
  Serial.println("data_out_union.channel_3_pos: " +
                 String(data_out_union.channel_3_pos));
  Serial.println("data_out_union.channel_3_vel: " +
                 String(data_out_union.channel_3_vel));
  Serial.println("data_out_union.channel_4_pos: " +
                 String(data_out_union.channel_4_pos));
  Serial.println("data_out_union.channel_4_vel: " +
                 String(data_out_union.channel_4_vel));

  //  Print data out raw as HEX
  for (size_t i = 0; i < BUFFER_OUT_SIZE; i++)
  {
    Serial.print(data_out_union.raw[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Copy data to tx buffer
  memcpy(tx_buf, data_out_union.raw, BUFFER_OUT_SIZE);

  // start and wait to complete one BIG transaction (same data will be received
  // from slave)
  const size_t received_bytes = slave.transfer(tx_buf, rx_buf, BUFFER_SIZE);

  Serial.println("received_bytes: " + String(received_bytes));

  //   Print received data
  dumpBuffers("received", rx_buf, 0, received_bytes);

  data_out_union.valid = !data_out_union.valid;

  //   // verify and dump difference with received data
  //   if (verifyAndDumpDifference("slave", tx_buf, BUFFER_SIZE, "master",
  //   rx_buf,
  //                               received_bytes))
  //   {
  //     Serial.println("successfully received expected data from master");
  //   }
  //   else
  //   {
  //     Serial.println("unexpected difference found between master/slave
  //     data");
  //   }
}