// This file is used to get the board ID from the MotorGo board
// and send it to the Raspberry Pi to identify which board is connected.

#include <ESP32DMASPISlave.h>
#include <Preferences.h>

#include "SPI.h"

#define BUFFER_SIZE (76)

#define INIT_MESSAGE_TYPE 0x01
#define INIT_IN_SIZE 25
union init_input_t
{
  struct __attribute__((packed))
  {
    uint8_t message_type = INIT_MESSAGE_TYPE;

    // Target update frequency
    float frequency;
    float power_supply_voltage;
    float channel_1_voltage_limit;
    float channel_2_voltage_limit;
    float channel_3_voltage_limit;
    float channel_4_voltage_limit;
  };

  uint8_t raw[INIT_IN_SIZE];
};

#define INIT_OUT_SIZE 5
union init_output_t
{
  struct __attribute__((packed))
  {
    uint8_t message_type = INIT_MESSAGE_TYPE;

    uint16_t board_id;
    uint16_t firmware_version;
  };

  uint8_t raw[INIT_OUT_SIZE];
};

// Pin the MotorGo uses to indicate that it is ready to transfer new data
#define DATA_READY 36

// SPI communication setup
static constexpr size_t QUEUE_SIZE = 1;
uint8_t *tx_buf;
uint8_t *rx_buf;

ESP32DMASPI::Slave slave;

// Time to delay between data transmissions
unsigned long delay_time = 0;
unsigned long last_data_time = 0;
unsigned long loop_start_time = 0;

bool communication_active = true;

void init_spi_comms()
{
  bool ready = false;

  //   Prepare the initialize data
  init_output_t init_out;

  //   Load board ID from preferences, 16-bit unsigned integer
  Preferences preferences;
  preferences.begin("__mg", true);
  init_out.board_id = preferences.getUShort("__mg_id", 0);
  preferences.end();

  init_out.firmware_version = 0x00;

  while (true)
  {
    // Copy data to tx buffer
    memcpy(tx_buf, init_out.raw, INIT_OUT_SIZE);

    //  Indicate that data is ready
    digitalWrite(DATA_READY, HIGH);

    //  Send the data
    const size_t received_bytes = slave.transfer(tx_buf, rx_buf, BUFFER_SIZE);

    //  Indicate that data is not ready
    digitalWrite(DATA_READY, LOW);
    delay(100);
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(DATA_READY, OUTPUT);
  digitalWrite(DATA_READY, LOW);

  //   Allocate buffers
  tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
  rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

  slave.setDataMode(SPI_MODE3);    // default: SPI_MODE0
  slave.setQueueSize(QUEUE_SIZE);  // default: 1
  slave.setMaxTransferSize(BUFFER_SIZE);

  slave.begin(FSPI, SCK, MISO, MOSI, SS);

  Serial.println("Sending board ID");

  init_spi_comms();
}

void loop() {}
