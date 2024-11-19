#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <ESP32DMASPISlave.h>
#include <WiFi.h>
#include <Wire.h>

#include "SPI.h"
#include "helper.h"
#include "motorgo_plink.h"
#include "utils.h"

// IMU objects
Adafruit_LSM6DS3TRC lsm6ds;
Adafruit_LIS3MDL lis3mdl;

MotorGo::MotorGoPlink plink;
MotorGo::MotorChannel& ch1 = plink.ch1;
MotorGo::MotorChannel& ch2 = plink.ch2;
MotorGo::MotorChannel& ch3 = plink.ch3;
MotorGo::MotorChannel& ch4 = plink.ch4;

#define MOSI 35
#define MISO 48
#define SCK 47
#define SS 21

#define DATA_READY 36

static constexpr size_t QUEUE_SIZE = 1;
uint8_t* tx_buf;
uint8_t* rx_buf;

data_out_t data_out;
data_in_t data_in;

ESP32DMASPI::Slave slave;

void setup()
{
  Serial.begin(115200);

  Wire1.begin(14, 13);

  pinMode(DATA_READY, OUTPUT);
  digitalWrite(DATA_READY, LOW);

  bool lsm6ds_success = lsm6ds.begin_I2C(0x6a, &Wire1);
  //   bool lis3mdl_success = lis3mdl.begin_I2C(0x1C, &Wire1);

  //   Allocate buffers
  tx_buf = slave.allocDMABuffer(BUFFER_SIZE);
  rx_buf = slave.allocDMABuffer(BUFFER_SIZE);

  //   // Restart if IMU initialization fails
  //   if (!lsm6ds_success || !lis3mdl_success)
  //   {
  //     Serial.println("IMU initialization failed, restarting!");
  //     ESP.restart();
  //   }

  if (!lsm6ds_success)
  {
    Serial.println("IMU initialization failed, restarting!");
    ESP.restart();
  }

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_208_HZ);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_208_HZ);

  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true,  // enable z axis
                          true,                // polarity
                          false,               // don't latch
                          true);               // enabled!

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  slave.setDataMode(SPI_MODE3);    // default: SPI_MODE0
  slave.setQueueSize(QUEUE_SIZE);  // default: 1
  slave.setMaxTransferSize(BUFFER_SIZE);

  slave.begin(FSPI, SCK, MISO, MOSI, SS);

  Serial.println(BUFFER_IN_SIZE);
  Serial.println(BUFFER_OUT_SIZE);

  Serial.println("start spi slave");

  data_out.valid = false;

  plink.init();
}

unsigned long last_data_time = 0;
void loop()
{
  sensors_event_t accel, gyro, mag, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);
  //   lis3mdl.getEvent(&mag);

  data_out.channel_1_pos = ch1.get_position();
  data_out.channel_1_vel = ch1.get_velocity();

  data_out.channel_2_pos = ch2.get_position();
  data_out.channel_2_vel = ch2.get_velocity();

  data_out.channel_3_pos = ch3.get_position();
  data_out.channel_3_vel = ch3.get_velocity();

  data_out.channel_4_pos = ch4.get_position();
  data_out.channel_4_vel = ch4.get_velocity();

  data_out.gyro_x = gyro.gyro.x;
  data_out.gyro_y = gyro.gyro.y;
  data_out.gyro_z = gyro.gyro.z;

  data_out.accel_x = accel.acceleration.x;
  data_out.accel_y = accel.acceleration.y;
  data_out.accel_z = accel.acceleration.z;

  data_out.mag_x = (float)WiFi.RSSI();
  data_out.mag_y = 0;
  data_out.mag_z = 0;

  data_out.valid = true;

  //   Print imu data
  //   String str_out = "imu: ";
  //   str_out += "gyro_x: ";
  //   str_out += data_out.gyro_x;
  //   str_out += "\ngyro_y: ";
  //   str_out += data_out.gyro_y;
  //   str_out += "\ngyro_z: ";
  //   str_out += data_out.gyro_z;
  //   str_out += "\naccel_x: ";
  //   str_out += data_out.accel_x;
  //   str_out += "\naccel_y: ";
  //   str_out += data_out.accel_y;
  //   str_out += "\naccel_z: ";
  //   str_out += data_out.accel_z;

  //   freq_println(str_out, 10);

  // Copy data to tx buffer
  memcpy(tx_buf, data_out.raw, BUFFER_OUT_SIZE);

  digitalWrite(DATA_READY, HIGH);

  const size_t received_bytes =
      slave.transfer(tx_buf, rx_buf, BUFFER_SIZE, 100);

  digitalWrite(DATA_READY, LOW);

  if (received_bytes != 0)
  {
    // No data received, do nothing
    //   Decode data into data_in_t
    memcpy(data_in.raw, rx_buf, BUFFER_IN_SIZE);

    // Use freq_println to print the data at a specific frequency
    // String str_out = "data_in: ";
    // str_out += "valid: ";
    // str_out += data_in.valid;
    // str_out += "\nchannel_1_brake_mode: ";
    // str_out += (int)data_in.channel_1_brake_mode;
    // str_out += "\nchannel_2_brake_mode: ";
    // str_out += (int)data_in.channel_2_brake_mode;
    // str_out += "\nchannel_3_brake_mode: ";
    // str_out += (int)data_in.channel_3_brake_mode;
    // str_out += "\nchannel_4_brake_mode: ";
    // str_out += (int)data_in.channel_4_brake_mode;
    // str_out += "\nchannel_1_command: ";
    // str_out += data_in.channel_1_command;
    // str_out += "\nchannel_2_command: ";
    // str_out += data_in.channel_2_command;
    // str_out += "\nchannel_3_command: ";
    // str_out += data_in.channel_3_command;
    // str_out += "\nchannel_4_command: ";
    // str_out += data_in.channel_4_command;

    // freq_println(str_out, 10);

    if (data_in.valid)
    {
      if (data_in.channel_1_brake_mode == BrakeMode::BRAKE)
      {
        ch1.set_brake();
      }
      else
      {
        ch1.set_coast();
      }

      if (data_in.channel_2_brake_mode == BrakeMode::BRAKE)
      {
        ch2.set_brake();
      }
      else
      {
        ch2.set_coast();
      }

      if (data_in.channel_3_brake_mode == BrakeMode::BRAKE)
      {
        ch3.set_brake();
      }
      else
      {
        ch3.set_coast();
      }

      if (data_in.channel_4_brake_mode == BrakeMode::BRAKE)
      {
        ch4.set_brake();
      }
      else
      {
        ch4.set_coast();
      }

      ch1.set_power(data_in.channel_1_command);
      ch2.set_power(data_in.channel_2_command);
      ch3.set_power(data_in.channel_3_command);
      ch4.set_power(data_in.channel_4_command);

      last_data_time = millis();
    }
  }

  if (millis() - last_data_time > 1000)
  {
    ch1.set_power(0);
    ch2.set_power(0);
    ch3.set_power(0);
    ch4.set_power(0);
  }

  delayMicroseconds(100);
}
