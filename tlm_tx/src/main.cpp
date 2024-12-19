#include <avr/wdt.h>
#include "Chrono.h"
#include "crc16.h"
#include <SPIMemory.h>
#include "MPU9250.h"
#include "BME280I2C.h"
#include "ublox.hpp"

#define Console Serial
#define Radio   Serial1

MPU9250 mpu;
BME280I2C bme;
Ublox ubx;
SPIFlash flash;

//ts is uptime in microseconds
//unix_ts is unix timestamp in seconds

struct imu_data_s { // 24B
  uint32_t ts;
  int16_t gyro[3];
  int16_t acc[3];
  int16_t mag[3];
  int16_t t;
} imu_data;

struct baro_data_s { // 12B
  uint32_t ts;
  uint32_t patm;
  int16_t temp;
  uint16_t rh;
} baro_data;

struct gnss_data_s { // 48B
  uint32_t ts;
  uint32_t unix_ts;
  uint16_t flags;
  uint8_t num_sats;
  uint8_t rsvd;
  int32_t lat;     // 1e-7 deg
  int32_t lon;     // 1e-7 deg
  int32_t height;  // WGS84 height cm
  int32_t velN;    // mm/s
  int32_t velE;    // mm/s
  int32_t velD;    // mm/s
  uint16_t hAcc;   // cm
  uint16_t vAcc;   // cm
  uint16_t sAcc;   // mm/s
} gnss_data;   

struct fdr_status_s { // 12B
  uint32_t ts;
  uint32_t used;   // bytes
  uint32_t total;  // bytes
} fdr_status;       

#define FLAG_IMU  1
#define FLAG_BARO    2
#define FLAG_GNSS    4
#define FLAG_FDR     8

#define TLM_HEADER   0x1acffc1d
#define FDR_HEADER   0x352ef853

struct hdr_s {    // 6B
  uint32_t header;
  uint16_t flags;
} tlm_hdr, fdr_hdr;

bool imu_init() {
  MPU9250Setting s;
  s.accel_fs_sel = ACCEL_FS_SEL::A16G;
  s.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
  s.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  s.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  s.gyro_fchoice = 0x03;
  s.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_10HZ;
  s.accel_fchoice = 0x01;
  s.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_10HZ;

  return mpu.setup(0x68, s);
}

bool imu_update() {
  mpu.read_accel_gyro_temp(imu_data.acc, imu_data.gyro, &imu_data.t);
  mpu.read_mag(imu_data.mag);
  imu_data.ts = micros();
  imu_data.t = mpu.get_temperature_int(imu_data.t);
  return true;
}

bool baro_init() {
  if (!bme.begin()) return false;

  BME280I2C::Settings s;
  s.tempOSR = BME280::OSR_X16;
  s.humOSR = BME280::OSR_X16;
  s.presOSR = BME280::OSR_X16;
  s.standbyTime = BME280::StandbyTime_500us;
  s.filter = BME280::Filter_Off;
  bme.setSettings(s);
  return true;
}

bool baro_update() {
  if(!bme.read(&baro_data.patm, &baro_data.temp, &baro_data.rh)) return false;
  baro_data.ts = micros();
  return true;
}

bool gnss_init() {
  ubx.begin();
  ubx.configure();
  return true;
}

bool gnss_update() {
  while(Serial.available()) {
    if(ubx.decode(Serial.read()) > 0) {
      //parse NAV-PVT
      
      return true;
    }
  }

  return false;
}

uint8_t fdr_buf[512];
uint16_t fdr_buf_wpos;
uint16_t fdr_buf_rpos;
uint32_t fdr_erase_pos;

bool fdr_init() {
  flash.begin();
  fdr_buf_wpos = 0;
  fdr_buf_rpos = 0;
  fdr_erase_pos = 0;
  fdr_status.used = 0;
  fdr_status.total = flash.getCapacity();
  return true;
}

bool fdr_update() {
  return false;
}

void fdr_save(uint8_t flags) {

}

bool tlm_init() {
  Radio.begin(9600);
  tlm_hdr.header = 0x1dfccf1a;  //0x1a, 0xcf, 0xfc, 0x1d
  return true;
}

void tlm_send(uint8_t flags) {
  tlm_hdr.flags = flags;
  Serial.write((const char*)&tlm_hdr, sizeof(tlm_hdr));

  CRC16 crc;
  if(flags & FLAG_IMU) {
    crc.calc(&imu_data, sizeof(imu_data));
    Serial.write((const char*)&imu_data, sizeof(imu_data));
  }

  if(flags & FLAG_BARO) {
    crc.calc(&baro_data, sizeof(baro_data));
    Serial.write((const char*)&baro_data, sizeof(baro_data));
  }

  if(flags & FLAG_GNSS) {
    crc.calc(&gnss_data, sizeof(gnss_data));
    Serial.write((const char*)&gnss_data, sizeof(gnss_data));
  }

  if(flags & FLAG_FDR) {
    crc.calc(&fdr_status, sizeof(fdr_status));
    Serial.write((const char*)&fdr_status, sizeof(fdr_status));
  }

  uint16_t c = crc.get();
  Serial.write((const char*)&c, sizeof(c));
}

void sys_err(const char* err) {
  while(1) {
    if(err) {
      Console.print("Err:");
      Console.println(err);
    }
    delay(1000);
  }
}

void setup() {
  Console.begin(115200);
  while(!Console);

  Wire.begin();
  //wdt_enable(WDTO_2S);
  
  if(!tlm_init())    sys_err("TLM");
  if(!fdr_init())    sys_err("FDR");

  if(!imu_init())    sys_err("IMU");
  if(!baro_init())   sys_err("Baro");
  if(!gnss_init())   sys_err("GNSS");
}

/*
  module  raw   fdr   tlm  
  imu     200   50    10
  baro    25    25    5
  gnss    5     5     5
  fdr     -     0     1

  module  rate  size
  header  20    6
  imu     20    24
  baro    5     12
  gnss    5     48
  fdr     1     12
  crc16   20    2

  radio 9600bps 1200B/s
*/

uint8_t imu_cnt, baro_cnt;
Chrono fdr_timer;

void loop() {
  if(imu_update()) {
    uint8_t flags = FLAG_IMU;
    if(baro_update()) flags |= FLAG_BARO;
    if(gnss_update()) flags |= FLAG_GNSS;
    if(fdr_update())  flags |= FLAG_FDR;
    fdr_save(flags);
    tlm_send(flags);
    wdt_reset();
  }
}
