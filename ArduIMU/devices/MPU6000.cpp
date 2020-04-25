
#include <SPI.h>
#include "MPU6000.h"
#include "../storage.h"
#include "../mpudata.h"
#include "../runmode.h"

namespace {

   QUAN_QUANTITY_LITERAL(angle,deg);
   QUAN_QUANTITY_LITERAL(acceleration,m_per_s2);

   constexpr inline 
   quan::reciprocal_time_<
      quan::angle_<float>::deg 
   >::per_s operator "" _deg_per_s ( long double v)
   {
      return quan::reciprocal_time_<
        quan::angle_<float>::deg 
      >::per_s {quan::angle_<float>::deg{v}};
   }

   // MPU6000 chip select 
   byte constexpr pinMpuCS = 4;

   byte constexpr regMpuUserCtrl = 106U;
      byte constexpr bitI2cIfDis = (1U << 4U);

   byte constexpr regMpuPwrMgmt1 = 107U;
      byte constexpr bitMpuDeviceReset  = (1U << 7U);
      byte constexpr valMpuClkSelPLLGyroZ = 0x03;

// 4.2 Register 25 – Sample Rate Divider
   byte constexpr regMpuSampleRateDiv  = 25U;
      // Sample Rate = GyroscopeOutputRate / (1 + SMPLRT_DIV)
      // SMPLRT_DIV = (GyroScopeOutputRate / SampleRate) - 1
      static constexpr bool UsingDLPF = true;
      uint16_t constexpr GyroscopeOutputRate = UsingDLPF ? 1000U : 8000U;
      uint16_t constexpr SampleRate = 25;  // Hz
      byte constexpr  valSampleRateDiv = (GyroscopeOutputRate / SampleRate) - 1U;
     // static_assert(valSampleRateDiv == 99,"");
   
   byte constexpr regMpuConfig = 26U;
      byte constexpr valDlpfConfig20Hz = 4U; // 20 Hz
      byte constexpr valDlpfConfig5Hz = 6U; // 5 Hz

   byte constexpr regMpuGyroConfig = 27U;
      byte constexpr valGyroScale2000_deg_per_s = (0b11U << 3U); 

   byte constexpr regMpuAccelConfig = 28U;
      byte constexpr valAccelScale4g = (0b01U << 3U); // Accel range 4g

   byte constexpr regMpuIntPinConfig = 55U;
      byte constexpr bitClearIntOnRead = (1U << 4U);

   byte constexpr regMpuIntEnable = 56U;
     byte constexpr bitDataReadyIntEnable = (1U << 0U);
         
   quan::reciprocal_time_<
      quan::angle_<float>::deg 
   >::per_s constexpr 
   gyro_resolution{quan::angle_<float>::deg{2000.f/32768} };

   byte constexpr regGyroData = 67U;

   quan::acceleration_<
      float
   >::m_per_s2 constexpr // N.B for some reason 2 * what it should be according to ref man?
   accel_resolution = (8.f * quan::acceleration::g)/ 32768;
 
   byte constexpr regAccelData = 59U;

   void MPU6000_SPI_write(byte reg, byte data)
   {
     digitalWrite(pinMpuCS, LOW);
     (void) SPI.transfer(reg);
     (void) SPI.transfer(data);
     digitalWrite(pinMpuCS, HIGH);
   }

   volatile uint8_t newdata = 0U;

   void MPU6000_data_int()
   {
      newdata = 1U;
   }

   quan::three_d::vect<quan::acceleration_<float>::m_per_s2> 
   accel_offset
   { 0.0_m_per_s2,0.0_m_per_s2,0.0_m_per_s2};

   quan::three_d::vect<float> accel_gain{1.f,1.f,1.f};

   quan::three_d::vect<
      quan::reciprocal_time_<
         quan::angle_<float>::deg 
      >::per_s
   > gyro_offset
   { 0.0_deg_per_s,0.0_deg_per_s,0.0_deg_per_s};

   quan::three_d::vect<float> gyro_gain{1.f,1.f,1.f};

   bool mpu6000_initialised = false;

   // ready ready to read but not read
   // avaialable - resd
   bool gyr_data_ready = false;
   bool gyr_data_available = false;
   bool acc_data_ready = false;
   bool acc_data_available = false;

   MpuData mpuData;
}

namespace {

   void read_data(byte reg, int16_t & result)
   {
     result = static_cast<int16_t>(SPI.transfer(0) << 8) | 
                 static_cast<int16_t>(SPI.transfer(0));
   }

   void read_data(byte reg, quan::three_d::vect<int16_t> & result)
   {
      digitalWrite(pinMpuCS, LOW);
         (void) SPI.transfer(static_cast<byte>(reg | 0x80)); // Set most significant bit
         read_data(reg,result.x);
         read_data(reg+2U,result.y);
         read_data(reg+4U,result.z);
      digitalWrite(pinMpuCS, HIGH);
   }
}

// Read gyros and accel sensors on MPU6000
void MPU6000read(MpuData & result)
{
   quan::three_d::vect<int16_t> raw_data;
   {
      read_data(regAccelData,raw_data);
      auto const accel = raw_data * accel_resolution + accel_offset;
      result.accel.x = accel.x * accel_gain.x;
      result.accel.y = accel.y * accel_gain.y;
      result.accel.z = accel.z * accel_gain.z;
      acc_data_available = true;
   }
   {
      read_data(regGyroData,raw_data);
      auto const gyro = raw_data * gyro_resolution + gyro_offset;
      result.gyro.x = gyro.x * gyro_gain.x;
      result.gyro.y = gyro.y * gyro_gain.y;
      result.gyro.z = gyro.z * gyro_gain.z;
      gyr_data_available = true;
   }
}

bool MPU6000dataReady()
{
   cli();
   bool const result = newdata != 0U;
   newdata = 0U;
   sei();
   if ( result){
      acc_data_ready = true;
      gyr_data_ready = true;
   }
   return result;
}

bool MPU6000AccDataReady()
{
   if ( acc_data_ready){
      acc_data_ready = false;
      return true;
   }else{
      return MPU6000dataReady();
   }
}

bool MPU6000GyrDataReady()
{
   if( gyr_data_ready){
      gyr_data_ready = false;
      return true;
   }else{
      return MPU6000dataReady();
   }
}

bool MPU6000AccRead(quan::three_d::vect<quan::acceleration_<float>::m_per_s2> & result)
{
   if (!acc_data_available){
      MPU6000read(mpuData);
   }
   result = mpuData.accel;
   acc_data_available = false;
   return true;
}

bool MPU6000GyrRead(
      quan::three_d::vect<
         quan::reciprocal_time_<
            quan::angle_<float>::deg
         >::per_s 
      > & result)
{
   if (!gyr_data_available){
      MPU6000read(mpuData);
   }
   result = mpuData.gyro;
   gyr_data_available = false;
   return true;
}

void MPU6000init(void)
{
   if ( mpu6000_initialised){
      return;
   }
   // MPU6000 chip select setup
   pinMode(pinMpuCS, OUTPUT);
   digitalWrite(pinMpuCS, HIGH);

   bool accel_calibrated = false;
   readValueFromStorage(ACC_CALIBRATED,accel_calibrated);
   if ( accel_calibrated){
      // technically a float in eeprom but should be OK
      readValueFromStorage(ACC_OFST,accel_offset);
      readValueFromStorage(ACC_GAIN,accel_gain);
   }
 
   bool gyro_calibrated = false;
   readValueFromStorage(GYR_CALIBRATED,gyro_calibrated);
   if ( gyro_calibrated){
      // a float in eeprom
      quan::three_d::vect<float> temp;
      readValueFromStorage(GYR_OFST,temp);
      // scale it by the deg_per_s unit
      gyro_offset = temp * 1.0_deg_per_s;
      readValueFromStorage(GYR_GAIN,gyro_gain);
   }

   // SPI initialization
   SPI.begin();
   SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)

   delay(10);

   // allow peripheral to get to running state from power up
   while (millis() < 500U){ asm volatile ("nop":::);}

   // Chip reset
   MPU6000_SPI_write(regMpuPwrMgmt1, bitMpuDeviceReset);

   delay(100);
   // Wake up device and select GyroZ clock (better performance)
   MPU6000_SPI_write(regMpuPwrMgmt1, valMpuClkSelPLLGyroZ);

   delay(1);
   // Disable I2C bus (recommended on datasheet)
   MPU6000_SPI_write(regMpuUserCtrl, bitI2cIfDis);

   delay(1);
   // SAMPLE RATE    
   MPU6000_SPI_write(regMpuSampleRateDiv,valSampleRateDiv);        

   delay(1);

   MPU6000_SPI_write(regMpuConfig, valDlpfConfig20Hz);  

   delay(1);

   MPU6000_SPI_write(regMpuGyroConfig,valGyroScale2000_deg_per_s);  // Gyro scale 2000º/s

   delay(1);

   MPU6000_SPI_write(regMpuAccelConfig,valAccelScale4g);            // Accel scale 4g (4096LSB/g)

   delay(1);

   MPU6000_SPI_write(regMpuIntPinConfig,bitClearIntOnRead); 

   delay(1);  

   // MPU_INT is connected to INT 0. Enable interrupt on INT0
   attachInterrupt(0,MPU6000_data_int,RISING);
 
   MPU6000_SPI_write(regMpuIntEnable,bitDataReadyIntEnable);         // INT: Raw data ready

   SPI.setClockDivider(SPI_CLOCK_DIV2);      // SPI at 8Mhz (on 16Mhz clock)

   mpu6000_initialised = true;

   MpuData prev_result;
   quan::time_<uint32_t>::ms time_stamp;
}
