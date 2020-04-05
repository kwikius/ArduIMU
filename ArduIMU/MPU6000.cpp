
#include <SPI.h>
#include "MPU6000.h"

namespace {

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
      uint16_t constexpr SampleRate = 10;
      byte constexpr  valSampleRateDiv = (GyroscopeOutputRate / SampleRate) - 1U;
      static_assert(valSampleRateDiv == 99,"");
   
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
   gyro_resolution{quan::angle_<float>::deg{2000.f/32767} };

   byte constexpr regGyroData = 67U;

   quan::acceleration_<
      float
   >::m_per_s2 constexpr
   accel_resolution{4.f/32767};
 
   byte constexpr regAccelData = 59U;

   // MPU6000 SPI functions
   byte MPU6000_SPI_read(byte reg)
   {
     digitalWrite(pinMpuCS, LOW);
     (void) SPI.transfer(static_cast<byte>(reg | 0x80)); // Set most significant bit
     byte const return_value = SPI.transfer(0);
     digitalWrite(pinMpuCS, HIGH);
     return(return_value);
   }

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
}

// return true if new data ready and clear flag
bool MPU6000dataReady()
{
   cli();
   bool const result = newdata != 0U;
   newdata = 0U;
   sei();
   return result;
}

// MPU6000 Initialization and configuration
void MPU6000init(void)
{
    // MPU6000 chip select setup
    pinMode(pinMpuCS, OUTPUT);
    digitalWrite(pinMpuCS, HIGH);

    // allow peripheral to get to running state
    while (millis() < 500U){ asm volatile ("nop":::);}
    
    // SPI initialization
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)

    delay(10);
    
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

    MPU6000_SPI_write(regMpuIntEnable,bitDataReadyIntEnable);         // INT: Raw data ready

    delay(1);

    MPU6000_SPI_write(regMpuIntPinConfig,bitClearIntOnRead);  

 //   delay(2);

    // MPU_INT is connected to INT 0. Enable interrupt on INT0
    attachInterrupt(0,MPU6000_data_int,RISING);
}

namespace {

   void read_data(byte reg, int16_t & result)
   {
       int16_t byte_H = MPU6000_SPI_read(reg);
       int16_t byte_L = MPU6000_SPI_read(reg + 1U );
       result = (byte_H << 8U) | byte_L;
   }

   void read_data(byte reg, quan::three_d::vect<int16_t> & result)
   {
      read_data(reg,result.x);
      read_data(reg+2U,result.y);
      read_data(reg+4U,result.z);
   }

}
// Read gyros and accel sensors on MPU6000
void MPU6000read(MpuData & result)
{
   quan::three_d::vect<int16_t> raw_data;

   read_data(regAccelData,raw_data);
   result.accel = raw_data * accel_resolution;

   read_data(regGyroData,raw_data);
   result.gyro = raw_data * gyro_resolution;
}
