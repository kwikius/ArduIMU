#ifndef QUAN_SENSORS_SENSOR_READING_H_INCLUDED
#define QUAN_SENSORS_SENSOR_READING_H_INCLUDED

#include <quan/time.hpp>

template <typename Q>
struct sensor_reading{
  sensor_reading(Q const & valueIn, quan::time_<uint32_t>::ms const& timeStampIn)
   : m_value{valueIn},m_time_stamp{timeStampIn}{}
  quan::time_<uint32_t>::ms const & getTimeStamp() const{ return m_time_stamp;}
  Q const & getValue() const { return m_value;}
 private:
   quan::time_<uint32_t>::ms m_time_stamp;
   Q m_value;
};

#endif // QUAN_SENSORS_SENSOR_READING_H_INCLUDED
