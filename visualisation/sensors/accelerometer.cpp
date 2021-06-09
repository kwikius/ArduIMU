

#include "accelerometer.hpp"
#include <quan/three_d/sign_adjust.hpp>

namespace {

   QUAN_QUANTITY_LITERAL(acceleration,m_per_s2);

   quan::three_d::vect<quan::acceleration::m_per_s2>
   constexpr earth_gravity{0.0_m_per_s2,0.0_m_per_s2,-quan::acceleration::g};

   quan::three_d::vect<quan::acceleration::m_per_s2> acc_vector = earth_gravity;

   quan::three_d::vect<int> constexpr acc_vector_sign = {
      -1,
      1,
      -1
   };
}

quan::three_d::vect<quan::acceleration::m_per_s2> const &
get_gravity_vector() { return earth_gravity;}

quan::three_d::vect<quan::acceleration::m_per_s2> const & 
get_accelerometer()
{
   return acc_vector;
}

void set_accelerometer( quan::three_d::vect<quan::acceleration::m_per_s2> const & v)
{
   acc_vector = sign_adjust(v,acc_vector_sign);
}