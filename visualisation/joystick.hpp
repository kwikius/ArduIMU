#ifndef ARDUIMU_VISUALISATION_JOYSTICK_HPP_INCLUDED
#define ARDUIMU_VISUALISATION_JOYSTICK_HPP_INCLUDED

#include <quan/joystick.hpp>
#include <quan/three_d/vect.hpp>

struct joystick{
   joystick(const char* dev) : m_js(dev){}

   static constexpr double joystick_half_range = 32767.0;
   static constexpr uint8_t roll_idx = 0;  // roll on ch 0
   static constexpr uint8_t pitch_idx = 1; // pitch on ch 1
   static constexpr uint8_t throttle_idx = 2;
   static constexpr uint8_t yaw_idx = 3;   // yaw on ch 3

   /**
   @brief joystick channel direction , either 1 or -1
   **/
   static constexpr int32_t m_js_sign []= {
       1,   // roll
      -1,   // pitch
       1,   // throttle
      -1    // yaw
   };

   void update(quan::three_d::vect<double>& stick_percent)const
   {
   /**
   * @brief function to turn raw joystick number into a representation 
   * of the joystick axis in range  -1 to 1  ...
   **/

   /**
   * @brief ... so work out stick percent
   **/
      stick_percent.x =  get_js_percent(roll_idx);
      stick_percent.y =  get_js_percent(pitch_idx);
      stick_percent.z = get_js_percent(yaw_idx);
   }

  private:

   double get_js_percent(int32_t i)const {
      return static_cast<double>(m_js.get_channel(i) * m_js_sign[i]) / joystick_half_range ;
   };
   quan::joystick m_js;
};

joystick const & get_joystick();
bool use_joystick();

#endif // ARDUIMU_VISUALISATION_JOYSTICK_HPP_INCLUDED
