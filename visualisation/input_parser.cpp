
#include <fstream>
#include <sstream>
#include <quan/three_d/vect.hpp>
#include <quan/serial_port.hpp>

namespace {
   typedef quan::three_d::vect<float> vect;
   std::string sp_input_string;

   /** sp_input_string may contains <sensor> x y z value
   * /param[out] out The vector to parse the value to 
   * \return 1 for mag 2 for acc 4 for gyro or 0 for nothing to out
   */
   int parse_string( quan::three_d::vect<float> & out)
   {
      std::istringstream istr(sp_input_string);
      std::string id;
      int result = 0;
      istr >> id;
      if ( id == "mag" ){
         result = 1;
      }else if ( id == "acc" ){
         result = 2;
      }else if ( id == "gyr" ){
         result = 4;
      }
      if ( result != 0){
         istr >> out.x >> out.y >> out.z;
      }
      sp_input_string = "";
      return result;
   }
}

/** read serial port. 
*
* \note Continues while serial port has data
* \param[in] sp Valid serial port
* \param[out] out vector to put the result of parse packet
* \return 1 for mag 2 for acc 4 for gyro and result put to out, 0 for anything else (e.g middle of packet)
*/
int parse_sp(quan::serial_port& sp, quan::three_d::vect<float> & out)
{
   while ( sp.in_avail() ){
      unsigned char ch;
      sp.read(&ch,1);
      if ( static_cast<char>(ch) == '\n'){
         auto const result = parse_string(out);
         if (result != 0){
            return result;
         }
      }else{
         sp_input_string += static_cast<char>(ch);
      } 
   }
   return 0;
}
