
#include <fstream>
#include <sstream>
#include <quan/out/angle.hpp>
#include <quan/basic_matrix/basic_matrix.hpp>
#include <quan/three_d/out/vect.hpp>
#include <quan/serial_port.hpp>

typedef quan::three_d::vect<double> vect;

#if 0
/*
   axis angle
   axis is a unit vector
   angle = 0;
angle to quaternion

*/

//https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm
/*
template <typename T>
inline
quan::three_d::quat<typename quan::meta::binary_op<T,quan::meta::divides,T>::type> 
axis_angle_to_quat( quan::three_d::vect<T> const & axis, quan::angle::rad const & angle)
{
   const auto v = unit_vector(axis) * sin(angle/2);
   return {cos(angle/2),v.x,v.y,v.z};
}
*/

template <typename T>
inline constexpr
quan::angle::rad 
get_rotation_angle( quan::three_d::vect<T> const & vecta, quan::three_d::vect<T> const & vectb)
{
   return std::acos(
      dot_product(
         unit_vector(vecta),
         unit_vector(vectb)
      )
   );   
}

/*
before entry
    // check that unit_vector( a ) != unit_vector(b) 
    // check that unit_vector(a) - unit_vector(b)  != 0;
*/
template <typename T>
quan::three_d::vect<typename quan::meta::binary_op<T,quan::meta::divides,T>::type> 
get_rotation_axis(quan::three_d::vect<T> const & vecta, quan::three_d::vect<T> const & vectb)
{ 
    return unit_vector(cross_product(vecta,vectb));
}

/*
template <typename T> 
quan::three_d::quat<T> 
angle_to_quat ( quan::three_d::vect<T> a, quan::three_d::vect<T> const & b)
{
   return axis_angle_to_quat(
         get_rotation_axis(a,b),
         get_rotation_angle(a,b)
   );
}
*/

quan::basic_matrix<4,4,double>
axis_angle_to_rotation_matrix(quan::three_d::vect<double> const & u, quan::angle::rad const & theta)
{
    auto const ct = cos(theta);
    auto const _1_m_ct = 1. - ct;
    auto const st = sin(theta);
    auto const uxst = u.x * st;
    auto const uyst = u.y * st;
    auto const uxuy = u.x * u.y;
    auto const uxuz = u.x * u.z;
    auto const uyuz = u.y * u.z;
    auto const uz2 = u.z * u.z;
    auto const uzst = u.z*st;
    
    return {
         ct + (u.x * u.x) * _1_m_ct, uxuy * _1_m_ct - uzst, uxuz * _1_m_ct + uyst, 0.,
         uxuy * _1_m_ct + uzst , ct + uz2 * _1_m_ct    , uyuz *_1_m_ct - uxst ,    0.,
         uxuz * _1_m_ct - uyst, uyuz * _1_m_ct + uxst, ct + uz2 *  _1_m_ct ,       0.,
         0.                   , 0.                   , 0.                  ,       1.
    };
}

template <typename T>
quan::basic_matrix<4,4,double>
angle_to_rotation_matrix( quan::three_d::vect<T> const & a, quan::three_d::vect<T> const & b)
{
   return axis_angle_to_rotation_matrix(
      get_rotation_axis(a,b),
      get_rotation_angle(a,b)
   );
}
#endif

namespace {

 std::string sp_input_string;
 vect the_value;
}

// sp_input_string contains
// mag x y z
int parse_string()
{
   std::istringstream istr(sp_input_string);

   std::string id;
   int result = 0;
   istr >> id;

   if ( id == "mag"){
      istr >> the_value.x >> the_value.y >> the_value.z ;
      // sign fix up
      // the_value.x *= -1;
      the_value.y *= -1;
      //the_value.z *= -1;
      result = 1;
   }

   sp_input_string = "";
   return result;
 /*
   while ( sp_input_string.length() ){
      if ( isspace(sp_input_string.at(0))){
         sp_input_string = sp_input_string.substr(1,sp_input_string.npos);
      }else{
         break;
      }
   }
   int result = 0;
   if ( sp_input_string.length() > 20){
   // magC_uT
      if ( sp_input_string.substr(0,7) == "val = ["){
           auto const pos = sp_input_string.find(']');
           if (pos != sp_input_string.npos){
               sp_input_string = sp_input_string.substr(7,pos-7);
               for ( int i = 0; i < 3; ++i){
                  auto const pos = sp_input_string.find("uT");
                  if (pos != sp_input_string.npos){
                     auto digits = sp_input_string.substr(0,pos-1);
                     if ( i < 2){
                        sp_input_string = sp_input_string.substr(pos+4,sp_input_string.npos);
                     }else{
                        sp_input_string = sp_input_string.substr(pos+2,sp_input_string.npos);
                     }
                    std::stringstream iostr;
                    iostr << digits;
                    iostr >> the_value[i];
                  }
               } 
                // sign fix up
                // the_value.x *= -1;
               the_value.y *= -1;
               //the_value.z *= -1;
               result = 1;
           }
      }
   }

   sp_input_string = "";
   return result;
   */
}

int parse_sp(quan::serial_port& sp, quan::three_d::vect<double> & out)
{
    while ( sp.in_avail() ){
      unsigned char ch;
      sp.read(&ch,1);
      if ( static_cast<char>(ch) == '\n'){
        if (parse_string() != 0){
            out = the_value;
            return 1;
        }
      }else{
         sp_input_string += static_cast<char>(ch);
      } 
    }
    return 0;
}
