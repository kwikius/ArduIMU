#include <random>
#include <quan/three_d/out/vect.hpp>
#include <quan/three_d/rotation.hpp>
#include <quan/out/angle.hpp>

#include <iostream>
#include <fstream>

quan::angle::deg 
modulo( quan::angle::deg const & in)
{
   auto constexpr one_rev = quan::angle::deg{360};
   auto out = in;
   while ( out >= one_rev) {
       out -= one_rev;
   }
   while (out < quan::angle::deg{0} ){
      out += one_rev;
   }
   return out;
}

bool create_sim_mag_data(

    double const & sphere_radius, 
    quan::three_d::vect<double> const &  sphere_offset,
    double const & radius_random_ratio,
    std::size_t num_points,
    std::vector<quan::three_d::vect<double> > & points_out
    // TODO x y z scales ( elliptical error)
){

   typedef quan::three_d::vect<double> vect;

   // generate a random angle in degrees
   std::random_device rd{};
   std::mt19937 gen{rd()};
   std::uniform_real_distribution<> random_angle(0.0,359.999);

   std::uniform_real_distribution<> random_radius(
      sphere_radius * (1.0-radius_random_ratio),
      sphere_radius * (1.0+radius_random_ratio)
   );

   // TODO
   // introduce ellipticality scale x, y ,z after rotation

   quan::three_d::vect<quan::angle::deg> rotation;
   for ( auto i = 0U ; i < num_points; ++i){

      vect base_vect{random_radius(gen),0,0};

      rotation.x = modulo(rotation.x + quan::angle::deg{random_angle(gen)});
      rotation.y += modulo(rotation.x + quan::angle::deg{random_angle(gen)});
      rotation.z += modulo(rotation.x + quan::angle::deg{random_angle(gen)});

      quan::three_d::x_rotation rx(rotation.x);
      quan::three_d::y_rotation ry(rotation.y);
      quan::three_d::z_rotation rz(rotation.z);

      auto const rotated_vect = rz(ry(rx(base_vect)));
      auto const measured_vect = rotated_vect + sphere_offset;

      points_out.push_back(measured_vect);
   }


   return true;

}

#if 0
int main()
{

/*
   create a simulated magnetometer data set
*/
   typedef quan::three_d::vect<double> vect;

   constexpr vect sphere_offset{15,20,-7};  
   double constexpr sphere_radius = 33.0; // to be uT

   quan::three_d::vect<quan::angle::deg> rotation;

   // generate a random angle in degrees
   std::random_device rd{};
   std::mt19937 gen{rd()};
   std::uniform_real_distribution<> random_angle(0.0,359.999);

   double diff_ratio = 0.05;
   std::uniform_real_distribution<> random_radius(sphere_radius * (1.0-diff_ratio),sphere_radius * (1.0+diff_ratio));

   // TODO
   // introduce ellipticality scale x, y ,z after rotation

   // number of points
   auto n = 100U;

   std::ofstream fout("compass_data.txt");
   std::ostream & out = fout;

   out << "{\n";
   for ( auto i = 0U ; i < n; ++i){

      if ( i != 0){
         out << ",\n";
      }

      vect base_vect{random_radius(gen),0,0};
      std::cout << "r = " << base_vect.x <<'\n';

      rotation.x = modulo(rotation.x + quan::angle::deg{random_angle(gen)});
      rotation.y += modulo(rotation.x + quan::angle::deg{random_angle(gen)});
      rotation.z += modulo(rotation.x + quan::angle::deg{random_angle(gen)});

      quan::three_d::x_rotation rx(rotation.x);
      quan::three_d::y_rotation ry(rotation.y);
      quan::three_d::z_rotation rz(rotation.z);

      auto const rotated_vect = rz(ry(rx(base_vect)));
      auto const measured_vect = rotated_vect + sphere_offset;
      
      out << "   { " << measured_vect.x << ", " << measured_vect.y  << ", " << measured_vect.z << " } /* r = " << base_vect.x << " */" ;
      
   }

   out << "\n};\n";

   std::cout << "done\n";

 
}

#endif



     