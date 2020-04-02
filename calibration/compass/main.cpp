
#include <vector>
#include <iostream>
#include <cassert>
#include <fstream>

//https://www.geometrictools.com/
#include <GteApprSphere3.h>

#include <quan/three_d/out/vect.hpp>
#include <quan/three_d/sphere.hpp>

#include <quan/min.hpp>
#include <quan/max.hpp>

int get_sphere(
   std::vector<quan::three_d::vect<double> > const & point_cloud,
   quan::three_d::sphere<double> & out, 
   int num_iters
)
{
   const auto length = point_cloud.size();
   auto points = new gte::Vector3<double> [length] ;
   assert( points != nullptr);
   for ( auto i = 0U; i < length ;++i){
      auto const & vect = point_cloud[i];
      points[i] = {vect.x,vect.y,vect.z};
   }
   gte::Sphere3<double> sphere;

   gte::ApprSphere3<double> v;
   auto result = v(
      length,  // number of points in set
      points,     // the address of the points set
      num_iters,      // number of iterations
      false,      // true to use average of points as initial centre
      sphere     // put result in circle object
    );

    delete [] points;
   
    out.centre.x = sphere.center[0];
    out.centre.y = sphere.center[1];
    out.centre.z = sphere.center[2];
    out.radius = sphere.radius;

    return result;
}

bool create_sim_mag_data(
    double const & sphere_radius, 
    quan::three_d::vect<double> const &  sphere_offset,
    double const & radius_random_ratio,
    std::size_t num_points,
    std::vector<quan::three_d::vect<double> > & points_cloud_out
    // TODO x y z scales
);

/*
  not the best. Relies on having points near min and max Really want to fit a 3d ellipse.
  For accuracy look at pointset in gnuplot to check coverage.
*/
quan::three_d::vect<double> calculate_extents(std::vector<quan::three_d::vect<double> > const & point_cloud)
{
      quan::three_d::vect<double> pmin;
      quan::three_d::vect<double> pmax;

      for ( auto const & p : point_cloud)
      {
          pmin.x = quan::min(p.x,pmin.x);
          pmax.x = quan::max(p.x,pmax.x);

          pmin.y = quan::min(p.y,pmin.y);
          pmax.y = quan::max(p.y,pmax.y);

          pmin.z = quan::min(p.z,pmin.z);
          pmax.z = quan::max(p.z,pmax.z);
      }
      return pmax - pmin;
}

bool get_file_real_mag_data( std::ifstream & in, std::vector<quan::three_d::vect<double> > & points_cloud_out)
{
   while (in) {
      std::string str;
      quan::three_d::vect<double> v;
      in >> str;
      if (str == ""){continue;}
      if ( str == "mag"){
         in >> v.x >> v.y >> v.z;
         points_cloud_out.push_back(v);
      }

   }
      if (points_cloud_out.size() < 50){
         std::cout << "Warning : small number of points\n";
      }
   return true;
}

/**  Compass calibration algorithm
*    Read raw point data from file
*    Find extents in x, y, z
*    Use local magnetic field density data and extents to work out 
*    gain in z,y,z required to squish the data into a sphere
*    multiply points by gain
*    call sphere fit algorithm.
*/
int main(int argc, char const * argv[])
{
   std::vector<quan::three_d::vect<double> > point_cloud;

   if ( argc  < 2 ) {
      std::cout << "useage " << argv[0] << " <data_file>\n";
      return EXIT_SUCCESS;
   }

   std::ifstream in(argv[1]);

   if ( !in || in.fail()){
      std::cout << "input file \"" << argv[1] << "\" failed\n";
      return EXIT_FAILURE;
   }

  // fill the points_cloud with the data ...
  // Assume the nominal units are uT
#if 0
  // for testing create a simulated data set
  create_sim_mag_data(
      47.0,        // sphere radius
      quan::three_d::vect<double>{50,9,-20}, // offset
      0.1,         // radius random percentage error
      1000,        // number of data points to generate.
      point_cloud  // data points out
  );
#else
  get_file_real_mag_data(in,point_cloud);
#endif

  //--------------

  // find extents of x,y,z raw data axes, which represent the diameter at each axis at some scaling factor
  auto extents = calculate_extents(point_cloud);
  std::cout << "extents = " << extents <<'\n';
  
  // Get best estimate of earth magnetic flux density at the location ( and altitude) the data was sampled
  // https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
  // This tells us what the actual extents should be (extent.n == 2 * flux_density)
  static constexpr double geo_location_flux_density_uT = 49.0 ; // uT

  // From the extents and earth flux density,
  // calculate the sensor gain in each axis to scale from reading to actual magnetic flux density
  auto const gain =  (2 * geo_location_flux_density_uT) / extents ;
  
  // gain by which to multiply raw points
  std::cout << "gain = " << gain << "\n";

  // For computing the sphere radius and offset, scale the raw input data by gain of each axis
  for ( auto & p : point_cloud){
      p.x *= gain.x;
      p.y *= gain.y;
      p.z *= gain.z;
  }
  
  //Now compute centre and offset of scaled data
  quan::three_d::sphere<double> result{0,{0,0,0}};

  int n_iters = get_sphere(point_cloud,result,1000);
  std::cout << "n iters = " << n_iters <<'\n';
  std::cout << "result : r = " << result.radius << ", centre = " << result.centre <<'\n'; 

  std::ofstream out("output.dat");

  // remove the calculated offset from the scaled data
  // to get the best estimate of the unscaled vector for each point
  for ( auto const & p : point_cloud){
     auto const p1 = p - result.centre;
     out << p1.x << " " << p1.y << " " << p1.z << "\n";
  }

  std::cout << "output written to output.dat\n";
  std::cout << "invoke \'gnuplot\'\n";
  std::cout << "type \'set view equal xyz\' to get equal size x y z axes.\n";
  std::cout << "type \'splot \"output.dat\"\' to view points in 3d graph.\n";
  std::cout << "Fly around graph with mouse\n";

  

  

}
