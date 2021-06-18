

#include <serial_port.hpp>
#include <joystick.hpp>
#include <quanGL.hpp>
#include <stdexcept>

namespace {
  joystick * p_joystick = nullptr;
}

const joystick & get_joystick()
{
  if (p_joystick == nullptr){
    throw std::logic_error("joystick not available");
  }
  return *p_joystick;
}

namespace {

   void set_lighting()
   {
      float position[3] = {0.25,-1,-1};
      float ambient[4]= {0.1,0.1,0.1,1};
      float diffuse[4] = {0.7,0.7,0.7,1};
      float specular[4] = {0.2,0.2,0.2,1};
      float emission[4] = {0.1,0.1,0.1,1};
      float ambient_light_model[4] = {0.2,0.2,0.2,1.0};

      glLightfv(GL_LIGHT0, GL_POSITION, position);
      glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
      glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
      glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);

      glLightModelfv(GL_LIGHT_MODEL_AMBIENT,ambient_light_model);
      glEnable(GL_LIGHTING);
      glEnable(GL_LIGHT0);

      glEnable(GL_COLOR_MATERIAL);

      glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
      glMaterialfv( GL_FRONT_AND_BACK,GL_SPECULAR,specular);
      glMaterialfv( GL_FRONT_AND_BACK,GL_EMISSION,emission);
   }
}

int main(int argc, char** argv) {

   
   if ( !use_serial_port() || open_serial_port()){

      if (use_joystick()){
         p_joystick = new joystick("/dev/input/js0");
      }
      glutInit(&argc, argv);
      glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );
      
      glutInitWindowPosition(80, 80);
      glutInitWindowSize(800, 500);
      glutCreateWindow(get_title());
      glutReshapeFunc(reshape);
      glutDisplayFunc(display);
      glutKeyboardFunc( onKeyboard);
      glutIdleFunc(onIdle);
      glEnable(GL_CULL_FACE);
      glEnable(GL_DEPTH_TEST);
      glDepthMask(GL_TRUE);
      glDepthRange(-1,1);
      glDepthFunc(GL_LESS);  
      set_lighting();

      glutMainLoop();
      if (use_serial_port()){
         close_serial_port();
      }
      if ( use_joystick()){
         delete p_joystick;
         p_joystick = nullptr;
      }
   } else{
      return 1;
   }
}