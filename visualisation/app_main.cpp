

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

int main(int argc, char** argv) {

   
   if ( !use_serial_port() || open_serial_port()){

      if (use_joystick()){
         p_joystick = new joystick("/dev/input/js0");
      }
      glutInit(&argc, argv);
      glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
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