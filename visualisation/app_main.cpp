

#include <serial_port.hpp>
#include <quanGL.hpp>

int main(int argc, char** argv) {

   if ( open_serial_port()){
      init_algorithm();
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

      close_serial_port();
   } else{
      return 1;
   }
}