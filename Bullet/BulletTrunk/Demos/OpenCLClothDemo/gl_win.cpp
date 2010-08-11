#include <GL/glew.h>

#include "clstuff.hpp"
#include "gl_win.hpp"

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <iterator>
#include <math.h>
#include <cmath>
#include <cstring>


#ifndef _WIN32
#include <GL/glx.h>
#endif //!_WIN32

#if defined(__APPLE__) || defined(__MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <CL/cl.hpp>

static GLuint vbo = 0;

#ifdef _WIN32
#include <windows.h>
#endif


static unsigned int windowWidth  = 1280;
static unsigned int windowHeight = 1024;

// mouse controls
int mouseOldX;
int mouseOldY;
int mouseButtons         = 0;

float rotateX;
float rotateY;

float translateZ;
float translateX;
float translateY;

static GLuint glProgram;


void doFlags();


void render( void)
{
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
//	glDisable ( GL_CULL_FACE );

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	glTranslatef( translateX, translateY, translateZ );
	glRotatef( rotateX, 0.5f , 0.0f, 0.0f );
	glRotatef( rotateY, 0.0f, 0.5f, 0.0f );

//	glDisable (GL_BLEND);

	doFlags();
	// TODO:
	//glBindBuffer(GL_ARRAY_BUFFER, vbo);
	//glVertexPointer(4, GL_FLOAT, 0, NULL);
    //glEnableClientState(GL_VERTEX_ARRAY);

	//glDrawArrays(GL_POINTS, 0, 4*4);

//	glDisableClientState(GL_VERTEX_ARRAY);
 //   glBindBuffer(GL_ARRAY_BUFFER, 0);


	glUseProgram(0);
}

static void initGL(void) 
{
	glClearColor( 0.05f, 0.0f, 0.1f, 0.1f );

#if 0
	GLfloat mat_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_shininess[] = { 50.0f };
	GLfloat light_position[] = { 
	   -10.f, 
	   5.f, 
	   -1.f, 
	   1.0f };

	glEnable ( GL_COLOR_MATERIAL );
	glShadeModel( GL_SMOOTH );
	glEnable( GL_LINE_SMOOTH );


	glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular );
	glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess );
	glLightfv( GL_LIGHT0, GL_POSITION, light_position );

	//glEnable( GL_LIGHTING );
	//glEnable( GL_LIGHT0 ); // Switch on and crashes!
	glEnable( GL_DEPTH_TEST );
#endif 
#if 0


   glEnable ( GL_COLOR_MATERIAL );
   glShadeModel( GL_SMOOTH );
   glEnable( GL_LINE_SMOOTH );

   glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular );
   glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess );
   glLightfv( GL_LIGHT0, GL_POSITION, light_position );

   glEnable( GL_LIGHTING );
   glEnable( GL_LIGHT0 );
   glEnable( GL_DEPTH_TEST );
#endif
   rotateX    = 0;
   rotateY    = 30;
   translateX = 0.0f;
   translateY = -30.0f;
   translateZ = -120.0;
}

void display(void)
{
	render();

	glutSwapBuffers();
	glutPostRedisplay();
}

void keyboard( unsigned char key, int /*x*/, int /*y*/)
{
  switch( key) {
  case('q') :
#ifdef _WIN32
  case VK_ESCAPE:
#endif //_WIN32
    exit(0);
  break;
  case('a'):
    translateY += 0.1f;
    break;
  case('z'):
    translateY -= 0.1f;
    break;
  case('d'):
    translateX += 0.1f;
    break;
  case('s'):
    translateX -= 0.1f;
    break;
  case('f'):
    translateZ += 0.1f;
    break;
  case('g'):
    translateZ -= 0.1f;
    break;
  }
}

void mouse(int button, int state, int x, int y)
{
  if (state == GLUT_DOWN) {
    mouseButtons |= 1<<button;
  } else if (state == GLUT_UP) {
    mouseButtons = 0;
  }

  mouseOldX = x;
  mouseOldY = y;
  glutPostRedisplay();
}

void motion(int x, int y)
{
  float dx, dy;
  dx = x - mouseOldX;
  dy = y - mouseOldY;
  
  if (mouseButtons & 1) {
    rotateX += dy * 0.2;
    rotateY += dx * 0.2;
  } 
  else if (mouseButtons & 5) {
    translateY -= dy * 0.01;
    translateX -= dx * 0.01;
  }
  else if (mouseButtons & 4) {
    translateZ += dy * 0.01;
  } 

  mouseOldX = x;
  mouseOldY = y;
}


void reshape (int w, int h)
{
	windowWidth  = w;
	windowHeight = h;
	glViewport(0, 0, windowWidth, windowHeight);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(
		60.0,
		(GLfloat)windowWidth / (GLfloat) windowHeight,
		0.1,
		600.0f );
}

void goGL(void)
{
	glutMainLoop();
}

void preInitGL(int argc, char ** argv)
{
	glutInit( &argc, argv );

	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );
	glutInitWindowSize( windowWidth, windowHeight );    
	glutCreateWindow ("OpenCL Renderer");

    glewInit();
    if (!glewIsSupported( "GL_VERSION_2_0 " )) {
          std::cerr
              << "Support for necessary OpenGL extensions missing."
              << std::endl;
          exit(EXIT_FAILURE);
    }

	initGL();

	glViewport( 0, 0, windowWidth, windowHeight);

	reshape( windowWidth, windowHeight );

	glutDisplayFunc(display); 
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
}

int getVBO( std::string, GLuint size )
{
	if (vbo == 0) {
		// Create VBO
		// create buffer object
		glGenBuffers(1, &vbo);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, size, 0, GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	return vbo;
}
