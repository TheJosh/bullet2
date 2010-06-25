#ifndef __GL_RENDER_H__
#define __GL_RENDER_H__

//#include "Sph.hpp"
#include  "BulletFluilds/btFluid.h"

// mouse controls
extern int mouseOldX;
extern int mouseOldY;
extern int mouseButtons;
extern float rotateX;
extern float rotateY;
extern float translateZ;
extern float translateX;
extern float translateY;

void render( btSphFluid * fluid );

void preInitGL( int argc, char ** argv );
void goGL( btSphFluid * fluid );
int getVBO( std::string );

#endif

