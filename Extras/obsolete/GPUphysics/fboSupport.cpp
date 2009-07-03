#include "GPU_physics.h"

// #define FBO_USE_NVIDIA_FLOAT_TEXTURE_EXTENSION  1
#include "fboSupport.h"

#ifdef FBO_USE_NVIDIA_FLOAT_TEXTURE_EXTENSION

/* nVidia float texture extension */
#define FBO_R_HALFFLOAT     GL_FLOAT_R16_NV
#define FBO_RG_HALFFLOAT    GL_FLOAT_RG16_NV
#define FBO_RGB_HALFFLOAT   GL_FLOAT_RGB16_NV
#define FBO_RGBA_HALFFLOAT  GL_FLOAT_RGBA16_NV

#define FBO_R_FLOAT         GL_FLOAT_R32_NV
#define FBO_RG_FLOAT        GL_FLOAT_RG32_NV
#define FBO_RGB_FLOAT       GL_FLOAT_RGB32_NV
#define FBO_RGBA_FLOAT      GL_FLOAT_RGBA32_NV

#else

/* ATI float texture extension */
#define FBO_R_HALFFLOAT     GL_LUMINANCE_FLOAT16_ATI
#define FBO_RG_HALFFLOAT    GL_LUMINANCE_ALPHA_FLOAT16_ATI
#define FBO_RGB_HALFFLOAT   GL_RGB_FLOAT16_ATI
#define FBO_RGBA_HALFFLOAT  GL_RGBA_FLOAT16_ATI

#define FBO_R_FLOAT         GL_LUMINANCE_FLOAT32_ATI
#define FBO_RG_FLOAT        GL_LUMINANCE_ALPHA_FLOAT32_ATI
#define FBO_RGB_FLOAT       GL_RGB_FLOAT32_ATI
#define FBO_RGBA_FLOAT      GL_RGBA_FLOAT32_ATI

#endif

static void checkFrameBufferStatus ()
{
  GLenum status ;

  status = glCheckFramebufferStatusEXT ( GL_FRAMEBUFFER_EXT ) ;

  switch ( status )
  {
    case GL_FRAMEBUFFER_COMPLETE_EXT :
      break ;

    case GL_FRAMEBUFFER_UNSUPPORTED_EXT :
      fprintf ( stderr, "ERROR: Unsupported FBO setup.\n" ) ;
      exit ( 1 ) ;

    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT          :
      fprintf ( stderr, "WARNING: Incomplete FBO attachment.\n" ) ;
      break ;

    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT  :
      fprintf ( stderr, "WARNING: Incomplete FBO - missing attachment.\n" ) ;
      break ;

#ifdef GL_FRAMEBUFFER_INCOMPLETE_DUPLICATE_ATTACHMENT_EXT
    case GL_FRAMEBUFFER_INCOMPLETE_DUPLICATE_ATTACHMENT_EXT:
      fprintf ( stderr, "WARNING: Incomplete FBO - duplicate attachment.\n" ) ;
      break ;
#endif

    case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT          :
      fprintf ( stderr, "WARNING: Incomplete FBO - improper dimensions.\n" ) ;
      break ;

    case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT             :
      fprintf ( stderr, "WARNING: Incomplete FBO - improper formats.\n" ) ;
      break ;

    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT         :
      fprintf ( stderr, "WARNING: Incomplete FBO draw buffer.\n" ) ;
      break ;

    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT         :
      fprintf ( stderr, "WARNING: Incomplete FBO read buffer.\n" ) ;
      break ;

    default :
      fprintf ( stderr, "WARNING: Unexpected FBO status : 0x%04x\n", status ) ;
      break ;
  }
}



FrameBufferObject::FrameBufferObject ( int         _width ,
                                       int         _height,
                                       int         _ncomps,
                                       fboDataType _type  )
{
  if ( (_width & (_width-1)) == 0 )
    width  = _width  ;
  else
  {
    fprintf ( stderr, "FBO: Non-power of two width!\n" ) ;
    width  = 512 ;
  }

  if ( (_height & (_height-1)) == 0 )
    height  = _height  ;
  else
  {
    fprintf ( stderr, "FBO: Non-power of two height!\n" ) ;
    width  = 512 ;
  }

  ncomps = _ncomps ;
  type   = _type   ;

  switch ( ncomps )
  {
    case 1 : format = GL_LUMINANCE       ; break ;
    case 2 : format = GL_LUMINANCE_ALPHA ; break ;
    case 3 : format = GL_RGB             ; break ;
    case 4 : format = GL_RGBA            ; break ;
    default: fprintf ( stderr, "Illegal number of components"
                               " in a FrameBufferObject.\n" ) ;
             ncomps = 4 ; format = GL_RGBA ; break ;
  }


  switch ( type )
  {
    case FBO_BYTE           :
    case FBO_UNSIGNED_BYTE  : iformat = format ; break ;

    case FBO_INT            :
    case FBO_UNSIGNED_INT   :
      fprintf ( stderr, "FBO: GL_INT/GL_UINT textures are unsupported,"
                        " truncating to 16 bits per component" ) ;
      /* FALL THROUGH */
    case FBO_SHORT          :
    case FBO_UNSIGNED_SHORT :
      switch ( ncomps )
      {
        case 1 : iformat = GL_LUMINANCE16         ; break ;
        case 2 : iformat = GL_LUMINANCE16_ALPHA16 ; break ;
        case 3 : iformat = GL_RGB16               ; break ;
        case 4 : iformat = GL_RGBA16              ; break ;
      }
      break ;

    case FBO_DOUBLE :
      fprintf ( stderr, "FBO: GL_DOUBLE textures are unsupported,"
                        " truncating to GL_FLOAT per component" ) ;
      /* FALL THROUGH */
    case FBO_FLOAT  :
      switch ( ncomps )
      {
        case 1 : iformat = FBO_R_FLOAT           ; break ;
        case 2 : iformat = FBO_RG_FLOAT          ; break ;
        case 3 : iformat = FBO_RGB_FLOAT         ; break ;
        case 4 : iformat = FBO_RGBA_FLOAT        ; break ;
      }
      break ;

    case FBO_HALF   :
      switch ( ncomps )
      {
        case 1 : iformat = FBO_R_HALFFLOAT       ; break ;
        case 2 : iformat = FBO_RG_HALFFLOAT      ; break ;
        case 3 : iformat = FBO_RGB_HALFFLOAT     ; break ;
        case 4 : iformat = FBO_RGBA_HALFFLOAT    ; break ;
      }
      break ;

    default :
      fprintf ( stderr, "FBO: Unsupported data type?!?" ) ;
      break ;
  }

fprintf(stderr,"@a\n" ) ;
  glGenTextures ( 1, & textureHandle ) ;
fprintf(stderr,"@b\n" ) ;
  glBindTexture   ( GL_TEXTURE_2D, textureHandle ) ;
fprintf(stderr,"@c\n" ) ;
  glTexParameterf ( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST ) ;
fprintf(stderr,"@d\n" ) ;
  glTexParameterf ( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST ) ;
fprintf(stderr,"@e\n" ) ;
  //glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE ) ;
  //glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE ) ;
  fillTexture ( (void *) NULL ) ;
fprintf(stderr,"@f\n" ) ;

  glGenFramebuffersEXT  ( 1, & fboHandle     ) ;
fprintf(stderr,"@g\n" ) ;
  glBindFramebufferEXT  ( GL_FRAMEBUFFER_EXT, fboHandle ) ;
fprintf(stderr,"@h\n" ) ;

  glFramebufferTexture2DEXT ( GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
                              GL_TEXTURE_2D, textureHandle, 0 ) ;
fprintf(stderr,"@i\n" ) ;

#ifdef NEED_DEPTH_BUFFER
  static GLuint depth_rb = 0 ;

  if ( depth_rb == 0 )
  {
fprintf(stderr,"@j\n" ) ;
    glGenRenderbuffersEXT        ( 1, & depth_rb ) ;
    glBindRenderbufferEXT        ( GL_RENDERBUFFER_EXT, depth_rb ) ;
    glRenderbufferStorageEXT     ( GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24,
                                                        width, height ) ;
    glFramebufferRenderbufferEXT ( GL_FRAMEBUFFER_EXT,  GL_DEPTH_ATTACHMENT_EXT,
                                   GL_RENDERBUFFER_EXT, depth_rb ) ;
  }
  else
{
fprintf(stderr,"@k\n" ) ;
    glFramebufferRenderbufferEXT ( GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
                                   GL_RENDERBUFFER_EXT, depth_rb ) ;
}
#else
fprintf(stderr,"@l\n" ) ;
  glDisable ( GL_DEPTH_TEST ) ;
  glDepthMask ( 0 ) ;
fprintf(stderr,"@m\n" ) ;
  glFramebufferRenderbufferEXT ( GL_FRAMEBUFFER_EXT , GL_DEPTH_ATTACHMENT_EXT,
                                 GL_RENDERBUFFER_EXT, 0 ) ;
fprintf(stderr,"@n\n" ) ;
#endif

#ifdef NEED_STENCIL_BUFFER
  static GLuint stencil_rb = 0 ;

  if ( stencil_rb == 0 )
  {
fprintf(stderr,"@o\n" ) ;
    glGenRenderbuffersEXT        ( 1, & stencil_rb ) ;
    glBindRenderbufferEXT        ( GL_RENDERBUFFER_EXT, stencil_rb ) ;
    glRenderbufferStorageEXT     ( GL_RENDERBUFFER_EXT, GL_STENCIL_INDEX,
                                                        width, height ) ;
    glFramebufferRenderbufferEXT ( GL_FRAMEBUFFER_EXT , GL_STENCIL_ATTACHMENT_EXT,
                                   GL_RENDERBUFFER_EXT, stencil_rb ) ;
  }
  else
    glFramebufferRenderbufferEXT ( GL_FRAMEBUFFER_EXT , GL_STENCIL_ATTACHMENT_EXT,
                                   GL_RENDERBUFFER_EXT, stencil_rb ) ;
#else
fprintf(stderr,"@p\n" ) ;
  glDisable ( GL_STENCIL_TEST ) ;
  glStencilMask ( 0 ) ;
fprintf(stderr,"@q\n" ) ;
  glFramebufferRenderbufferEXT ( GL_FRAMEBUFFER_EXT , GL_STENCIL_ATTACHMENT_EXT,
                                 GL_RENDERBUFFER_EXT, 0 ) ;
fprintf(stderr,"@r\n" ) ;
#endif

  // Check framebuffer completeness at the end of initialization.

  checkFrameBufferStatus () ;
fprintf(stderr,"@s\n" ) ;
  restoreFrameBuffer () ;
fprintf(stderr,"@t\n" ) ;
}



void FrameBufferObject::fetchTexture ( void *data )
{
  glBindTexture ( GL_TEXTURE_2D, textureHandle ) ;
  glGetTexImage ( GL_TEXTURE_2D, 0,             /* MIP level...zero  */
                                format,        /* External format   */
                                type,          /* Data type         */
                                data           /* Image data        */ ) ;
}



void FrameBufferObject::fetchTexture ( unsigned char *data )
{
  if ( type != FBO_UNSIGNED_BYTE )
  {
    fprintf ( stderr, "FBO: Data format mismatch!" ) ;
    return ;
  }

  fetchTexture ( (void *)data ) ;
}



void FrameBufferObject::fetchTexture ( unsigned short *data )
{
  if ( type != FBO_UNSIGNED_SHORT )
  {
    fprintf ( stderr, "FBO: Data format mismatch!" ) ;
    return ;
  }

  fetchTexture ( (void *)data ) ;
}



void FrameBufferObject::fetchTexture ( float *data )
{
  if ( type != FBO_FLOAT )
  {
    fprintf ( stderr, "FBO: Data format mismatch!" ) ;
    return ;
  }

  fetchTexture ( (void *)data ) ;
}



void FrameBufferObject::fillTexture ( void *data )
{
  glBindTexture( GL_TEXTURE_2D, textureHandle ) ;
  glTexImage2D ( GL_TEXTURE_2D, 0,             /* MIP level...zero  */
                                iformat,       /* Internal format   */
                                width, height, /* Size              */
                                0,             /* Border...false    */
                                format,        /* External format   */
                                type,          /* Data type         */
                                data           /* Image data        */ ) ;
}



void FrameBufferObject::fillTexture ( unsigned char *data )
{
  if ( type != FBO_UNSIGNED_BYTE )
  {
    fprintf ( stderr, "FBO: Data format mismatch!" ) ;
    return ;
  }

  fillTexture ( (void *)data ) ;
}

void FrameBufferObject::fillTexture ( unsigned short *data )
{
  if ( type != FBO_UNSIGNED_SHORT )
  {
    fprintf ( stderr, "FBO: Data format mismatch!" ) ;
    return ;
  }

  fillTexture ( (void *)data ) ;
}


void FrameBufferObject::fillTexture ( float *data )
{
  if ( type != FBO_FLOAT )
  {
    fprintf ( stderr, "FBO: Data format mismatch!" ) ;
    return ;
  }

  fillTexture ( (void *)data ) ;
}


void FrameBufferObject::prepare ( bool clear )
{
  makeDestination () ;

  glViewport        ( 0, 0, width, height ) ;

  if ( clear ) glClear ( GL_COLOR_BUFFER_BIT ) ;

  glMatrixMode      ( GL_PROJECTION ) ;
  glLoadIdentity    () ;
  glOrtho           ( -1, 1, -1, 1, -1, 1 ) ;

  glMatrixMode      ( GL_MODELVIEW ) ;
  glLoadIdentity    () ;

  glDisable         ( GL_DEPTH_TEST ) ;
  glDisable         ( GL_CULL_FACE  ) ;
  glDisable         ( GL_BLEND      ) ;
}


void FrameBufferObject::fill ()
{
  float s_min = 0.5f / (float) width ;
  float s_max = (((float) width) - 0.5f) / (float) width ;
  float t_min = 0.5f / (float) height ;
  float t_max = (((float) height) - 0.5f) / (float) height ;

  glBegin ( GL_QUADS ) ;
  glTexCoord2f ( s_min, t_min ) ; glVertex2f ( -1, -1 ) ;
  glTexCoord2f ( s_max, t_min ) ; glVertex2f (  1, -1 ) ;
  glTexCoord2f ( s_max, t_max ) ; glVertex2f (  1,  1 ) ;
  glTexCoord2f ( s_min, t_max ) ; glVertex2f ( -1,  1 ) ;
  glEnd () ;
}


