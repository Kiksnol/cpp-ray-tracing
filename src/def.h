/* FILE NAME  : def.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 23.07.2021
 * PURPOSE    : Default header.
 */

#ifndef __def_h_
#define __def_h_

#include <iostream>
#include <cassert>

#ifndef WIN32
#define WIN32
#undef WIN32
#endif /* WIN32 */

#pragma comment(lib, "opengl32")
#pragma comment(lib, "glu32")

#define GLEW_STATIC
#include <glew.h>

#include <wglew.h>
#include <wglext.h>

#include "mth/mth.h"

/* Project namespace */
namespace nirt
{
  // Forward declaration
  class anim;
  /* Useful types */
  typedef mth::fcamera camera;
  typedef mth::fvec2 vec2;
  typedef mth::fvec3 vec3;
  typedef mth::fvec4 vec4;
  typedef mth::fmatr matr;
}

/* Debug memory allocation support */
#ifndef NDEBUG 
# define _CRTDBG_MAP_ALLOC
# include <crtdbg.h>
# define SetDbgMemHooks() \
  _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CRTDBG_CHECK_ALWAYS_DF | \
  _CRTDBG_ALLOC_MEM_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG))

static class __Dummy
{
public:
  /* Class constructor */
  __Dummy( void )
  {
    SetDbgMemHooks();
  } /* End of '__Dummy' constructor */
} __ooppss;
#endif /* _DEBUG */ 

#ifdef _DEBUG 
# ifdef _CRTDBG_MAP_ALLOC 
#   define new new(_NORMAL_BLOCK, __FILE__, __LINE__) 
# endif /* _CRTDBG_MAP_ALLOC */ 
#endif /* _DEBUG */

#endif /* __def_h_ */
