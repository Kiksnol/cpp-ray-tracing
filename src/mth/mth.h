/* FILE NAME  : mth.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 17.07.2022
 * PURPOSE    : Math support module definitions.
 */

#ifndef __mth_h_
#define __mth_h_

#include "mth_vec2.h"
#include "mth_vec3.h"
#include "mth_vec4.h"
#include "mth_matr.h"
#include "mth_camera.h"

/* Math namespace */
namespace mth
{
  /* Random 0 and 1 functions */
  float Rnd0( void );
  float Rnd1( void );
}

#endif /* __mth_h_ */

/* END OF 'mth.h' FILE */
