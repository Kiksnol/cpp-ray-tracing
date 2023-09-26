/* FILE NAME  : mthdef.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 17.07.2022
 * PURPOSE    : Math support module definitions.
 */

#ifndef __mthdef_h_
#define __mthdef_h_

#include <cmath>

#include "../def.h"

/* Pi math constant */
#define PI 3.14159265358979323846

/* Degrees to radians conversion */
#define D2R(A) ((A) * (PI / 180.0))
#define Degree2Radian(a) D2R(a)

#define R2D(A) ((A) * (180.0 / PI))
#define Radian2Degree(A) R2D(A)

/* Double and float useful types */
typedef double double;
typedef float float;  // Interpolate(linear) A * (1 - T) + B * T

/* Math namespace */
namespace mth
{
  // Forward declarations
  template<typename Type> class vec3;  /* 3D vector class */
  template<typename Type1> class matr; /* Matrix class */

  /* Clamp function.
   * ARGUMENTS:
   *   - value to clamp:
   *       Type X;
   *   - min value:
   *       Type Mi;
   *   - max value:
   *       Type Ma;
   * RETURNS: (Type) clamped X
   */
  template <typename Type>
  Type Clamp( Type X, Type Mi = 0, Type Ma = 1 )
  {
    if (X < Mi)
      return Mi;
    if (X > Ma)
      return Ma;
    return X;
  } /* End of 'Clamp' function */

  /* Linear Interpolate function.
   * ARGUMENTS:
   *   - values to interpolate:
   *       Type A, B;
   *   - factor to interpolate:
   *       Type T;
   * RETURNS: (Type) interpolated value
   */
  template<typename Type>
  Type LinearInterpolate( Type A, Type B, Type T )
  {
    return A * (1 - T) + B * T;
  } /* End of 'LinearInterpolate' function */
} /* End of 'mth' namespace */

#endif /* __mthdef_h_ */

/* END OF 'mthdef.h' FILE */