/* FILE NAME  : anim.cpp
 * PROGRAMMER : ND4
 * LAST UPDATE: 23.07.2021
 * PURPOSE    : Animation system class.
 *              Animation definition.
 */

#include "anim.h"

/* Rand 0 and 1 functions */
/* 0...1 */
float mth::Rnd0( void )
{
  srand(clock());
  return rand() / (float)RAND_MAX;
}

/* -1...1 */
float mth::Rnd1( void )
{
  srand(clock());
  return 2.0 * rand() / RAND_MAX - 1.0;
}

 /* Animation single tone */
nirt::anim nirt::anim::Instance;

/* Animation path */
std::string nirt::anim::Path;

/* END OF 'anim.cpp' FILE */