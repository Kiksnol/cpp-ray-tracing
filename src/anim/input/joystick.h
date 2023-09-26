/* FILE NAME  : joystick.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 17.07.2022
 * PURPOSE    : Input system.
 *              Joystick handle.
 */

#ifndef __joystick_h_
#define __joystick_h_

#include "../../def.h"

#include <mmsystem.h>
#pragma comment(lib, "winmm")

#define ND4_GET_JOYSTIC_AXIS(A) \
   (2.0 * (ji.dw ## A ## pos - jc.w ## A ## min) / (jc.w ## A ## max - jc.w ## A ## min) - 1)

 /* My NSF namespace */
namespace nirt
{
  /* Mouse handle class */
  class joystick
  {
  public:
    unsigned char
      JBut[32], JButOld[32], JButClick[32]; // Joystick button states
    int JPov;                               // Joystick point-of-view control [-1, 0..7]
    double
      JoyX, JoyY, JoyZ, JoyR;               // Joystick axes

    /* Class constructor */
    joystick( void ) : JPov(0), JoyX(0.0), JoyY(0.0), JoyZ(0.0), JoyR(0.0)
    {
      ZeroMemory(JBut, 32);
      ZeroMemory(JButOld, 32);
      ZeroMemory(JButClick, 32);
    } /* End of 'joystick' constructor */

    /* Responce function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Responce( void )
    {
      int i;

      if (joyGetNumDevs() > 0)
      {
        JOYCAPS jc;

        /* get joysick info */
        if (joyGetDevCaps(JOYSTICKID1, &jc, sizeof(jc)) == JOYERR_NOERROR)
        {
          JOYINFOEX ji;

          ji.dwSize = sizeof(JOYINFOEX);
          ji.dwFlags = JOY_RETURNALL;
          if (joyGetPosEx(JOYSTICKID1, &ji) == JOYERR_NOERROR)
          {
            /* buttons */
            for (i = 0; i < 32; i++)
            {
              JBut[i] = (ji.dwButtons >> i) & 1;
              JButClick[i] = JBut[i] && !JButOld[i];
              JButOld[i] = JBut[i];
            }
            /* axes */
            JoyX = ND4_GET_JOYSTIC_AXIS(X);
            JoyY = ND4_GET_JOYSTIC_AXIS(Y);
            JoyZ = ND4_GET_JOYSTIC_AXIS(Z);
            JoyR = ND4_GET_JOYSTIC_AXIS(R);
          }
        }
      }
    } /* End of 'Responce' function */
  }; /* End of 'joystick' class */
} /* End of 'nirt' namespace */

#endif /* __joystick_h_ */

/* END OF 'joystick.h' FILE */