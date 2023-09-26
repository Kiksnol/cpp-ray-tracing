/* FILE NAME  : input.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 17.07.2022
 * PURPOSE    : Input system.
 *              Joystick handle.
 */

#ifndef __input_h_
#define __input_h_

#include "../../def.h"

#include "keyboard.h"
#include "joystick.h"
#include "mouse.h"

 /* My NSF namespace */
namespace nirt
{
  /* Mouse handle class */
  class input : public keyboard, public mouse, public joystick
  {
  public:
    /* Class constructor */
    input( int &WheelRef, HWND hWnd1 ) : mouse(WheelRef, hWnd1)
    {
    } /* End of 'input' constructor */

    /* Responce function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Responce( void )
    {
      keyboard::Responce();
      mouse::Responce();
      joystick::Responce();
    } /* End of 'Responce' function */
  }; /* End of 'input' class */
} /* End of 'nirt' namespace */

#endif /* __input_h_ */

/* END OF 'input.h' FILE */