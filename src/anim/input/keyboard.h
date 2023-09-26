/* FILE NAME  : keyboard.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 17.07.2022
 * PURPOSE    : Input system.
 *              Keyboard handle
 */

#ifndef __keyboard_h_
#define __keyboard_h_

#include "../../def.h"

/* My NSF namespace */
namespace nirt
{
  /* Keyboard handle class */
  class keyboard
  {
  protected:
      unsigned char KeysOld[256]; // State of keys on the current frame
  public:
    unsigned char
      Keys[256],         // State of keys on the current frame
      KeysClick[256];    // Signs of a single click of the keyboard

    /* Class constructor */
    keyboard( void )
    {
      ZeroMemory(Keys, 256);
      ZeroMemory(KeysClick, 256);
      ZeroMemory(KeysOld, 256);
    } /* End of 'keyboard' constructor */

    /* Responce function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Responce( void )
    {
      int i;

      (void)GetKeyboardState(Keys);
      
      for (i = 0; i < 256; i++)
      {
        Keys[i] >>= 7;
        KeysClick[i] = Keys[i] && !KeysOld[i];
      }
      memcpy(KeysOld, Keys, 256);
    } /* End of 'Responce' function */
  }; /* End of 'keyboard' class */
} /* End of 'nirt' namespace */

#endif /* __keyboard_h_ */

/* END OF 'keyboard.h' FILE */