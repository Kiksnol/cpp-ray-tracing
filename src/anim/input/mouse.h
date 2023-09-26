/* FILE NAME  : mouse.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 17.07.2022
 * PURPOSE    : Input system.
 *              Mouse handle.
 */

#ifndef __mouse_h_
#define __mouse_h_

#include "../../def.h"

 /* My NSF namespace */
namespace nirt
{
  /* Mouse handle class */
  class mouse
  {
  protected:
    HWND hWnd1;
  public:
    int Mx, My, Mz,   // Mouse current coordinates
      Mdx, Mdy;     // Mouse delta coordinates
    int &Wheel, Mdz;  // Wheel information

    /* Class constructor */
    mouse( int &WheelRef, HWND hWnd0 ) : Mx(0), Mdx(0), My(0), Mdy(0), Wheel(WheelRef), Mdz(0), Mz(0), hWnd1(hWnd0)
    {
    } /* End of 'mouse' constructor */


    /* Responce function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Responce( void )
    {
      POINT pt;

      GetCursorPos(&pt);
      ScreenToClient(hWnd1, &pt);

      /* Transform coordinates on the screen */
      Mdx = pt.x - Mx;
      Mdy = pt.y - My;
      Mdz = Wheel;
      /* absolute val */
      Mx = pt.x;
      My = pt.y;
      Mz += Wheel;

      Wheel = 0;
    } /* End of 'Responce' function */
  }; /* End of 'mouse' class */
} /* End of 'nirt' namespace */

#endif /* __mouse_h_ */

/* END OF 'mouse.h' FILE */