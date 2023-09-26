/* FILE NAME  : winmsg.cpp
 * PROGRAMMER : ND4
 * LAST UPDATE: 17.07.2022
 * PURPOSE    : WinAPI class.
 *              Window messages functions handlers
 */

#include "win.h"

/* Refresh timer constant */
#define REFRESH_TIMER  0x30102

/* My namespace */
namespace nirt
{
  /* WM_CREATE window message handle function.
   * ARGUMENTS:
   *   - structure with creation data:
   *       CREATESTRUCT *CS;
   * RETURNS:
   *   (int) TRUE to continue creation window, FALSE to terminate.
   */
  int win::OnCreate( CREATESTRUCT *CS )
  {
    SetTimer(hWnd, INIT_TIMER, 1, nullptr);
    return TRUE;
  } /* End of 'win::OnCreate' function */
  
  /* WM_DESTROY window message handle function.
   * ARGUMENTS: None.
   * RETURNS: None.
   */
  void win::OnDestroy( void )
  {
    if (IsInit)
    {
      Close();
      KillTimer(hWnd, REFRESH_TIMER);
    }
    else
      KillTimer(hWnd, INIT_TIMER);
    PostQuitMessage(30);
  } /* End of 'win::OnDestroy' function */
  
  
  /* WM_SIZE window message handle function.
   * ARGUMENTS:
   *   - sizing flag (see SIZE_***, like SIZE_MAXIMIZED)
   *       UINT State;
   *   - new width and height of client area:
   *       int W, H;
   * RETURNS: None.
   */
  void win::OnSize( UINT State, int W, int H )
  {
    this->W = W;
    this->H = H;
    if (IsInit)
      Resize();
  } /* End of 'win::OnSize' function */
  
  /* WM_ERASEBKGND window message handle function.
   * ARGUMENTS:
   *   - device context of client area:
   *       HDC hDC;
   * RETURNS:
   *   (int) TRUE if background is erased, FALSE otherwise.
   */
  int win::OnEraseBkgnd( HDC hDC )
  {
    if (IsInit)
      Erase(hDC);
    return TRUE;
  } /* End of 'win::OnEraseBkgnd' function */
  
  /* WM_PAINT window message handle function.
   * ARGUMENTS:
   *   - window device context:
   *       HDC hDC;
   *   - paint message structure pointer:
   *       PAINTSTRUCT *PS;
   * RETURNS: None.
   */
  void win::OnPaint( HDC hDC, PAINTSTRUCT *PS )
  {
    if (IsInit)
      Paint(hDC);
  } /* End of 'win::OnPaint' function */
  /* WM_ACTIVATE window message handle function.
   * ARGUMENTS:
   *   - reason (WA_CLICKACTIVE, WA_ACTIVE or WA_INACTIVE):
   *       UINT Reason;
   *   - handle of active window:
   *       HWND hWndActDeact;
   *   - minimized flag:
   *       int IsMinimized;
   * RETURNS: None.
   */
  void win::OnActivate( UINT Reason, HWND hWndActDeact, int IsMinimized )
  {
    IsActive = Reason == WA_CLICKACTIVE || Reason == WA_ACTIVE;
    if (IsInit)
      Activate();
  } /* End of 'win::OnActivate' function */
  
  /* WM_TIMER window message handle function.
   * ARGUMENTS:
   *   - specified the timer identifier.
   *       int Id;
   * RETURNS: None.
   */
  void win::OnTimer( int Id )
  {
    if (!IsInit)
    {
      KillTimer(hWnd, INIT_TIMER);
      SetTimer(hWnd, REFRESH_TIMER, 1, nullptr);
      IsInit = TRUE;
      // call callbacks
      Init();
      Resize();
    }
    Timer();
  } /* End of 'win::OnTimer' function */
  
  /* WM_MOUSEWHEEL window message handle function.
   * ARGUMENTS:
   *   - mouse window position:
   *       int X, Y;
   *   - mouse wheel relative delta value:
   *       int Z;
   *   - mouse keys bits (see MK_*** bits constants):
   *       UINT Keys;
   * RETURNS: None.
   */
  void win::OnMouseWheel( int X, int Y, int Z, UINT Keys )
  {
    MouseWheel += Z;
  } /* End of 'win::OnMouseWheel' function */
  
  /* WM_GETMINMAXINFO window message handle function.
   * ARGUMENTS:
   *   - struct to get info:
   *       LPMINMAXINFO MinMax;
   * RETURNS: None.
   */
  void win::OnGetMinMaxInfo( LPMINMAXINFO MinMax )
  {
    MinMax->ptMaxTrackSize.y = GetSystemMetrics(SM_CYMAXTRACK) +
      GetSystemMetrics(SM_CYCAPTION) + GetSystemMetrics(SM_CYBORDER) * 2;
  } /* End of 'win::OnGetMinMaxInfo' function */
  
  /* WM_*BUTTONDOWN window message handle function
   * ARGUMENTS:
   *   - is click double flag:
   *       int IsDouble;
   *   - mouse coordinates:
   *       int X, Y;
   *   - button flag:
   *       UINT Key;
   * RETURNS: None.
   */
  void win::OnButtonDown( int IsDouble, int X, int Y, UINT Key )
  {
    switch (Key)
    {
    case BUTTON_LEFT:
      if (IsDouble)
      {
      }
      else
      {
      }
      break;
    case BUTTON_RIGHT:
      if (IsDouble)
      {
      }
      else
      {
      }
      break;
    case BUTTON_MIDDLE:
      if (IsDouble)
      {
      }
      else
      {
      }
      break;
    }
  } /* End of 'win::OnButtonDown' function */
  
  /* WM_*BUTTONUP window message handle function
   * ARGUMENTS:
   *   - mouse coordinates:
   *       int X, Y;
   *   - button flag:
   *       UINT Key;
   * RETURNS: None.
   */
  void win::OnButtonUp( int X, int Y, UINT Key )
  {
    switch (Key)
    {
    case BUTTON_LEFT:
      break;
    case BUTTON_RIGHT:
      break;
    case BUTTON_MIDDLE:
      break;
    }
  } /* End of 'win::OnButtonDown' function */
  
  /* WM_CLOSE window message handle function.
   * ARGUMENTS: None.
   * RETURNS: None.
   */
  void win::OnClose( void )
  {
    //if ((MessageBox(hWnd, "Are you sure?", "Closing window", MB_YESNO | MB_ICONINFORMATION)) == IDYES)
    DestroyWindow(hWnd);
  } /* End of 'win::OnClose' function */
  
  /* WM_KEYDOWN window message handle function.
   * ARGUMENTS:
   *   - key of pressed button:
   *       UINT Key;
   * RETURNS: None.
   */
  void win::OnKeyDown( UINT Key )
  {
    if (Key == VK_F5)
      FullScreen();
  } /* End of 'win::OnKeyDown' function */
} /* End of 'nirt' namespace */

/* END OF 'win.cpp' FILE */