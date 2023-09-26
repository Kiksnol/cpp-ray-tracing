/* FILE NAME  : win.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 17.07.2022
 * PURPOSE    : WinAPI class
 */

#ifndef __win_h_
#define __win_h_

#include "../def.h"

#include <cstring>

 /* Buttons constants */
#define BUTTON_LEFT   30
#define BUTTON_RIGHT  47
#define BUTTON_MIDDLE 102

/* Initialization timer constant */
#define INIT_TIMER    0x3047

/* My NSF namespace */
namespace nirt
{
  /* Windows class */
  class win
  {
  public:
    HWND hWnd;               // window handle
    int IsFullScreen;       // Full Screen flag
  protected:
    int W, H;                // window size
    HINSTANCE hInstance;     // application handle
  private:
    RECT FullScreenSaveRect; // FullScreen rectangle
    int IsInit;             // Initialization flag

    /* Window handle function.
     * ARGUMENTS:
     *   - window handle:
     *       HWND hWnd;
     *   - message type (see WM_***):
     *       UINT Msg;
     *   - message 'word' parameter:
     *       WPARAM wParam;
     *   - message 'long' parameter:
     *       LPARAM lParam;
     * RETURNS:
     *   (LRESULT) message depende return value.
     */
    static LRESULT CALLBACK WinFunc( HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam )
    {
      win *Win = nullptr;
      HDC hDC;
      PAINTSTRUCT ps;
      UINT Key = 0;

      switch (Msg)
      {
      case WM_GETMINMAXINFO:
        Win->OnGetMinMaxInfo((MINMAXINFO *)lParam);
        return 0;
      case WM_CREATE:
        SetWindowLongPtr(hWnd, 0, (UINT_PTR)((CREATESTRUCT *)lParam)->lpCreateParams);
      default:
        Win = reinterpret_cast<win *>(GetWindowLongPtr(hWnd, 0));
        if (Win != nullptr)
          switch (Msg)
          {
          case WM_CREATE:
            Win->hWnd = hWnd;
            return Win->OnCreate((CREATESTRUCT *)lParam) ? 0 : -1;
          case WM_ACTIVATE:
            Win->OnActivate((UINT)LOWORD(wParam), (HWND)lParam, (int)HIWORD(lParam));
            return 0;
          case WM_SIZE:
            Win->OnSize(wParam, LOWORD(lParam), HIWORD(lParam));
            return 0;
          case WM_ERASEBKGND:
            return (LRESULT)Win->OnEraseBkgnd((HDC)wParam);
          case WM_PAINT:
            hDC = BeginPaint(hWnd, &ps);
            Win->OnPaint(hDC, &ps);
            EndPaint(hWnd, &ps);
            return 0;
          case WM_TIMER:
            Win->OnTimer((UINT)wParam);
            return 0;
          case WM_MOUSEWHEEL:
            Win->OnMouseWheel((int)(SHORT)LOWORD(lParam),
              (int)(SHORT)HIWORD(lParam),
              (int)(SHORT)HIWORD(wParam),
              (UINT)(SHORT)LOWORD(wParam));
            return 0;
          case WM_LBUTTONDOWN:
          case WM_RBUTTONDOWN:
          case WM_MBUTTONDOWN:
            switch (Msg)
            {
            case WM_LBUTTONDOWN:
              Key = BUTTON_LEFT;
              break;
            case WM_MBUTTONDOWN:
              Key = BUTTON_MIDDLE;
              break;
            case WM_RBUTTONDOWN:
              Key = BUTTON_RIGHT;
              break;
            }
            Win->OnButtonDown(FALSE,
              (int)(SHORT)LOWORD(lParam),
              (int)(SHORT)HIWORD(lParam), Key);
            return 0;
          case WM_LBUTTONUP:
          case WM_RBUTTONUP:
          case WM_MBUTTONUP:
            switch (Msg)
            {
            case WM_LBUTTONUP:
              Key = BUTTON_LEFT;
              break;
            case WM_MBUTTONUP:
              Key = BUTTON_MIDDLE;
              break;
            case WM_RBUTTONUP:
              Key = BUTTON_RIGHT;
              break;
            }
            Win->OnButtonUp((int)(SHORT)LOWORD(lParam),
              (int)(SHORT)HIWORD(lParam), Key);
            return 0;
          case WM_LBUTTONDBLCLK:
          case WM_RBUTTONDBLCLK:
          case WM_MBUTTONDBLCLK:
            switch (Msg)
            {
            case WM_LBUTTONDBLCLK:
              Key = BUTTON_LEFT;
              break;
            case WM_MBUTTONDBLCLK:
              Key = BUTTON_MIDDLE;
              break;
            case WM_RBUTTONDBLCLK:
              Key = BUTTON_RIGHT;
              break;
            }
            Win->OnButtonDown(TRUE,
              (int)(SHORT)LOWORD(lParam),
              (int)(SHORT)HIWORD(lParam), Key);
            return 0;
          case WM_KEYDOWN:
            Win->OnKeyDown(wParam);
            return 0;
          case WM_CLOSE:
            Win->OnClose();
            return 0;
          case WM_DESTROY:
            Win->OnDestroy();
            return 0;
          }
      }
      return DefWindowProc(hWnd, Msg, wParam, lParam);
    } /* End of 'WinFunc' function */
  public:
    int IsActive;  // Active flag
    int MouseWheel; // Mouse wheel value

    /* Default constructor */
    win( HINSTANCE hInst = GetModuleHandle(nullptr) ) : hInstance(hInst), W(0), H(0), MouseWheel(0),
      IsFullScreen(FALSE), IsActive(TRUE), hWnd((HWND)0), FullScreenSaveRect(), IsInit(FALSE)
    {
      WNDCLASS wc;

      wc.style = CS_VREDRAW | CS_HREDRAW;          /* Window redraw style */
      wc.cbClsExtra = 0;                           /* Extra bytes for class */
      wc.cbWndExtra = sizeof(win *);               /* To connect class with Windows */
      wc.hbrBackground = (HBRUSH)COLOR_WINDOW;     /* Background color */
      wc.hCursor = LoadCursor(nullptr, IDC_CROSS); /* Window cursor */
      wc.hIcon = LoadIcon(nullptr, IDI_ASTERISK);  /* Window icon */
      wc.lpfnWndProc = WinFunc;                    /* Message peeker callback function */
      wc.lpszMenuName = nullptr;                   /* Menu resource name */
      wc.hInstance = hInstance;                    /* Program registering class descriptor */
      wc.lpszClassName = "T01RTRT project";        /* Class name */

      /* Register class in system */
      if (!RegisterClass(&wc))
      {
        MessageBox(nullptr, "Error register window class", ERROR, MB_OK);
        return;
      }

      /* Create window */
      hWnd = CreateWindow("T01RTRT project", "T01RTRT", WS_OVERLAPPEDWINDOW,
        30, 30, CW_USEDEFAULT, CW_USEDEFAULT, nullptr, nullptr, hInstance,
        reinterpret_cast<void *>(this));
      /* Show and draw window */
      ShowWindow(hWnd, SW_SHOWNORMAL);
      UpdateWindow(hWnd);
    } /* End of 'win' constructor */

    /* Win virtual destructor */
    virtual ~win( void )
    {
    } /* End of 'win' destructor */

    /* Start message loop function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Run( void )
    {
      MSG Msg;

      /* Message loop */
      while (TRUE)
      {
        if (PeekMessage(&Msg, nullptr, 0, 0, PM_REMOVE))
        {
          if (Msg.message == WM_QUIT)
            break;
          DispatchMessage(&Msg);
        }
        else
        {
          if (IsInit)
            Idle();
        }
      }
    } /* End of 'Run' function */

    /* Flip full screen function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void FullScreen( void )
    {
      if (!IsFullScreen)
      {
        HMONITOR hMon;
        MONITORINFOEX moninfo;
        RECT rc;

        IsFullScreen = TRUE;

        /* Save old window size and position */
        GetWindowRect(hWnd, &FullScreenSaveRect);

        /* Get closest monitor */
        hMon = MonitorFromWindow(hWnd, MONITOR_DEFAULTTONEAREST);

        /* Get monitor information */
        moninfo.cbSize = sizeof(moninfo);
        GetMonitorInfo(hMon, (MONITORINFO *)&moninfo);

        rc = moninfo.rcMonitor;
        AdjustWindowRect(&rc, GetWindowLong(hWnd, GWL_STYLE), FALSE);

        /* Expand window to full screen */
        SetWindowPos(hWnd, HWND_TOP,
          rc.left, rc.top,
          rc.right - rc.left, rc.bottom - rc.top,
          SWP_NOOWNERZORDER);
      }
      else
      {
        IsFullScreen = FALSE;

        /* Restore window size and position */
        SetWindowPos(hWnd, HWND_NOTOPMOST,
          FullScreenSaveRect.left, FullScreenSaveRect.top,
          FullScreenSaveRect.right - FullScreenSaveRect.left, FullScreenSaveRect.bottom - FullScreenSaveRect.top,
          SWP_NOOWNERZORDER);
      }
    } /* End of 'FullScreen' function */
  private:
    /******** Message crackers ********/
    /* WM_CREATE window message handle function.
     * ARGUMENTS:
     *   - structure with creation data:
     *       CREATESTRUCT *CS;
     * RETURNS:
     *   (int) TRUE to continue creation window, FALSE to terminate.
     */
    int OnCreate( CREATESTRUCT *CS );

    /* WM_DESTROY window message handle function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void OnDestroy( void );


    /* WM_SIZE window message handle function.
     * ARGUMENTS:
     *   - sizing flag (see SIZE_***, like SIZE_MAXIMIZED)
     *       UINT State;
     *   - new width and height of client area:
     *       int W, H;
     * RETURNS: None.
     */
    void OnSize( UINT State, int W, int H );

    /* WM_ERASEBKGND window message handle function.
     * ARGUMENTS:
     *   - device context of client area:
     *       HDC hDC;
     * RETURNS:
     *   (int) TRUE if background is erased, FALSE otherwise.
     */
    int OnEraseBkgnd( HDC hDC );

    /* WM_PAINT window message handle function.
     * ARGUMENTS:
     *   - window device context:
     *       HDC hDC;
     *   - paint message structure pointer:
     *       PAINTSTRUCT *PS;
     * RETURNS: None.
     */
    void OnPaint( HDC hDC, PAINTSTRUCT *PS );

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
    void OnActivate( UINT Reason, HWND hWndActDeact, int IsMinimized );

    /* WM_TIMER window message handle function.
     * ARGUMENTS:
     *   - specified the timer identifier.
     *       int Id;
     * RETURNS: None.
     */
    void OnTimer( int Id );

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
    void OnMouseWheel( int X, int Y, int Z, UINT Keys );

    /* WM_GETMINMAXINFO window message handle function.
     * ARGUMENTS:
     *   - struct to get info:
     *       LPMINMAXINFO MinMax;
     * RETURNS: None.
     */
    void OnGetMinMaxInfo( LPMINMAXINFO MinMax );

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
    void OnButtonDown( int IsDouble, int X, int Y, UINT Key );

    /* WM_*BUTTONUP window message handle function
     * ARGUMENTS:
     *   - mouse coordinates:
     *       int X, Y;
     *   - button flag:
     *       UINT Key;
     * RETURNS: None.
     */
    void OnButtonUp( int X, int Y, UINT Key );

    /* WM_CLOSE window message handle function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void OnClose( void );

    /* WM_KEYDOWN window message handle function.
     * ARGUMENTS:
     *   - key of pressed button:
     *       UINT Key;
     * RETURNS: None.
     */
    void OnKeyDown( UINT Key );
  private:
    /******** Virtual callbacks functions ********/

    /* Initialization function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    virtual void Init( void )
    {
    } /* End of 'Init' function */

    /* Deinitialization function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    virtual void Close( void )
    {
    } /* End of 'Close' function */

    /* Resize window function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    virtual void Resize( void )
    {
    } /* End of 'Resize' function */

    /* Timer function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    virtual void Timer( void )
    {
    } /* End of 'Timer' function */

    /* Activate window function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    virtual void Activate( void )
    {
    } /* End of 'Activate' function */

    /* Idle function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    virtual void Idle( void )
    {
    } /* End of 'Idle' function */

    /* Erase background window function.
     * ARGUMENTS: 
     *   - window device context:
     *       HDC hDC;
     * RETURNS: None.
     */
    virtual void Erase( HDC hDC )
    {
    } /* End of 'Erase' function */

    /* Paint function.
     * ARGUMENTS:
     *   - window device context:
     *       HDC hDC;
     * RETURNS: None.
     */
    virtual void Paint( HDC hDC )
    {
      Rectangle(hDC, -1, -1, W + 1, H + 1);
    } /* End of 'Paint' function */
  }; /* End of 'win' class */
} /* End of 'nirt' namespace */

#endif /* __win_h_ */

/* END OF 'win.h' FILE */