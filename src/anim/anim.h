/* FILE NAME  : anim.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 23.07.2021
 * PURPOSE    : Animation system class
 */

#ifndef __anim_h_
#define __anim_h_

#include <vector>

#include "../win/win.h"
#include "../anim/render/render.h"
#include "timer.h"
#include "input/input.h"
//#include "../units/units.h"

/* My NSF namespace */
namespace nirt
{
  /* Unit base class */
  class unit
  {
  public:

    /* Unit deinitialization */
    virtual ~unit( void )
    {
    } /* End of 'unit' destructor */

    /* Unit render function.
     * ARGUMENTS:
     *   - animation context:
     *      anim &Ani;
     * RETURNS: None.
     */
    virtual void Render( anim *Ani )
    {
    } /* End of 'Render' function */

    /* Unit inter frame events handle function.
     * ARGUMENTS:
     *   - animation context:
     *      anim &Ani;
     * RETURNS: None.
     */
    virtual void Response( anim *Ani )
    {
    } /* End of 'Response' function */
  }; /* End of 'unit' class */

  /* Animation system class */
  class anim : public win, public render, public timer, public input
  {
  private:
    /* Anim constructor */
    anim( HINSTANCE hInst = GetModuleHandle(nullptr) ) : win(hInst), input(win::MouseWheel, win::hWnd),
      render(win::hWnd, W, H)
    {
      char Buf[_MAX_PATH];

      GetCurrentDirectory(sizeof(Buf), Buf);
      Path = std::string(Buf);
    } /* End of 'anim' constructor */

    /* Class destructor */
    ~anim( void ) override
    {
      for (auto &Uni : Units)
        delete Uni;
    } /* End of 'anim' destructor */

    std::vector<unit *> Units;
    static anim Instance;        // Single tone
  public:
    static std::string Path;     // Current directory

    /* Get single tone function.
     * ARGUMENTS: None.
     * RETURNS: (anim &) single tone.
     */
    static anim & Get( void )
    {
      return Instance;
    } /* End of 'Get' function */

    /* Get single tone ptr.
     * ARGUMENTS: None.
     * RETURNS: (anim *) single tone.
     */
    static anim * GetPtr( void )
    {
      return &Instance;
    } /* End of 'GetPtr' function */

    /* Render function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Render( void )
    {
      this->FrameStart();

      timer::Response();
      input::Responce();

      for (auto &Uni : Units)
        Uni->Response(this);

      for (auto &Uni : Units)
        Uni->Render(this);

      this->FrameEnd();
    } /* End of 'Render' function */

    /* Close window function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void CloseWnd( void )
    {
      DestroyWindow(win::hWnd);
    } /* End of 'CloseWnd' function */

    /* Initialization function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Init( void ) override
    {
    } /* End of 'Init' function */

    /* Deinitialization function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Close( void ) override
    {
    } /* End of 'Close' function */

    /* Resize window function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Resize( void ) override
    {
      render::Resize();
      this->Render();
      this->CopyFrame();
    } /* End of 'Resize' function */

    /* Activate window function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Activate( void ) override
    {
    } /* End of 'Activate' function */

    /* Idle function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Idle( void ) override
    {
      this->Render();
      this->CopyFrame();
    } /* End of 'Idle' function */

    /* Timer function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Timer( void ) override
    {
      char Buf[300];

      sprintf_s(Buf, 300, "%.3f\n", FPS);
      //SetWindowText(win::hWnd, Buf);

      //char Buf[30];

      //sprintf_s(Buf, 300, "CAMLOC: X: %.3f Y: %.3f Z: %.3f\n", Cam.Loc[0], Cam.Loc[1], Cam.Loc[2]);
      //sprintf_s(Buf, 300, "CAMAT: X: %.3f Y: %.3f Z: %.3f\n", Cam.At[0], Cam.At[1], Cam.At[2]);
      SetWindowText(win::hWnd, Buf);

      this->Render();
      this->CopyFrame();

      //sprintf(Str, "FPS: %.3f\n", FPS);
      //sprintf(Str, "%.3f %.3f %.3f\n", cam.Loc[0], cam.Loc[1], cam.Loc[2]);
      //SetWindowTextA(win::hWnd, Str);
    } /* End of 'Timer' function */

    /* Paint function.
     * ARGUMENTS:
     *   - window device context:
     *       HDC hDC;
     * RETURNS: None.
     */
    void Paint( HDC hDC ) override
    {
    } /* End of 'Paint' function */

    /* Add unit operator.
     * ARGUMENTS:
     *   - unit to add:
     *       unit *Uni;
     * RETURNS: (anim &) *this
     */
    anim & operator << ( unit *Uni )
    {
      Units.push_back(Uni);
      return *this;
    } /* End of 'operator = ' function. */
  }; /* End of 'anim' class */
} /* End of 'nirt' namespace */

#endif /* __anim_h_ */

/* END OF 'anim.h' FILE */