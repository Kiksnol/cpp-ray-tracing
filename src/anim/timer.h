/* FILE NAME  : timer.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 23.07.2021
 * PURPOSE    : Timer class
 */

#ifndef __timer_h_
#define __timer_h_

#include "../win/win.h"

/* Unsigned __int64 typedef */
typedef unsigned __int64 UINT64;

/* My NSF namespace */
namespace nirt
{
  /* Timer class definition */
  class timer
  {
  protected:
    UINT64
      StartTime,       // Start program time
      OldTime,         // Previous frame time
      OldTimeFPS,      // Old time FPS measurement
      PauseTime,       // Time during pause period
      TimePerSec,      // Timer resolution
      FrameCounter;    // Frames counter
  public:
    int IsPause;      // Pause flag
    double
      FPS,             // Frame per second value
      DeltaTime;       // Time before previous and current frame
    double
      GlobalTime,      // Global time (MSK)
      GlobalDeltaTime, // Global time before previous and current frame
      Time;            // Time before program started

    /* Class constructor */
    timer( void ) : DeltaTime(0), GlobalDeltaTime(0), GlobalTime(0), Time(0)
    {
      LARGE_INTEGER t;

      QueryPerformanceFrequency(&t);
      TimePerSec = t.QuadPart;
      QueryPerformanceCounter(&t);
      StartTime = OldTime = OldTimeFPS = t.QuadPart;
      FrameCounter = 0;
      IsPause = FALSE;
      FPS = 30.0;
      PauseTime = 0;
    } /* End of 'timer' constructor */

    /* Timer response function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Response( void )
    {
      LARGE_INTEGER t;

      QueryPerformanceCounter(&t);
      /* Global time */
      GlobalTime = (double)(t.QuadPart - StartTime) / TimePerSec;
      GlobalDeltaTime = (double)(t.QuadPart - OldTime) / TimePerSec;
      /* Time with pause */
      if (IsPause)
      {
        DeltaTime = 0;
        PauseTime += t.QuadPart - OldTime;
      }
      else
      {
        DeltaTime = GlobalDeltaTime;
        Time = (double)(t.QuadPart - PauseTime - StartTime) / TimePerSec;
      }
      /* FPS count */
      FrameCounter++;
      if (t.QuadPart - OldTimeFPS > TimePerSec)
      {
        FPS = FrameCounter * TimePerSec / (double)(t.QuadPart - OldTimeFPS);
        OldTimeFPS = t.QuadPart;
        FrameCounter = 0;
      }
      OldTime = t.QuadPart;
    } /* End of 'Responce' function */
  }; /* End of 'timer' class */
} /* End of 'nirt' namespace */

#endif /* __timer_h_ */

/* END OF 'timer.h' FILE */