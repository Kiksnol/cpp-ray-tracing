/* FILE NAME  : render.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 23.07.2021
 * PURPOSE    : Animation system.
 *              Rendering subsystem module.
 */

#ifndef __render_h_
#define __render_h_

#include "../../def.h"

// Resources headers
#include "render_primitive.h"
#include "resources/shader.h"
#include "resources/material.h"
#include "resources/texture.h"

/* My NSF namespace */
namespace nirt
{
  /* Rendering class */
  class render : public shader_manager, public material_manager, public texture_manager
  {
  private:
    HWND    &hWnd;            /* Work window handle */                                            
    HDC     hDC;              /* Work window memory device context */
    HGLRC   hGLRC;            /* OpenGL rendering context */

    /* Debug output function.
     * ARGUMENTS:
     *   - source APi or device:
     *       UINT Source;
     *   - error type:
     *       UINT Type;
     *   - error message id:
     *       UINT Id,
     *   - message severity:
     *       UINT severity,
     *   - message text length:
     *       int Length,
     *   - message text:
     *       char *Message,
     *   - user addon parameters pointer:
     *       void *UserParam;
     * RETURNS: None.
     */
    static void APIENTRY glDebugOutput( UINT Source, UINT Type, UINT Id, UINT Severity,
      int Length, const char *Message, const void *UserParam )
    {
      int len = 0;
      static char Buf[10000];

      /* Ignore non-significant error/warning codes */
      if (Id == 131169 || Id == 131185 || Id == 131218 || Id == 131204)
        return;

      len += sprintf(Buf + len, "Debug message (%i) %s\n", Id, Message);
      switch (Source)
      {
      case GL_DEBUG_SOURCE_API:
        len += sprintf(Buf + len, "Source: API\n");
        break;
      case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
        len += sprintf(Buf + len, "Source: Window System\n");
        break;
      case GL_DEBUG_SOURCE_SHADER_COMPILER:
        len += sprintf(Buf + len, "Source: Shader Compiler\n");
        break;
      case GL_DEBUG_SOURCE_THIRD_PARTY:
        len += sprintf(Buf + len, "Source: Third Party\n");
        break;
      case GL_DEBUG_SOURCE_APPLICATION:
        len += sprintf(Buf + len, "Source: Application");
        break;
      case GL_DEBUG_SOURCE_OTHER:
        len += sprintf(Buf + len, "Source: Other");
        break;
      }
      len += sprintf(Buf + len, "\n");

      switch (Type)
      {
      case GL_DEBUG_TYPE_ERROR:
        len += sprintf(Buf + len, "Type: Error");
        break;
      case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
        len += sprintf(Buf + len, "Type: Deprecated Behaviour");
        break;
      case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
        len += sprintf(Buf + len, "Type: Undefined Behaviour");
        break;
      case GL_DEBUG_TYPE_PORTABILITY:
        len += sprintf(Buf + len, "Type: Portability");
        break;
      case GL_DEBUG_TYPE_PERFORMANCE:
        len += sprintf(Buf + len, "Type: Performance");
        break;
      case GL_DEBUG_TYPE_MARKER:
        len += sprintf(Buf + len, "Type: Marker");
        break;
      case GL_DEBUG_TYPE_PUSH_GROUP:
        len += sprintf(Buf + len, "Type: Push Group");
        break;
      case GL_DEBUG_TYPE_POP_GROUP:
        len += sprintf(Buf + len, "Type: Pop Group");
        break;
      case GL_DEBUG_TYPE_OTHER:
        len += sprintf(Buf + len, "Type: Other");
        break;
      }
      len += sprintf(Buf + len, "\n");

      switch (Severity)
      {
      case GL_DEBUG_SEVERITY_HIGH:
        len += sprintf(Buf + len, "Severity: high");
        break;
      case GL_DEBUG_SEVERITY_MEDIUM:
        len += sprintf(Buf + len, "Severity: medium");
        break;
      case GL_DEBUG_SEVERITY_LOW:
        len += sprintf(Buf + len, "Severity: low");
        break;
      case GL_DEBUG_SEVERITY_NOTIFICATION:
        len += sprintf(Buf + len, "Severity: notification");
        break;
      }
      len += sprintf(Buf + len, "\n\n");

      OutputDebugStringA(Buf);
    } /* End of 'glDebugOutput' function */
  public:
    int &FrameW, &FrameH; /* Work window size */
    camera cam;           /* Working camera */

    /* Class constructor by window and size: */
    render( HWND &hWndRef, int &FrameWRef, int &FrameHRef ) : 
      hWnd(hWndRef), FrameW(FrameWRef), FrameH(FrameHRef)
    {
      int i;
      UINT nums;
      PIXELFORMATDESCRIPTOR pfd = {0};
      HGLRC hRC;
      int PixelAttribs[] =
      {
        WGL_DRAW_TO_WINDOW_ARB, TRUE,
        WGL_SUPPORT_OPENGL_ARB, TRUE,
        WGL_DOUBLE_BUFFER_ARB, TRUE,
        WGL_PIXEL_TYPE_ARB, WGL_TYPE_RGBA_ARB,
        WGL_ACCELERATION_ARB, WGL_FULL_ACCELERATION_ARB,
        WGL_COLOR_BITS_ARB, 32,
        WGL_DEPTH_BITS_ARB, 32,
        0
      };
      int ContextAttribs[] =
      {
        WGL_CONTEXT_MAJOR_VERSION_ARB, 4,
        WGL_CONTEXT_MINOR_VERSION_ARB, 6,
        WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_COMPATIBILITY_PROFILE_BIT_ARB, 0
      };
      hDC = GetDC(hWnd);

      /* OpenGL init: pixel format setup */
      pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
      pfd.nVersion = 1;
      pfd.dwFlags = PFD_DOUBLEBUFFER | PFD_SUPPORT_OPENGL;
      pfd.cColorBits = 32;
      pfd.cDepthBits = 32;
      i = ChoosePixelFormat(hDC, &pfd);
      DescribePixelFormat(hDC, i, sizeof(PIXELFORMATDESCRIPTOR), &pfd);
      SetPixelFormat(hDC, i, &pfd);

      /* OpenGL init: setup rendering context */
      hGLRC = wglCreateContext(hDC);
      wglMakeCurrent(hDC, hGLRC);

      /* Initializing GLEW library */
      if (glewInit() != GLEW_OK)
      {
        MessageBox(hWnd, "Error extensions initializing", "Error",
          MB_ICONERROR | MB_OK);
        exit(0);
      }

      if (!(GLEW_ARB_vertex_shader && GLEW_ARB_fragment_shader))
      {
        MessageBox(hWnd, "Error: no shaders support", "Error", MB_ICONERROR | MB_OK);
        exit(0);
      }

      /* Enable a new OpenGL profile support */
      wglChoosePixelFormatARB(hDC, PixelAttribs, NULL, 1, &i, &nums);
      hRC = wglCreateContextAttribsARB(hDC, NULL, ContextAttribs);
      wglMakeCurrent(NULL, NULL);
      wglDeleteContext(hGLRC);

      hGLRC = hRC;
      wglMakeCurrent(hDC, hGLRC);
      /* Set default OpenGL parameters */
      glClearColor(0.30, 0.47, 0.8, 1);
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_PRIMITIVE_RESTART);
      glPrimitiveRestartIndex(-1);
      wglSwapIntervalEXT(0);

      shader_manager::Add("DEFAULT");
    } /* End of 'Render' constructor */

    /* Class destructor */
    ~render( void )
    {
      shader_manager::Close();
      material_manager::Close();
      texture_manager::Close();
      // RndTargetClose();
      // RndResClose();
      // RndShdClose();

      wglMakeCurrent(NULL, NULL);
      wglDeleteContext(hGLRC);
      ReleaseDC(hWnd, hDC);
    } /* End of 'render' destructor */

    /* Frame start function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void FrameStart( void );

    /* Frame end function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void FrameEnd( void )
    {
      glFinish();
    } /* End of 'FrameEnd' function */

    /* Copyframe function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void CopyFrame( void )
    {
      wglSwapLayerBuffers(hDC, WGL_SWAP_MAIN_PLANE);
    } /* End of 'CopyFrame' function */

    /* Resize function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Resize( void )
    {
      glViewport(0, 0, FrameW, FrameH);
      cam.Resize(FrameW, FrameH);
    } /* End of 'Resize' function */

    /* Draw primitive function.
     * ARGUMENTS:
     *   - primitive to draw:
     *       primitive *Pr;
     *   - matrix to draw:
     *       matr World;
     * RETURMS: None.
     */
    void PrimitiveDraw( primitive *Pr, matr World = matr::Identity() )
    {
      matr
        w = Pr->Trans * World,
        wvp = w * cam.VP;
      int gl_prim_type = Pr->Type;
      material *Mtl = Pr->Mtl;

      if (Mtl != nullptr && Pr->Type >= 0 )
      {
        Mtl->Apply();
        Mtl->Shd->UniSet("MatrWVP", wvp);
        Mtl->Shd->UniSet("MatrW", w);
        Mtl->Shd->UniSet("FrameW", FrameW);
        Mtl->Shd->UniSet("FrameH", FrameH);
      }

      //glLoadMatrixf(wvp);
      /* making an array of vertices active */
      glBindVertexArray(Pr->VA);
      if (gl_prim_type == GL_PATCHES)
      {
        glPatchParameteri(GL_PATCH_VERTICES, 16);
        glDrawArrays(gl_prim_type, 0, Pr->NumOfEl);
      }
      else if (Pr->IBuf == 0)
        glDrawArrays(gl_prim_type, 0, Pr->NumOfEl);
      else
      {
        /* making an array of indexes active */
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, Pr->IBuf);
        /* drawing */
        glDrawElements(gl_prim_type, Pr->NumOfEl, GL_UNSIGNED_INT, NULL);
        /* disable index array */
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
      }
      /* disable vertex array */
      glBindVertexArray(0);
    } /* End of 'PrimitiveDraw' function */

    /* Draw model function.
     * ARGUMENTS:
     *   - model to draw:
     *       model *Mdl;
     *   - matrix to draw:
     *       matr World;
     * RETURNS: None.
     */
    void ModelDraw( model *Mdl, matr World = matr::Identity() )
    {
      auto walker = [this, World]
        ( primitive *Pr )
      {
        PrimitiveDraw(Pr, World);
      };

      Mdl->WalkAll(walker);
    } /* End of 'ModelDraw' function. */
  }; /* End of 'render' class */
} /* End of 'nirt' namespace */

#endif /* __render_h_ */

/* END OF 'render.h' FILE */