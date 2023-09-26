/* FILE NAME   : shader.h
 * PURPOSE     : Animation project.
 *               Shader class declaration module.
 * PROGRAMMER  : ND4
 * LAST UPDATE : 27.07.2021. 
 */

#ifndef __shader_h_
#define __shader_h_

#include <fstream>

#include "res.h"

/* Project namespace */
namespace nirt
{
  /* Shader class */
  class shader
  {
    /* Manager - friend */
    friend class resource_manager<shader, std::string>;
    friend class shader_manager;
    friend class render;

  private:
    UINT ProgId;    /* Shader program id */

    /* Load text from file function.
     * ARGUMENTS:
     *   - file name to load:
     *       const std::string FileName;
     * RETURNS: (std::string) file text
     */
    std::string LoadText( const std::string FileName );

    /* Store log to file function.
     * ARGUMENTS:
     *   - message file prefix, shader name and text:
     *       char *Prefix, *PartName, *Text;
     * RETURNS: None.
     */
    void Log( const std::string Prefix, const char *PartName, const char *Text );

    /* Load shader function.
     * ARGUMENTS:
     *   - shader name:
     *       const char *Prefix;
     * RETURNS: (shader & ) this pointer
     */
    shader & Load( const std::string Prefix )
    {
      char Buf[1000] = "";
      std::string text;

      struct
      {
        const char *Name; /* Shader file prefix name (e.g. "VERT") */
        int Type;         /* Shader OpenFL type (e.g. GL_VERTEX_SHADER) */
        int Id;           /* Obtained shader Id from OpenGL */
      } shd[] =
      {
        {"VERT", GL_VERTEX_SHADER},
        {"CTRL", GL_TESS_CONTROL_SHADER},
        {"EVAL", GL_TESS_EVALUATION_SHADER},
        {"GEOM", GL_GEOMETRY_SHADER},
        {"FRAG", GL_FRAGMENT_SHADER},
      };
      int NoofS = sizeof(shd) / sizeof(shd[0]), res, prg = 0;
      int is_ok = TRUE;

      for (int i = 0; i < NoofS; i++)
      {
        /* Filename */
        sprintf(Buf, "bin/shaders/%s/%s.GLSL", Prefix.c_str(), shd[i].Name);
        /* Load text */
        text = LoadText(Buf);
        if (text == "")
        {
          if (i == 0 || i == NoofS - 1)
          {
            Log(Prefix, shd[i].Name, "Error load file");
            is_ok = FALSE;
            break;
          }
          continue;
        }
        /* Create shader program */
        if ((shd[i].Id = glCreateShader(shd[i].Type)) == 0)
        {
          Log(Prefix, shd[i].Name, "Error create shader");
          is_ok = FALSE;
          break;
        }
        /* Attach text to shader */
        const char *Src = text.c_str();
        glShaderSource(shd[i].Id, 1, &Src, nullptr);

        /* Compile shader */
        glCompileShader(shd[i].Id);
        glGetShaderiv(shd[i].Id, GL_COMPILE_STATUS, &res);
        if (res != 1)
        {
          glGetShaderInfoLog(shd[i].Id, sizeof(Buf), &res, Buf);
          Log(Prefix, shd[i].Name, Buf);
          is_ok = FALSE;
          break;
        }
        text = "";
      }

      /* Create shader program */
      if (is_ok)
      {
        if ((prg = glCreateProgram()) == 0)
        {
          Log(Prefix, "PROG", "Error create program");
          is_ok = FALSE;
        }
        else
        {
          /* Attach shaders to program */
          for (int i = 0; i < NoofS; i++)
            if (shd[i].Id != 0)
              glAttachShader(prg, shd[i].Id);
          /* Link shader program */
          glLinkProgram(prg);
          glGetProgramiv(prg, GL_LINK_STATUS, &res);
          if (res != 1)
          {
            glGetProgramInfoLog(prg, sizeof(Buf), &res, Buf);
            Log(Prefix, "PROG", Buf);
            is_ok = FALSE;
          }
        }
      }

      if (!is_ok)
      {
        /* Delete all created shaders */
        for (int i = 0; i < NoofS; i++)
          if (shd[i].Id != 0)
          {
            if (prg)
              glDetachShader(prg, shd[i].Id);
            glDeleteShader(shd[i].Id);
          }
        /* Delete program */
        if (prg)
          glDeleteProgram(prg);
        prg = 0;
      }
      ProgId = prg;
      return *this;
    } /* End of 'Load' function */

    /* Delete shader function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Free( void )
    {
      UINT shdrs[5];
      int n;

      if (!glIsProgram(ProgId))
        return;

      if (ProgId == 0)
        return;

      glGetAttachedShaders(ProgId, 5, &n, shdrs);
      for (int i = 0; i < n; i++)
      {
        glDetachShader(ProgId, shdrs[i]);
        glDeleteShader(shdrs[i]);
      }
      glDeleteProgram(ProgId);
      ProgId = 0;
    } /* End of 'Free' function */
  public:
    char Name[300]; /* Shader name */

    /* Class constructor */
    shader( void ) : ProgId(0), Name("")
    {
    } /* End of 'shader' constructor */

    /* Shader constructor by string */
    shader( const char *ShaderName ) : ProgId(0)
    {
      strcpy(Name, ShaderName);
    } /* End of 'shader' constructor */

    /* Class destructor */
    ~shader( void )
    {
    } /* End of 'shader' destructor */

    /* Shader uniform value set function.
     * ARGUMENTS:
     *   - uniform name:
     *       const char *Name;
     *   - uniform value:
     *       const value_type &Value;
     * RETURNS: None.
     */
    template<typename value_type>
    void UniSet( const char *Name, value_type &Value )
    {
      int loc = glGetUniformLocation(ProgId, Name);

      if (loc != -1)
      {
        if constexpr (std::is_convertible_v<float, value_type>)
          glUniform1f(loc, Value);
        else if constexpr (std::is_convertible_v<int, value_type>)
          glUniform1i(loc, Value);
        else if constexpr (std::is_convertible_v<vec3, value_type>)
          glUniform3fv(loc, 1, Value);
        else if constexpr (std::is_convertible_v<matr, value_type>)
          glUniformMatrix4fv(loc, 1, FALSE, Value);
      }
    } /* End of 'Uniset' function */

    /* Apply shader function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Apply( void )
    {
      if (!glIsProgram(ProgId))
        return;

      glUseProgram(this->ProgId);
    } /* End of 'Apply' function */
    
  }; /* End of 'shader' class */

  /* Shader manager class */
  class shader_manager : public resource_manager<shader, std::string>
  {
  public:

    ~shader_manager( void )
    {
    }
    /* Add shader to manager function.
     * ARGUMENTS:
     *   - shader name to add:
     *       const char *ShaderName;
     * RETURNS: (shader *) created shader.
     */
    shader * Add( const char *ShaderName )
    {
      shader *find = nullptr;
      shader shd(ShaderName);

      if ((find = Find(ShaderName)) != nullptr)
        return find;

      shd.Load(ShaderName);
      return resource_manager::Add(shd);
    } /* End of 'Add' function. */

    /* Initialize shaders function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Init( void )
    {
    } /* End of 'Init' function */

    /* Update shaders function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void UpdateShaders( void )
    {
      for (auto &shd : Stock)
      {
        shd.second.Free();
        shd.second.Load(shd.second.Name);
      }
    } /* End of 'UpdateShaders' function. */

    /* Close manager function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Close( void )
    {
      resource_manager::Close();
    } /* End of 'Close' function */
  }; /* End of 'shader_manager' class */
} /* End of 'nirt' namespace */


#endif /* __shader_h_ */

/* END OF 'shader.h' FILE */