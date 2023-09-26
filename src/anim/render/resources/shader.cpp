/* FILE NAME   : shader.cpp
 * PURPOSE     : Animation project.
 *               Shader class module.
 * PROGRAMMER  : ND4
 * LAST UPDATE : 27.07.2021. 
 */

#include <iostream>

#include "shader.h"
#include "../../anim.h"

namespace nirt
{
  /* Load text from file function.
   * ARGUMENTS:
   *   - file name to load:
   *       const std::string FileName;
   * RETURNS: (std::string) file text
   */
  std::string shader::LoadText( const std::string FileName )
  {
    std::ifstream f(anim::Path + FileName);
  
    return std::string((std::istreambuf_iterator<char>(f)),
                   std::istreambuf_iterator<char>());
  } /* End of 'shader::LoadText' function */

  /* Store log to file function.
   * ARGUMENTS:
   *   - message file prefix, shader name and text:
   *       char *Prefix, *PartName, *Text;
   * RETURNS: None.
   */
  void shader::Log( const std::string Prefix, const char *PartName, const char *Text )
  {
    std::ofstream(anim::Path + "bin/shaders/ND4{30}.LOG", std::ios_base::app) <<
      Prefix << ":" << PartName << ":\n" << Text << "\n";
  } /* End of 'shader::Log' function */
}

/* END OF 'shader.cpp' FILE */