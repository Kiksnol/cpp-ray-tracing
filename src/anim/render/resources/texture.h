/* FILE NAME   : texture.h
 * PURPOSE     : Animation project.
 *               texture class declaration module.
 * PROGRAMMER  : ND4
 * LAST UPDATE : 28.07.2021. 
 */

#ifndef __texture_h_
#define __texture_h_

#include "res.h"

#include <fstream>

/* Texture flags */
#define TEXTURE_G32 4
#define TEXTURE_G24 3

/* Project namespace */
namespace nirt
{
  /* texture class */
  class texture
  {
    /* Manager - friend */
    friend class resource_manager<texture, std::string>;
    friend class texture_manager;
    friend class render;
    friend class material;

  private:
    int W, H;   /* Texture size in pixels */
    UINT TexId; /* OpenGL texture Id */

    /* Delete texture function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Free( void )
    {
      memset(this, 0, sizeof(texture));
    } /* End of 'Free' function */
  public:
    char Name[300]; /* Texture name */

    /* Class constructor */
    texture( void ) : Name("")
    {
    } /* End of 'texture' constructor */

    /* texture constructor by string */
    texture( std::string TextureName )
    {
      strcpy((char *)Name, TextureName.c_str());
    } /* End of 'texture' constructor */

    /* Class destructor */
    ~texture( void )
    {
    } /* End of 'texture' destructor */

    /* Apply texture function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Apply( void )
    {
    }

    /* Add texture from image function.
     * ARGUMENTS:
     *   - image file name:
     *       char *Name;
     *   - image width:
     *       int W;
     *   - image hight:
     *       int H;
     *   - components:
     *       int C;
     *   - color bits:
     *       void *Bits;
     * RETURNS: None.
     */
    void LoadArray( const char *Name, int w, int h, int C, void *Bits )
    {
      float mips;

      mips = w > h ? h : w;
      mips = log(mips) / log(2);
      if (mips < 1)
        mips = 1;

      /* Allocate texture space */
      glGenTextures(1, &TexId);
      glBindTexture(GL_TEXTURE_2D, TexId);

      /* Upload texture */
      glTexStorage2D(GL_TEXTURE_2D, mips, C == 3 ? GL_RGB8 : C == 4 ? GL_RGBA8 : GL_R8, w, h);
      glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, Bits);
      glGenerateMipmap(GL_TEXTURE_2D);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

      strcpy(this->Name, Name);
      W = w;
      H = h;

      glBindTexture(GL_TEXTURE_2D, 0);
    } /* End of 'LoadArray' function */

    /* Load texture from '.g32' or '.g24' file function.
     * ARGUMENTS:
     *   - filename to load:
     *       std::string FileName;
     *   - texture flag to load:
     *       int tag;
     * RETURNS: (texture &) created texture
     */
    void LoadFromFile( std::string FileName, int tag )
    {
      DWORD d = 0;
      DWORD *img;
      int
        w = 0,
        h = 0,
        x,
        y;
      std::fstream f;
    
      f.open(FileName, std::fstream::in | std::fstream::binary);
      if (!f.is_open())
        return;
      f.read((char *)&w, 2);
      f.read((char *)&h, 2);

      img = new DWORD[w * h];
    
      for (y = 0; y < h; y++)
        for (x = 0; x < w; x++)
        {
          f.read((char *)&d, tag);
          d |= 0xFF000000;
          img[(h - y - 1) * w + x] = d;
        }
      this->LoadArray(FileName.c_str(), w, h, tag, img);

      delete[] img;
      f.close();
    } /* End of 'LoadFromFile' function */
  }; /* End of 'texture' class */

  /* texture manager class */
  class texture_manager : public resource_manager<texture, std::string>
  {
  protected:

    ~texture_manager( void )
    {
    }

    /* Initialize textures function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Init( void )
    {
    } /* End of 'Init' function */

    /* Close manager function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Close( void )
    {
      resource_manager::Close();
    } /* End of 'Close' function */

  public:
    /* Add texture to manager function.
     * ARGUMENTS:
     *   - name to add:
     *       const char *TexName;
     *   - width and height:
     *       int W, H;
     *   - image components (rgb, rgba):
     *       int C;
     *   - data to save texture:
     *       void *Bits;
     * RETURNS: (texture *) added texture
     */
    texture * Add( const char *TexName, int W, int H, int C, void *Bits )
    {
      texture *fi;
      texture Tex(TexName);

      if ((fi = Find(TexName)) != nullptr)
        return fi;

      Tex.LoadArray(TexName, W, H, C, Bits);
      return resource_manager::Add(Tex);
    } /* End of 'Add' function */

    /* Add texture by filename function.
     * ARGUMENTS:
     *   - filename to add:
     *       const char *FileName;
     *   - flag to load (G24/G32)
     *       int flag;
     * RETURNS: (texture *) added texture
     */
    texture * Add( const char *FileName, int flag )
    {
      texture *fi;
      texture Tex(FileName);

      if ((fi = Find(FileName)) != nullptr)
        return fi;

      Tex.LoadFromFile(FileName, flag);
      return resource_manager::Add(Tex);
    } /* End of 'Add' function */

  }; /* End of 'texture_manager' class */
} /* End of 'nirt' namespace */

#endif /* __texture_h_ */

/* END OF 'texture.h' FILE */