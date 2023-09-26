/* FILE NAME  : render_primitive.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 29.07.2021
 * PURPOSE    : Animation system.
 *              Rendering subsystem module.
 *              Primitives handle
 */

#ifndef __render_primitive_h_
#define __render_primitive_h_

#include <vector>
#include <cstring>
#include <fstream>
#include <sstream>

#include "../../def.h"
#include "resources/material.h"

/* My NSF namespace */
namespace nirt
{
  /* Reader class */
  class rdr
  {
  private:
    unsigned char *&Ptr; /* Reference for pointer to read */
  public:
    /* Class constructor by pointer */
    rdr( unsigned char *&PtrRef ): Ptr(PtrRef)
    {
    } /* End of 'rdr' constructor */
  
    /* Read data function.
     * ARGUMENTS:
     *   - data to read:
     *       type *Data;
     *   - amount of read data:
     *       int Cnt;
     * RETURNS: None.
     */
    template<typename type>
    void operator ()( type *Data, int Cnt = 1 )
    {
      memcpy(Data, Ptr, sizeof(type) * Cnt), Ptr += sizeof(type) * Cnt;
    } /* End of 'operator()' function */
  }; /* End of 'rdr' class */

  /* VBO vertex class */
  class vertex
  {
  public:
    vec3 P; /* Position */
    vec2 T; /* Texture coordinates */
    vec3 N; /* Vector of normal */
    vec4 C; /* Color (r, g, b, a) */

    /* Constructor */
    vertex( void ) : P(0), T(0), N(0), C(0)
    {
    } /* End of 'vertex' constructor */
  };

  enum prtype
    {
      PRIM_TRIMESH = GL_TRIANGLES,         /* Triangle mesh - array of triangles */
      PRIM_TRISTRIP = GL_TRIANGLE_STRIP,   /* Triangle strip - array of stripped triangles */
      PRIM_LINES = GL_LINES,               /* Line segments (by 2 points) */
      PRIM_POINTS = GL_POINTS,             /* Array of points */
      PRIM_PATH = GL_PATCHES,              /* path of points */
    };

  /* Primitive class */
  class primitive
  {
    /* Render - friend */
    friend class render;

  private:
    prtype Type;
    UINT
      NumOfEl,         /* Number of indices/vecrtices */
      VA,              /* Vertex array Id */
      VBuf,            /* Vertex buffer Id */
      IBuf;            /* Index buffer Id */

  public:
    matr Trans;        /* Additional transformation matrix */
    material *Mtl;     /* Material pointer */
    vec3 MinBB, MaxBB; /* Bound box */

    /* Class constructor */
    primitive( void ) : Type(prtype::PRIM_TRIMESH), NumOfEl(0),
      VA(0), VBuf(0), IBuf(0), Trans(), MinBB(0), MaxBB(0), Mtl(nullptr)
    {
      Trans.toIdentity();
    } /* End of 'primitive' constructor */

    /* Class destructor */
    ~primitive( void )
    {
      if (this->VA != 0)
      {
        glBindVertexArray(VA);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glDeleteBuffers(1, &VBuf);
        glBindVertexArray(0);
        glDeleteVertexArrays(1, &VA);
      }
      if (this->IBuf != 0)
        glDeleteBuffers(1, &IBuf);
      VA = IBuf = 0;
      this->Mtl = nullptr;
    } /* End of 'primitive' destructor */

    /* Evaluate bound box function.
     * ARGUMENTS: 
     *   - vertex array:
     *       vertex *V;
     *   - index array:
     *       int *Ind;
     *   - num of indices:
     *       int NoofI;
     */
    void EvalBB( vertex *V, int *Ind, int NoofI )
    {
      for (int i = 0; i < NoofI; i += 3)
      {
        this->MinBB.Min(V[Ind[i]].P);
        this->MaxBB.Max(V[Ind[i]].P);
      }
    } /* End of 'EvalBB' function. */

    /* Primitive free function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Free( void )
    {
      if (VA != 0)
      {
        glBindVertexArray(VA);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glDeleteBuffers(1, &VBuf);
        glBindVertexArray(0);
        glDeleteVertexArrays(1, &VA);
      }
      if (IBuf != 0)
        glDeleteBuffers(1, &IBuf);
      VA = IBuf = 0;
      this->Mtl = nullptr;
    } /* End of 'Free' function */

    /* Primitive creation function.
     * ARGUMENTS:
     *   - primitive type:
     *       prtype Type;
     *   - vertex attributes array:
     *       vertex *V;
     *   - number of vertices:
     *       int NumOfV;
     *   - index array (for trimesh – by 3 ones, may be NULL)
     *       int *I;
     *   - number of indices
     *       int NumOfI;
     * RETURNS: None.
     */
    void Create( prtype Type, vertex *V, int NumOfV, int *I, int NumOfI, int IsBB = TRUE );

    /* Grid auto normal function.
     * ARGUMENTS:
     *   - vertex array:
     *      vertex *V;
     *   - index:
     *      int *I;
     *   - num of index:
     *      int NoofI;
     *   - height and width:
     *      int W, H;
     * RETURNS: None.
     */
    void GridAutoNorm( vertex *V, int *I, int NoofI, int W, int H )
    {
      int i, j;
    
      for (i = 0; i < H - 1; i++)
        for (j = 0; j < W - 1; j++)
        {
          vertex
            *P00 = V + i * W + j,
            *P01 = V + i * W + j + 1,
            *P10 = V + (i + 1) * W + j,
            *P11 = V + (i + 1) * W + j + 1;
          vec3 N;
    
          N = ((P00->P - P10->P) % (P11->P - P10->P)).Normalizing();
          P00->N = (P00->N + N).Normalizing();
          P10->N = (P10->N + N).Normalizing();
          P11->N = (P11->N + N).Normalizing();

          N = ((P11->P - P01->P) % (P00->P - P01->P)).Normalizing();
          P00->N = (P00->N + N).Normalizing();
          P01->N = (P01->N + N).Normalizing();
          P11->N = (P11->N + N).Normalizing();
        }
    } /* End of 'GridAutoNorm' function */
    
    /* Create primitive from grid function.
     * ARGUMENTS:
     *   - vertex buffer:
     *       vertex *V;
     *   - primitive width and height:
     *       int W, H;
     * RETURNS:
     *   (int) TRUE if success, FALSE otherwise.
     */
    int CreateFromGrid( vertex *V, int W, int H, int IsBB = TRUE )
    {
      int i, j, k;
      int *Ind;
    
      ZeroMemory(this, sizeof(primitive));

      Ind = new int[(H - 1) * (W * 2 + 1) - 1];

      for (i = 0, k = 0; i < H - 1; i++)
      {
        for (j = 0; j < W; j++)
        {
          Ind[k++] = (i + 1) * W + j;
          Ind[k++] = i * W + j;
        }
        if (i != H - 2)
          Ind[k++] = -1;
      }
      this->GridAutoNorm(V, Ind, NumOfEl, W, H);
      this->Create(prtype::PRIM_TRISTRIP, V, W * H,
        Ind, (H - 1) * (W * 2 + 1) - 1, IsBB);
      
      delete[] Ind;
      return TRUE;
    } /* End of 'DG5_RndPrimCreateFromGrid' function */

    /* Auto normalizing function.
     * ARGUMENTS:
     *   - vertex array:
     *       vertex *V;
     *   - index array:
     *       int *I;
     *   - num of indices:
     *       int NoofI;
     *   - flag to paint primitive color as normals:
     *       int IsPaint
     * RETURNS: None.
     */
    void AutoNorm( vertex *V, int *I, int NoofI, int IsPaint = FALSE )
    {
      for (int i = 0; i < NoofI; i += 3)
      {
        vec3 N = ((V[I[i + 1]].P - V[I[i]].P) % (V[I[i + 2]].P - V[I[i]].P)).Normalizing();

        V[I[i + 0]].N = ((V[I[i + 0]].N + N)).Normalizing();
        V[I[i + 1]].N = ((V[I[i + 1]].N + N)).Normalizing();
        V[I[i + 2]].N = ((V[I[i + 2]].N + N)).Normalizing();

        if (IsPaint)
        {
          V[I[i]].C = vec4(mth::Clamp(V[I[i]].N[0]), mth::Clamp(V[I[i]].N[1]), mth::Clamp(V[I[i]].N[2]), 1);
          V[I[i + 1]].C = vec4(mth::Clamp(V[I[i + 1]].N[0]), mth::Clamp(V[I[i + 1]].N[1]), mth::Clamp(V[I[i + 1]].N[2]), 1);
          V[I[i + 2]].C = vec4(mth::Clamp(V[I[i + 2]].N[0]), mth::Clamp(V[I[i + 2]].N[1]), mth::Clamp(V[I[i + 2]].N[2]), 1);
        }
      }
    } /* End of 'AutoNorm' function */

    /* Create 'Mobius lent' function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void CreateMobius( void )
    {
      std::vector<vertex> V;

      for (double u = 0; u <= 360; u += 0.2)
      {
        double siu = sin(D2R(u)), cou = cos(D2R(u)), cou2 = cos(D2R(u / 2)), siu2 = sin(D2R(u / 2));

        for (double v = -1; v <= 1; v += 0.1)
        {
          double hv = v / 2.0;
          double x, y, z;
          vertex NV;

          x = (1 + hv * cou2) * cou;
          y = 1.5 + (1 + hv * cou2) * siu;
          z = hv * siu2;

          NV.P = vec3(x, y, z);
          NV.T = vec2(x, y);

          V.push_back(NV);
        }
      }
      this->CreateFromGrid(V.data(), 1801, 21);
    } /* End of 'CreateMobius' function */

    /* Create sphere function.
     * ARGUMENTS: 
     *   - sphere radius:
     *       double R;
     * RETURNS: None.
     */
    void CreateSphere( double R, double X0 = 0.0, double Y0 = 0.0, double Z0 = 0.0, int M = 30, int N = 30 )
    {
      int i = 0, j = 0;
      std::vector<vertex> V;

      for (double th = 0, i = 0; i < N; i++, th += PI / (N - 1))
        for (double phi = 0, j = 0; j < M; j++, phi += 2 * PI / (M - 1))
        {
          double
            sit = sin(th),
            cot = cos(th),
            sip = sin(phi),
            cop = cos(phi);
          vertex NV;
          double x, y, z;

          x = X0 + R * sit * sip;
          y = Y0 + R * cot;
          z = Z0 + R * sit * cop;

          NV.P = vec3(x, y, z);
          NV.T = vec2(x, y);

          V.push_back(NV);
        }
      this->CreateFromGrid(V.data(), N, M, FALSE);
    } /* End of 'CreateSphere' function */

    /* Load primitive from '*.OBJ' file function.
     * ARGUMENTS:
     *   - '*.OBJ' file name:
     *       char *FileName;
     * RETURNS:
     *   (int) TRUE if success, FALSE otherwise.
     */
    int Load( const char *FileName )
    {
      std::fstream f;
      std::vector<vertex> V;
      std::vector<int> Ind;
      char Buf[1000];

      ZeroMemory(this, sizeof(primitive));

      f.open(FileName, std::fstream::in);
      if (!f.is_open())
        return FALSE;

      /* Load primitive */
      while (f)
      {
        f.getline(Buf, sizeof(Buf));
        if (Buf[0] == 'v' && Buf[1] == ' ')
        {
          vertex V1;
          double x{0}, y{0}, z{0};
          vec4 c(mth::Rnd0(), mth::Rnd0(), mth::Rnd0(), 1);
          std::stringstream Str;

          Str << (Buf + 2);

          Str >> x >> y >> z;

          V1.C = c;
          V1.P = vec3(x, y, z);
          V1.T = vec2(x, y);
          V.push_back(V1);
        }
        else if (Buf[0] == 'f' && Buf[1] == ' ')
        {
          int n = 0, n0 = 0, n1 = 0, nc;

          for (int i = 1; Buf[i] != 0; i++)
          {
            if (isspace((UCHAR)Buf[i - 1]) && !isspace((UCHAR)Buf[i]))
            {
              std::stringstream Str1;

              Str1 << (Buf + i);

              Str1 >> nc;
              if (nc < 0)
                nc = (int)V.size() + nc;
              else
                nc--;

              if (n == 0)
                n0 = nc;
              else if (n == 1)
                n1 = nc;
              else
              {
                Ind.push_back(n0);
                Ind.push_back(n1);
                Ind.push_back(nc);
                n1 = nc;
              }
              n++;
            }
          }
        }
      }

      /* Close file and create primitive */
      f.close();
      this->AutoNorm(V.data(), Ind.data(), (int)Ind.size(), FALSE);
      this->Create(prtype::PRIM_TRIMESH, V.data(), (int)V.size(), Ind.data(), (int)Ind.size());
      return TRUE;
    }
    /* End of 'Load' function */
  }; /* End of 'primitive' class */

  /* Model class */
  class model
  {
  private:
    std::vector<primitive *> Prims; /* Primitives stock */
  public:
    /* Default constructor */
    model( void )
    {
    } /* End of 'model' constructor */

    /* Destructor */
    ~model( void )
    {
      for (auto &pr : Prims)
        delete pr;
    } /* End of 'model' destructor */

    /* Model free function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Free( void )
    {
      for (auto& pr : Prims)
        pr->Free();
    } /* End of 'free' function */

    /* Load '.g3dm' function.
     * ARGUMENTS:
     *   - filename to load:
     *       char *FileName;
     * RETURNS: (int) TRUE if success, FALSE otherwise
     */
    int Load( const char *FileName );

    /* Set matrix to all models function.
     * ARGUMENTS:
     *   - matrix to apply:
     *       matr M;
     * RETURNS: None.
     */
    void SetTransMulty( matr M )
    {
      for (int i = 0; i < Prims.size(); i++)
        Prims[i]->Trans *= M;
    } /* End of 'SetTransMulty' function */

    /* Walk for all primitives function.
     * ARGUMENTS:
     *   - function to walk:
     *       const type &Walker;
     * RETURNS: None.
     */
    template <typename type>
    void WalkAll( const type &Walker )
    {
      for (auto& pr : Prims)
        Walker(pr);
    } /* End of 'WalkAll' function */

    /* Operator [] function.
     * ARGUMENTS:
     *   - index to find:
     *       int i;
     * RETURNS: (primitive *) result primitive pointer.
     */
    primitive * operator [] ( int i )
    {
      return Prims[i];
    }
  }; /* End of 'model' class */
} /* End of 'nirt' namespace */

#endif /* __render_primitive_h_ */

/* END OF 'render_primitive.h' FILE */