/* FILE NAME  : render.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 23.07.2021
 * PURPOSE    : Animation system.
 *              Rendering subsystem module.
 */

#include "render.h"

#include "../anim.h"

#include <fstream>

namespace nirt
{
  /* Frame start function.
   * ARGUMENTS: None.
   * RETURNS: None.
   */
  void render::FrameStart( void )
  {
    /*static double ReloadTime = 0;
  
    if ((ReloadTime += anim::GetPtr()->GlobalDeltaTime) > 3)
    {
      this->UpdateShaders();
      ReloadTime = 0;
    }*/
  
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  } /* End of 'render::FrameStart' function */

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
  void primitive::Create( prtype Type, vertex *V, int NumOfV, int *I, int NumOfI, int IsBB )
  {
    ZeroMemory(this, sizeof(primitive));
    this->Type = Type;

    if (V != NULL && NumOfV != 0)
    {
      glGenBuffers(1, &this->VBuf);
      glGenVertexArrays(1, &this->VA);

      glBindVertexArray(this->VA);
      glBindBuffer(GL_ARRAY_BUFFER, this->VBuf);
      glBufferData(GL_ARRAY_BUFFER, sizeof(vertex) * NumOfV, V, GL_STATIC_DRAW);

      glVertexAttribPointer(0, 3, GL_FLOAT, FALSE, sizeof(vertex),
        (void *)0); /* position */
      glVertexAttribPointer(1, 2, GL_FLOAT, FALSE, sizeof(vertex),
        (void *)sizeof(vec3)); /* texture coordinates */
      glVertexAttribPointer(2, 3, GL_FLOAT, FALSE, sizeof(vertex),
        (void *)(sizeof(vec3) + sizeof(vec2))); /* normal */
      glVertexAttribPointer(3, 4, GL_FLOAT, FALSE, sizeof(vertex),
        (void *)(sizeof(vec3) * 2 + sizeof(vec2))); /* color */

      glEnableVertexAttribArray(0);
      glEnableVertexAttribArray(1);
      glEnableVertexAttribArray(2);
      glEnableVertexAttribArray(3);
      glBindVertexArray(0);
    }

    if (I != NULL && NumOfI != 0)
    {
      glGenBuffers(1, &this->IBuf);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->IBuf);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * NumOfI, I, GL_STATIC_DRAW);
      this->NumOfEl = NumOfI;
    }

    if (I != nullptr && V != nullptr &&
        NumOfI != 0  && NumOfV != 0)
      if (IsBB)
        this->EvalBB(V, I, NumOfI);
    if (I == nullptr || NumOfI == 0)
      this->NumOfEl = NumOfV;
    this->Trans.toIdentity();

    this->Mtl = anim::GetPtr()->GetDefault();
  } /* End of 'primitive::Create' function */

  /* Load '.g3dm' function.
   * ARGUMENTS:
   *   - filename to load:
   *       char *FileName;
   * RETURNS: (int) TRUE if success, FALSE otherwise
   */
  int model::Load( const char *FileName )
  {
    unsigned char *mem, *ptr;
    int flen, p;// t, m;
    std::fstream f;
    DWORD Sign;         /* == "G3DM" */
    int NumOfPrims;
    int NumOfMaterials;
    int NumOfTextures;
    int *MtlNums;
    struct tagG3DMMTL
    {
      char Name[300];     /* Material name */
    
      /* Illumination coefficients */
      vec3 Ka, Kd, Ks; /* Ambient, diffuse, specular coefficients */
      float Ph;          /* Phong power coefficient – shininess */
      float Trans;       /* Transparency factor */
      DWORD Tex[8];    /* Texture references 
                            * (8 time: texture number in G3DM file, -1 if no texture) */
    
      /* Shader information */
      char ShaderString[300]; /* Additional shader information */
      DWORD Shader;           /* Shader number (uses after load into memory) */
    } *mtls;
    struct tagTEX1
    {
      char Name[300]; /* Texture name */
      DWORD W, H;     /* Texture image size in pixels */
      DWORD C;        /* Texture image components (1-mono, 3-bgr or 4-bgra) */
    } *tex, *AllTex;
    int TexNo = 0;
    struct tagTEXN
    {
      int Nums[300];
    } *Textures;

    ZeroMemory(this, sizeof(model));
    /* Open file */
    f.open(FileName, std::fstream::in | std::fstream::binary);

    /* Obtain file length */
    f.seekg(0, std::fstream::end);
    flen = f.tellg();
    f.seekg(0, std::fstream::beg);

    /* Allocate memory and load whole file to memory */
    try
    {
      mem = new unsigned char[flen];
    }
    catch (...)
    {
      f.close();
      return FALSE;
    }

    f.read((char *)mem, flen);
    f.close();
    ptr = mem;
    rdr rd(ptr);

    rd(&Sign);
    if (Sign != *(DWORD *)"G3DM")
    {
      delete [] mem;
      return FALSE;
    }
    rd(&NumOfPrims);
    rd(&NumOfMaterials);
    if (NumOfPrims)
      MtlNums = new int[NumOfPrims];
    else
      MtlNums = nullptr;
    if (NumOfMaterials)
      Textures = new tagTEXN[NumOfMaterials];
    else
      Textures = nullptr;
    rd(&NumOfTextures);
    if (NumOfTextures)
      AllTex = new tagTEX1[NumOfTextures];
    else
      AllTex = nullptr;

    /* Primitives */
    for (p = 0; p < NumOfPrims; p++)
    {
      primitive *Pr = new primitive;
      DWORD NumOfVertexes;
      DWORD NumOfFacetIndexes;  /* num of facets * 3 */
      DWORD MtlNo;              /* Material number in table below (in material section) */
      vertex *V;
      int *Ind;

      rd(&NumOfVertexes);
      rd(&NumOfFacetIndexes);
      rd(&MtlNo);
      V = new vertex[NumOfVertexes];
      rd(V, NumOfVertexes);
      Ind = new int[NumOfFacetIndexes];
      rd(Ind, NumOfFacetIndexes);
      Pr->AutoNorm(V, Ind, NumOfFacetIndexes, TRUE);
      Pr->Create(prtype::PRIM_TRIMESH, V, NumOfVertexes,
        Ind, NumOfFacetIndexes);
      Prims.push_back(Pr);
      if (MtlNums)
        MtlNums[p] = MtlNo;
      delete[] V;
      delete[] Ind;
    }

    /* Materials */
    mtls = (struct tagG3DMMTL *)ptr, ptr += sizeof(struct tagG3DMMTL) * NumOfMaterials;
    for (int m = 0; m < NumOfMaterials; m++)
    {
      material mtl;

      mtl = material(mtls[m].Name, mtls[m].Ka, mtls[m].Kd,
                     mtls[m].Ks, mtls[m].Ph);
      mtl.SetTrans(mtls[m].Trans);
      if (Textures)
      {
        for (int i = 0; i < 8; i++)
          Textures[m].Nums[i] = (mtls[m].Tex[i] == -1 ? -1 : mtls[m].Tex[i]);
      }
      anim::GetPtr()->material_manager::Add(mtl);
    }
    /* Textures */
    for (int t = 0; t < NumOfTextures; t++)
    {
      tex = new tagTEX1;
      rd(tex);
      if (AllTex)
        AllTex[TexNo++] = *tex;
      anim::GetPtr()->texture_manager::Add(tex->Name, tex->W, tex->H, tex->C, ptr);
      ptr += tex->W * tex->H * tex->C;
      delete tex;
    }
    /* Apply materials to prims */
    for (p = 0; p < NumOfPrims; p++)
    {
      Prims[p]->Mtl = anim::GetPtr()->material_manager::Find(mtls[MtlNums[p]].Name);
      for (int m = 0; m < NumOfMaterials; m++)
      {
        for (int t = 0; t < NumOfTextures; t++)
        {
          int TNum = Textures[m].Nums[t];

          if (TNum < 0 || TNum >= 8)
            continue;
          int Num = mtls[MtlNums[p]].Tex[TNum];

          if (Num < 0 || Num > NumOfTextures)
            continue;

          tagTEX1 tex0 = AllTex[Num];

          if (strcmp(tex0.Name, "Flying drone_Nor_2.jpg") == 0)
            strcpy(tex0.Name, "Flying drone_col.jpg");

          if (Prims[p]->Mtl->Tex[t] == nullptr)
            Prims[p]->Mtl->Tex[t] = anim::GetPtr()->texture_manager::Find(tex0.Name);
        }
      }
    }
    delete[] mem;
    delete[] MtlNums;
    delete[] Textures;
    delete[] AllTex;
    return TRUE;
  } /* End of 'model::Load' function. */
}

/* END OF 'render.cpp' FILE */