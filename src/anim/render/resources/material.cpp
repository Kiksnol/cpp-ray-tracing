/* FILE NAME   : material.cpp
 * PURPOSE     : Animation project.
 *               Material functions module.
 * PROGRAMMER  : ND4
 * LAST UPDATE : 27.07.2021. 
 */

#include "material.h"

#include "../../anim.h"

/* Project namespace */
namespace nirt
{
  /* Constructor by all fields */
  material::material( std::string MatName, vec3 RKa, vec3 RKd, 
    vec3 RKs, float RPh, std::string ShdName ) : Ka(RKa), Kd(RKd), Ks(RKs), Ph(RPh) 
  {
    for (int i = 0; i < 8; i++)
      Tex[i] = nullptr;
    strcpy((char *)Name, MatName.c_str());
    this->Shd = anim::GetPtr()->shader_manager::Add(ShdName.c_str());
  } /* End of 'material' constructor */

  /* Add material to manager function.
   * ARGUMENTS:
   *   - material name to add:
   *       const char *MaterialName;
   *   - illumination coefficients:
   *       vec3 Ka, Kd, Ks; float Ph;
   *   - shader name to load shader:
   *       const char *ShdName;
   * RETURNS: (material *) created material.
   */
  material * material_manager::Add( const char *MaterialName, vec3 Ka, vec3 Kd, vec3 Ks, float Ph, 
    const char *ShdName )
  {
    material *fMtl = nullptr;
    material Mtl(MaterialName, Ka, Kd, Ks, Ph, ShdName);
  
    if ((fMtl = Find(Mtl.Name)) != nullptr)
      return fMtl;
  
    return resource_manager::Add(Mtl);
  } /* End of 'material_manager::Add' function. */

  /* Add material to manager function.
   * ARGUMENTS:
   *   - material to add:
   *       material &NMtl;
   * RETURNS: (material *) created material.
   */
  material * material_manager::Add( material &NMtl )
  {
    material *fMtl = nullptr;
  
    if ((fMtl = Find(NMtl.Name)) != nullptr)
      return fMtl;
  
    return resource_manager::Add(NMtl);
  } /* End of 'material_manager::Add' function. */

  /* Get default material function.
   * ARGUMENTS: None.
   * RETURNS: (material *) default material.
   */
  material * material_manager::GetDefault( void )
  {
    material *def = nullptr;

    if ((def = Find("DEFAULT_MAT")) != nullptr)
      return def;

    material mtl("DEFAULT_MAT", vec3(0.24725, 0.2245, 0.0645),
                                vec3(0.34615, 0.3143, 0.0903),
                                vec3(0.797353, 0.723991, 0.208006), 83.2);
    mtl.SetTrans(1);

    return resource_manager::Add(mtl);
  } /* End of 'material_manager::GetDefault' function. */

  /* Apply material function.
   * ARGUMENTS: None.
   * RETURNS: None.
   */
  void material::Apply( void )
  {
    Shd->Apply();
    Shd->UniSet("Ka", Ka);
    Shd->UniSet("Kd", Kd);
    Shd->UniSet("Ks", Ks);
    Shd->UniSet("Ph", Ph);
    Shd->UniSet("Trans", Trans);
    Shd->UniSet("CamLoc", anim::GetPtr()->cam.Loc);
    Shd->UniSet("CamRight", anim::GetPtr()->cam.Right);
    Shd->UniSet("CamDir", anim::GetPtr()->cam.Dir);
    Shd->UniSet("CamUp", anim::GetPtr()->cam.Up);
    Shd->UniSet("CamSize", anim::GetPtr()->cam.Size);
    Shd->UniSet("CamProjDist", anim::GetPtr()->cam.ProjDist);
    Shd->UniSet("Time", anim::GetPtr()->Time);

    /* Set textures */
    for (int i = 0; i < 8; i++)
    {
      char tname[] = "IsTexture0";

      tname[9] = '0' + i;
      if (Tex[i] != nullptr)
      {
        /* Activate sampler */
        glActiveTexture(GL_TEXTURE0 + i);
        /* Bind texture to sampler */
        glBindTexture(GL_TEXTURE_2D, Tex[i]->TexId);
      }
      if (Tex[i] != nullptr)
      {
        int val = Tex[i]->TexId != -1;
        Shd->UniSet(tname, val);
      }
    }
  } /* End of 'material::Apply' function */
}

/* END OF 'material.cpp' FILE */