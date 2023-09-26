/* FILE NAME   : material.h
 * PURPOSE     : Animation project.
 *               Material class declaration module.
 * PROGRAMMER  : ND4
 * LAST UPDATE : 27.07.2021. 
 */

#ifndef __material_h_
#define __material_h_

#include "res.h"

#include "shader.h"
#include "texture.h"

/* Project namespace */
namespace nirt
{
  /* material class */
  class material
  {
    /* Manager - friend */
    friend class resource_manager<material, std::string>;
    friend class material_manager;
    friend class render;

  private:
    vec3 Ka;        /* Ambient coefficient */
    vec3 Kd;        /* Diffuse coefficient */
    float Trans;      /* Transparency factor */
    vec3 Ks;        /* Specular coefficient */

    float Ph;         /* Phong power coefficient */

    /* Delete material function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Free( void )
    {
      memset(this, 0, sizeof(material));
    } /* End of 'Free' function */
  public:
    shader *Shd;     /* Material shader pointer */
    texture *Tex[8]; /* Texture array */
    char Name[300];  /* Material name */

    /* Class constructor */
    material( void ) : Name("")
    {
    } /* End of 'material' constructor */

    /* Constructor by all fields */
    material( std::string MatName, vec3 RKa, vec3 RKd, 
      vec3 RKs, float RPh, std::string ShdName = "DEFAULT" );

    /* material constructor by string */
    material( std::string MaterialName )
    {
      for (int i = 0; i < 8; i++)
        Tex[i] = nullptr;
      strcpy((char *)Name, MaterialName.c_str());
    } /* End of 'material' constructor */

    /* Class destructor */
    ~material( void )
    {
    } /* End of 'material' destructor */

    void SetTrans( float NewTrans )
    {
      Trans = NewTrans;
    }

    /* Get default material static function.
     * ARGUMENTS: None.
     * RETURNS: (material &) default material
     */
    static material GetDefault( void )
    {
      material mtl("DEFAULT_MAT", vec3(0.24725, 0.2245, 0.0645),
                                     vec3(0.34615, 0.3143, 0.0903),
                                     vec3(0.797353, 0.723991, 0.208006), 83.2);
      mtl.Trans = 1;
      return mtl;
    } /* End of 'GetDefault' function */

    /* Apply material function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Apply( void );
  }; /* End of 'material' class */

  /* material manager class */
  class material_manager : public resource_manager<material, std::string>
  {
  protected:

    ~material_manager( void )
    {
    }

    /* Initialize materials function.
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
    /* Get default material function.
     * ARGUMENTS: None.
     * RETURNS: (material *) default material.
     */
    material * GetDefault( void );

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
    material * Add( const char *MaterialName, vec3 Ka, vec3 Kd, vec3 Ks, float Ph, const char *ShdName );

    /* Add material to manager function.
     * ARGUMENTS:
     *   - material to add:
     *       material NMtl;
     * RETURNS: (material *) created material.
     */
    material * Add( material &NMtl );

  }; /* End of 'material_manager' class */
} /* End of 'nirt' namespace */


#endif /* __material_h_ */

/* END OF 'material.h' FILE */