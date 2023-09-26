/* FILE NAME  : units.cpp
 * PROGRAMMER : ND4
 * LAST UPDATE: 18.07.2022
 * PURPOSE    : WinAPI preferences
 */

#include "units.h"

namespace nirt
{
  namespace units
  {
    fr::fr( void )
    {
      frame = new primitive;
      frame->Create(prtype::PRIM_POINTS, nullptr, 1, nullptr, 0, FALSE);
      frame->Mtl = anim::GetPtr()->material_manager::Add("RT primitive material", vec3(), vec3(), vec3(), 12, "default");
    }

    void fr::Render(anim *Ani)
    {
      Ani->PrimitiveDraw(frame);
    }

    void ctrl::Response( anim *Ani )
    {
      Dir = (Ani->cam.At - Ani->cam.Loc).Normalizing();
      Right = (Dir % vec3(0, 1, 0)).Normalizing();
      Up = (Right % Dir); /* Up */

      if (Ani->KeysClick['F'])
        Ani->FullScreen();


      float Time = Ani->DeltaTime * Ani->FPS / 30.0f;

      mth::vec3<float> Right = Ani->cam.Right;
      Right[1] = 0;
      mth::vec3<float> Dir = Ani->cam.Dir;
      Dir[1] = 0;

      if (Ani->KeysClick[27])
        exit(30);

      if (Ani->Mx != 0 && Ani->Keys[VK_LBUTTON])
      {
        Ani->cam.At -= Ani->cam.Loc;
        Ani->cam.At = mth::matr<float>::Rotate(Ani->Mdx / 30.0, Ani->cam.Up).TransformVector(Ani->cam.At);
        Ani->cam.Right = mth::matr<float>::Rotate(Ani->Mdx / 30.0, Ani->cam.Up).TransformVector(Ani->cam.At);
        Ani->cam.At += Ani->cam.Loc;
        Ani->cam.SetLocAtUp(Ani->cam.Loc, Ani->cam.At);
      }
      if (Ani->Mdy != 0 && Ani->Keys[VK_LBUTTON])
      {
        Ani->cam.At -= Ani->cam.Loc;
        Ani->cam.At = mth::matr<float>::Rotate(Ani->Mdy / 30.0, Ani->cam.Right).TransformVector(Ani->cam.At);
        Ani->cam.At += Ani->cam.Loc;
        Ani->cam.SetLocAtUp(Ani->cam.Loc, Ani->cam.At);
      }
      if (Ani->Keys[VK_RBUTTON])
      {
        float Hp, Wp = Hp = Ani->cam.ProjDist;

        if (Ani->cam.FrameW > Ani->cam.FrameH)
          Wp *= static_cast<float>(Ani->cam.FrameW) / Ani->cam.FrameH;
        else
          Hp *= static_cast<float>(Ani->cam.FrameH) / Ani->cam.FrameW;

        float Dist = !(Ani->cam.At - Ani->cam.Loc);

        float sx = -Ani->Mdx * Wp / Ani->cam.FrameW * Dist / Ani->cam.ProjDist;
        float sy = Ani->Mdy * Hp / Ani->cam.FrameH * Dist / Ani->cam.ProjDist;

        Ani->cam.Move((Ani->cam.Right * sx + Ani->cam.Up * sy) / 5);
      }
      if (Ani->Keys['W'])
        Ani->cam.Move(Dir * Time);
      if (Ani->Keys['S'])
        Ani->cam.Move(-Dir * Time);
      if (Ani->Keys['D'])
        Ani->cam.Move(Right * Time);
      if (Ani->Keys['A'])
        Ani->cam.Move(-Right * Time);
      if (Ani->Keys[VK_SPACE])
        Ani->cam.Move(vec3(0, Time, 0));
      if (Ani->Keys[VK_SHIFT])
        Ani->cam.Move(vec3(0, -Time, 0));
      Ani->cam.Move(Dir * Ani->Mdz * Time / 10);

      if (Ani->KeysClick['U'])
        Ani->shader_manager::UpdateShaders();
    }
  }
}

/* END OF 'units.cpp' FILE */