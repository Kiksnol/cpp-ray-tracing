/**/
#version 430
#define THRESHOLD 0.01
#define INFINITY 1e+30

layout(location = 0) out vec4 OutColor;

in vec2 DrawTexCoord;
uniform vec3 CamRight;
uniform vec3 CamUp;
uniform vec3 CamDir;
uniform vec3 CamLoc;

uniform float CamSize;
uniform float CamProjDist;
uniform float FrameW;
uniform float FrameH;
uniform float Time;

float CurrentTransparency = 1;

struct ray
{
  vec3 Org;
  vec3 Dir;
};

vec3 RayApply( ray R, float T )
{
  return R.Org + R.Dir * T;
}

struct material
{
  vec3 Ka;  // Ambient
  vec3 Kd;  // Diffuse
  vec3 Ks;  // Specular
  float Ph; // Shininess
};

material MatLib[20] = {material(vec3(0), vec3(0.01), vec3(0.5), 32),                                                                            // Black plastic
                       material(vec3(0.329412,0.223529,0.027451), vec3(0.780392,0.568627,0.113725), vec3(0.992157,0.941176,0.807843), 27.8974), // Brass
                       material(vec3(0.2125,0.1275,0.054), vec3(0.714,0.4284,0.18144), vec3(0.393548,0.271906,0.166721), 25.6),                 // Bronze
                       material(vec3(0.25), vec3(0.4), vec3(0.774597), 76.8),                                                                   // Chrome
                       material(vec3(0.19125,0.0735,0.0225), vec3(0.7038,0.27048,0.0828), vec3(0.256777,0.137622,0.086014), 12.8),              // Copper
                       material(vec3(0.24725,0.1995,0.0745), vec3(0.75164,0.60648,0.22648), vec3(0.628281,0.555802,0.366065), 51.2),            // Gold
                       material(vec3(0.10588,0.058824,0.113725), vec3(0.427451,0.470588,0.541176), vec3(0.3333,0.3333,0.521569), 9.84615),      // Peweter
                       material(vec3(0.19225,0.19225,0.19225), vec3(0.50754,0.50754,0.50754), vec3(0.508273,0.508273,0.508273), 51.2),          // Silver
                       material(vec3(0.23125,0.23125,0.23125), vec3(0.2775,0.2775,0.2775), vec3(0.773911,0.773911,0.773911), 89.6),             // Polished silver
                       material(vec3(0.1, 0.18725, 0.1745), vec3(0.396, 0.74151, 0.69102), vec3(0.297254, 0.30829, 0.306678), 12.8),            // Turquoise
                       material(vec3(0.1745, 0.01175, 0.01175), vec3(0.61424, 0.04136, 0.04136), vec3(0.727811, 0.626959, 0.626959), 76.8),     // Ruby
                       material(vec3(0.24725, 0.2245, 0.0645), vec3(0.34615, 0.3143, 0.0903), vec3(0.797357, 0.723991, 0.208006), 83.2),        // Polished gold
                       material(vec3(0.25, 0.148, 0.06475), vec3(0.4, 0.2368, 0.1036), vec3(0.774597, 0.458561, 0.200621), 76.8),               // Polished bronze
                       material(vec3(0.2295, 0.08825, 0.0275), vec3(0.5508, 0.2118, 0.066), vec3(0.580594, 0.223257, 0.0695701), 51.2),         // Polished copper
                       material(vec3(0.135, 0.2225, 0.1575), vec3(0.135, 0.2225, 0.1575), vec3(0.316228), 12.8),                                // Jade
                       material(vec3(0.05375, 0.05, 0.06625), vec3(0.18275, 0.17, 0.22525), vec3(0.332741, 0.328634, 0.346435), 38.4),          // Obsidian
                       material(vec3(0.25, 0.20725, 0.20725), vec3(1.0, 0.829, 0.829), vec3(0.296648, 0.296648, 0.296648), 11.264),             // Pearl
                       material(vec3(0.0215, 0.1745, 0.0215), vec3(0.07568, 0.61424, 0.07568), vec3(0.633, 0.727811, 0.633), 76.8),             // Emerald
                       material(vec3(0.0745, 0.01175, 0.41175), vec3(0.102, 0.047, 0.30), vec3(0.633, 0.727811, 0.633), 47),                    // Blue
                       material(vec3(0.02), vec3(0.01), vec3(0.4), 10)};                                                                        // Black rubber

struct envi
{
  float RefractionKoef;
  float DecayKoef;
};

envi EnvLib[2] = {envi(0.1, 0.1), // Air environment
                  envi(-1, -1), // Plane environment
                  };

struct intr
{
  float T;          // Intersection time
  vec3 P;           // Position
  vec3 N;           // Normal
  envi Env;         // Shape environment
  material Mtl;     // Material
};

struct sphere
{
  vec3 Center;  // Sphere center point coords
  float R2;     // Squared sphere radius
  material Mtl; // Sphere material
  envi Env;     // Sphere environment
  float Trans;  // Sphere transparency
};

struct plane
{
  vec3 N;
  float D;
  material Mtl;
  envi Env; // Plane environment (Always -1 -1)
};

struct box
{
  vec3 P1, P2;
  material Mtl;
  envi Env;
};

struct triangle
{
  float D;      // Plane coef
  vec3 U1, V1;  // Triangle points
  float u0, v0; // Triangle coeffs
  material Mtl;
  envi Env;
  vec3 N; 
};

struct light_info
{
  vec3 Dir;   // Light source direction
  vec3 Color; // Light color
  float Dist; // Distance between objects
  float Att;  // Attenuation coefficient
};

struct light
{
  float Cl, Cq, Cc;  // Linear, quadratic and constant coeffs
  light_info LI;     // Light info
  float R1, R2, Cr;
  vec3 Pos;          // Light position
};

triangle MakeTriangle( vec3 P0, vec3 P1, vec3 P2, material Mtl, envi Environment )
{
  triangle Res;
  vec3 M0 = P0, P;

  Res.N = normalize(cross((P1 - P0), P2 - P0));

  Res.D = Res.N.x * M0.x + Res.N.y * M0.y + Res.N.z * M0.z;

  vec3 r = P - P0, s1 = P1 - P0, s2 = P2 - P0;
  float s12 = dot(s1, s1), s22 = dot(s2, s2);

  Res.U1 = ((s1 * s22) - (s2 * dot(s1, s2))) / ((s12 * s22) - dot(s1, s2) * dot(s1, s2));
  Res.u0 = dot(P0, Res.U1);

  Res.V1 = ((s2 * s12) - (s1 * dot(s1, s2))) / ((s12 * s22) - dot(s1, s2) * dot(s1, s2));
  Res.v0 = dot(P0, Res.V1);

  Res.Mtl = Mtl;
  Res.Env = Environment;

  return Res;
}

light Lights[2] = {light(0.02, 0, 0, light_info(vec3(0), vec3(1, 1, 1), 0, 0), 1, 30, 29, vec3(1, 20, 1)),
                   light(0.02, 0, 0, light_info(vec3(0), vec3(0.30, 0.47, 0.8), 0, 0), 1, 47, 46, vec3(2.97 + 10 * sin(Time * 10), 10.25, -9.94))};

int LightsSize = 2;

light_info Shadow( light Li, vec3 P )
{
  vec3 Dir = normalize(Li.Pos - P);
  float Dist = distance(Li.Pos, P);
  light_info Res;

  Res.Att = 1 / (Li.Cq * Dist * Dist + Li.Cl * Dist + Li.Cc);

  Res.Dir = Dir;
  Res.Color = Li.LI.Color;
  Res.Dist = Dist;
  Res.Att = min(Res.Att, 1);

  return Res;
}

ray FrameRay( float Xs, float Ys )
{
  float Wp = CamSize, Hp = CamSize;

  if (FrameW >= FrameH)
    Wp *= FrameW / FrameH;
  else
    Hp *= FrameH / FrameW;

  vec3 
    A = CamDir * CamProjDist,
    B = CamRight * (Xs - FrameW / 2.0) * Wp / FrameW,
    C = CamUp * (FrameH / 2.0 - Ys) * Hp / FrameH,
    Q = A + B + C;

  ray Res;

  Res.Dir = normalize(Q);
  Res.Org = CamLoc + Q;

  return Res;
}

bool PlaneIntersection( ray R, plane P, inout intr Intr )
{
  float nd = dot(R.Dir, P.N);

  if (abs(nd) < THRESHOLD)
    return false;
  float T = -(dot(P.N, R.Org) - P.D) / nd;
  if (T < THRESHOLD)
    return false;

  Intr.T = T;
  Intr.N = P.N;
  Intr.P = RayApply(R, Intr.T);
  Intr.Env = P.Env;


  float spheral = sin(Intr.T) + sin(Time * 3);

  if (spheral > 0.5)
    Intr.Mtl = P.Mtl;
  else
    Intr.Mtl = MatLib[9];
  
  return true;
}

bool SphereIntersection( ray R, sphere Sph, inout intr Intr )
{
  vec3 a = Sph.Center - R.Org;
  float OC2, OK, OK2, h2;

  OC2 = dot(a, a);
  OK = dot(a, R.Dir);
  OK2 = OK * OK;
  h2 = Sph.R2 - (OC2 - OK2);
  if (OC2 < Sph.R2)
  {
    Intr.T = OK + sqrt(h2);
    Intr.P = RayApply(R, Intr.T);
    Intr.N = normalize(Intr.P - Sph.Center);
    Intr.Mtl = Sph.Mtl;
    Intr.Env = Sph.Env;
    CurrentTransparency = Sph.Trans;

    return true;
  }
  if (OK < THRESHOLD || h2 < THRESHOLD)
    return false;

  Intr.T = OK - sqrt(h2);
  Intr.P = RayApply(R, Intr.T);
  Intr.N = normalize(Intr.P - Sph.Center);
  Intr.Mtl = Sph.Mtl;
  Intr.Env = Sph.Env;
  CurrentTransparency = Sph.Trans;

  return true;
}

bool TriangleIntersection( ray R, triangle Tr, inout intr Intr )
{
  Intr.T = (Tr.D - dot(Tr.N, R.Org)) / dot(Tr.N, R.Dir);
  if (Intr.T < THRESHOLD)
    return false;

  vec3 P = RayApply(R, Intr.T);
  
  float u = dot(P, Tr.U1) - Tr.u0;
  float v = dot(P, Tr.V1) - Tr.v0;
  if (u >= THRESHOLD && v >= THRESHOLD && (u + v) <= 1)
  {
    Intr.N = Tr.N;
    Intr.P = P;
    Intr.Mtl = Tr.Mtl;
    Intr.Env = Tr.Env;

    return true;
  }
  return false;
}

bool BoxIntersection( ray Ray, box Box, inout intr Intr )
{
  int f = 0, normInd = 0;
  float t0, t1, tn = -INFINITY, tf = INFINITY;

  vec3 Norm[6] =
  {
    vec3(-1, 0, 0),
    vec3(1, 0, 0),
    vec3(0, -1, 0),
    vec3(0, 1, 0),
    vec3(0, 0, -1),
    vec3(0, 0, 1),
  };

  // X  
  if (abs(Ray.Dir.x) < 1E-3 && (Ray.Org.x < Box.P1.x || Ray.Org.x > Box.P2.x))
    return false;

  t0 = (Box.P1.x - Ray.Org.x) / Ray.Dir.x;
  t1 = (Box.P2.x - Ray.Org.x) / Ray.Dir.x;

  if (t0 > t1)
  {
    float T = t0;

    t0 = t1;
    t1 = T;
    f = 1;
  }

  if (t0 > tn)
  {
    if (f == 1)
      normInd = 1;
    else
      normInd = 0;

    tn = t0;
  }

  if (tf > t1)
    tf = t1;
  if (tn > tf || tf < 0)
    return false;
  
  f = 0;

  // Y
  if (abs(Ray.Dir.y) < THRESHOLD && (Ray.Org.y < Box.P1.y || Ray.Org.y > Box.P2.y))
    return false;

  t0 = (Box.P1.y - Ray.Org.y) / Ray.Dir.y;
  t1 = (Box.P2.y - Ray.Org.y) / Ray.Dir.y;

  if (t0 > t1)
  {
    float T = t0;

    t0 = t1;
    t1 = T;
    f = 1;
  }
  if (t0 > tn)
  {
    if (f == 1)
      normInd = 3;
    else
      normInd = 2;

    tn = t0;
  }
  if (tf > t1)
    tf = t1;

  if (tn > tf || tf < 0)
    return false;
  
  f = 0;

  // Z
  if (abs(Ray.Dir.z) < THRESHOLD && (Ray.Org.z < Box.P1.z || Ray.Org.z > Box.P2.z))
    return false;

  t0 = (Box.P1.z - Ray.Org.z) / Ray.Dir.z;
  t1 = (Box.P2.z - Ray.Org.z) / Ray.Dir.z;

  if (t0 > t1)
  {
    float T = t0;

    t0 = t1;
    t1 = T;
    f = 1;
  }
  if (t0 > tn)
  {
    if (f == 1)
      normInd = 5;
    else
      normInd = 4;

    tn = t0;
  }

  if (tf > t1)
    tf = t1;
  if (tn > tf || tf < 0)
    return false;

  Intr.T = tn;
  Intr.N = Norm[normInd];
  Intr.P = RayApply(Ray, Intr.T);
  Intr.Mtl = Box.Mtl;
  Intr.Env = Box.Env;

  return true;
}

// Shapes
sphere Sph[2] = {sphere(vec3(10, 8, 0), 10, MatLib[16], envi(0.1, 1), 1),
                 sphere(vec3(-10, 6, 0), 5, MatLib[10], envi(0.1, 2), 1)};
int NumOfSph = 2;

plane Planes[1] = {plane(vec3(0, 1, 0), 3, MatLib[1], EnvLib[1])};
int NumOfPlanes = 1;

box Boxes[2] = {box(vec3(-9, 4, 0), vec3(-5, 6, -7), MatLib[6], envi(0.1, 0.4)),
                box(vec3(8, 15, -3), vec3(4, 10, 3), MatLib[8], EnvLib[1])};
int NumOfBoxes = 1;

triangle Tris[1] = {MakeTriangle(vec3(3, 10, 5), vec3(6, 10, 5), vec3(4.5, 15, 8), MatLib[12], envi(0.1, 0.4))};
int NumOfTris = 1;

bool Intersection( ray R, inout intr Intr )
{
  intr ci, ti;

  ci.T = INFINITY;

  // Sphere intersection
  for (int i = 0; i < NumOfSph; i++)
    if (SphereIntersection(R, Sph[i], ti) && ci.T > ti.T)
      ci = ti;

  // Plane intersection
  for (int i = 0; i < NumOfPlanes; i++)
    if (PlaneIntersection(R, Planes[i], ti) && ci.T > ti.T)
        ci = ti;

  // Box intersection
  for (int i = 0; i < NumOfBoxes; i++)
    if (BoxIntersection(R, Boxes[i], ti) && ci.T > ti.T)
      ci = ti;

  // Triangle intersection
  for (int i = 0; i < NumOfTris; i++)
    if (TriangleIntersection(R, Tris[i], ti) && ci.T > ti.T)
      ci = ti;


  Intr = ci;
  return ci.T != INFINITY;
}

vec3 Trace( ray R, envi Media, float Weight = 1.0 );
vec3 TraceRecursive1( ray R, envi Media, float Weight = 1.0 );
vec3 TraceRefract1( ray R, envi Media, float Weight = 1.0 );

vec3 Shade( vec3 Dir, envi Media, inout intr Intr, float Weight = 1 )
{
  light_info L;
  vec3 diff = vec3(0), specular = vec3(0), color = vec3(0);

  if (Weight < THRESHOLD)
    return vec3(0);

  float vn = dot(Intr.N, Dir);
  if (vn > 0)
  {
    vn = -vn;
    Intr.N = -Intr.N;
  }
  
  vec3 R = Dir - Intr.N * (2 * dot(Dir, Intr.N));
  for (int i = 0; i < LightsSize; i++)
  {
    L = Shadow(Lights[i], Intr.P);

    // Shadows
    intr ISh;
    ray Sh = ray(Intr.P + L.Dir * 0.001, L.Dir);
    
    if (Intersection(Sh, ISh))
      L.Color *= 0.1;

    // Diffuse
    float nl = dot(Intr.N, L.Dir);

    if (nl > THRESHOLD)
      diff += L.Color * nl * L.Att;

    // Specular
    float rl = dot(R, L.Dir);

    if (rl > THRESHOLD)
      specular += L.Color * pow(rl, Intr.Mtl.Ph);
  }

  // Reflection
  float wr = Weight * 0.2;
  if (wr > 0.01)
  {
    ray RR;
    RR.Org = Intr.P + R * THRESHOLD;
    RR.Dir = R;
  
    color += TraceRecursive1(RR, Media, wr * Media.DecayKoef) * 0.2; 
  }
  
  // Refraction
  /*if (Intr.Env.RefractionKoef != - 1)
  {
    float 
      cs = dot(Dir, Intr.N),
      n = Intr.Env.RefractionKoef / Media.RefractionKoef,
      coef = 1 - (1 - cs * cs) * n * n;
    vec3 T = (Dir - Intr.N * dot(Dir, Intr.N)) * n - Intr.N * sqrt(coef);

    ray RRFR;

    RRFR.Org = Intr.P + T * THRESHOLD;
    RRFR.Dir = T;

    color += TraceRefract1(RRFR, Media, 1);
  }*/
  if (Intr.Env.RefractionKoef != -1)
  {
    float eta1 = Media.RefractionKoef, eta2 = Intr.Env.RefractionKoef;
    envi OutEnv = Intr.Env;
    if (dot(Intr.N, Dir) > 0)
      eta2 = 1, OutEnv = EnvLib[0];
    vec3 T = (-Intr.N * dot(Intr.N, Dir) + Dir) * eta2 / eta1 -
      Intr.N * sqrt(1 - (1 - dot(Intr.N, Dir) * dot(Intr.N, Dir)) * eta2 * eta2 / (eta1 * eta1));
    ray RRFR;
    
    RRFR.Org = Intr.P + T * THRESHOLD;
    RRFR.Dir = T;

    color += TraceRefract1(RRFR, OutEnv, Weight * Intr.Env.DecayKoef) * exp(-Intr.T * Media.DecayKoef) * 1.9;
  }

  color += Intr.Mtl.Ka + diff * Intr.Mtl.Kd + specular * Intr.Mtl.Ks;

  return 10 * color / distance(Lights[0].Pos, Intr.P);
}

vec3 Trace( ray R, envi Media, float Weight = 1.0 )
{
  if (Weight < THRESHOLD)
    return vec3(0);
  intr Intr;

  if (Intersection(R, Intr))
    return Shade(R.Dir, Media, Intr, Weight);

  return vec3(0);
}

vec3 ShadeRefract1( vec3 Dir, envi Media, inout intr Intr, float Weight = 1 );

vec3 TraceRefract1( ray R, envi Media, float Weight = 1.0 )
{
  if (Weight < THRESHOLD)
    return vec3(0);
  intr Intr;

  if (Intersection(R, Intr))
    return ShadeRefract1(R.Dir, Media, Intr, Weight);

  return vec3(0);
}

vec3 ShadeRefract2( vec3 Dir, envi Media, inout intr Intr, float Weight = 1 );

vec3 TraceRefract2( ray R, envi Media, float Weight = 1.0 )
{
  if (Weight < THRESHOLD)
    return vec3(0);
  intr Intr;

  if (Intersection(R, Intr))
    return ShadeRefract2(R.Dir, Media, Intr, Weight);

  return vec3(0);
}

vec3 ShadeRefract1( vec3 Dir, envi Media, inout intr Intr, float Weight = 1 )
{
  light_info L;
  vec3 diff = vec3(0), specular = vec3(0), color = vec3(0);

  if (Weight < THRESHOLD)
    return vec3(0);

  float vn = dot(Intr.N, Dir);
  if (vn > 0)
  {
    vn = -vn;
    Intr.N = -Intr.N;
  }
  
  vec3 R = Dir - Intr.N * (2 * dot(Dir, Intr.N));
  for (int i = 0; i < LightsSize; i++)
  {
    L = Shadow(Lights[i], Intr.P);

    // Shadows
    intr ISh;
    ray Sh = ray(Intr.P + L.Dir * 0.001, L.Dir);
    
    if (Intersection(Sh, ISh))
      L.Color *= 0.1;

    // Diffuse
    float nl = dot(Intr.N, L.Dir);

    if (nl > THRESHOLD)
      diff += L.Color * nl * L.Att;

    // Specular
    float rl = dot(R, L.Dir);

    if (rl > THRESHOLD)
      specular += L.Color * pow(rl, Intr.Mtl.Ph);
  }

  // Reflection
  float wr = Weight * 0.2;
  if (wr > 0.01)
  {
    ray RR;
    RR.Org = Intr.P + R * THRESHOLD;
    RR.Dir = R;
  
    color += TraceRecursive1(RR, Media, wr) * 0.2; 
  }
  
  // Refraction
  if (Intr.Env.RefractionKoef != -1)
   {
     float eta1 = Media.RefractionKoef, eta2 = Intr.Env.RefractionKoef;
     envi OutEnv = Intr.Env;
     if (dot(Intr.N, Dir) > 0)
       eta2 = 1, OutEnv = EnvLib[0];
     vec3 T = (-Intr.N * dot(Intr.N, Dir) + Dir) * eta2 / eta1 -
       Intr.N * sqrt(1 - (1 - dot(Intr.N, Dir) * dot(Intr.N, Dir)) * eta2 * eta2 / (eta1 * eta1));
     ray RRFR;
     
     RRFR.Org = Intr.P + T * THRESHOLD;
     RRFR.Dir = T;
  
     color += TraceRefract2(RRFR, OutEnv, Weight * Intr.Env.DecayKoef) * exp(-Intr.T * Media.DecayKoef) * 1.9;
   }

  color += Intr.Mtl.Ka + diff * Intr.Mtl.Kd + specular * Intr.Mtl.Ks;

  return 10 * color / distance(Lights[0].Pos, Intr.P);
}

vec3 ShadeRefract2( vec3 Dir, envi Media, inout intr Intr, float Weight = 1 )
{
  light_info L;
  vec3 diff = vec3(0), specular = vec3(0), color = vec3(0);

  if (Weight < THRESHOLD)
    return vec3(0);

  float vn = dot(Intr.N, Dir);
  if (vn > 0)
  {
    vn = -vn;
    Intr.N = -Intr.N;
  }
  
  vec3 R = Dir - Intr.N * (2 * dot(Dir, Intr.N));
  for (int i = 0; i < LightsSize; i++)
  {
    L = Shadow(Lights[i], Intr.P);

    // Shadows
    intr ISh;
    ray Sh = ray(Intr.P + L.Dir * 0.001, L.Dir);
    
    if (Intersection(Sh, ISh))
      L.Color *= 0.1;

    // Diffuse
    float nl = dot(Intr.N, L.Dir);

    if (nl > THRESHOLD)
      diff += L.Color * nl * L.Att;

    // Specular
    float rl = dot(R, L.Dir);

    if (rl > THRESHOLD)
      specular += L.Color * pow(rl, Intr.Mtl.Ph);
  }

  // Reflection
  float wr = Weight * 0.2;
  if (wr > 0.01)
  {
    ray RR;
    RR.Org = Intr.P + R * THRESHOLD;
    RR.Dir = R;
  
    color += TraceRecursive1(RR, Media, wr) * 0.2; 
  }

  color += Intr.Mtl.Ka + diff * Intr.Mtl.Kd + specular * Intr.Mtl.Ks;

  return 10 * color / distance(Lights[0].Pos, Intr.P);
}

vec3 ShadeRecursive1( vec3 Dir, envi Media, inout intr Intr, float Weight = 1 );

vec3 TraceRecursive1( ray R, envi Media, float Weight = 1.0 )
{
  if (Weight < THRESHOLD)
    return vec3(0);
  intr Intr;

  if (Intersection(R, Intr))
    return ShadeRecursive1(R.Dir, Media, Intr, Weight);

  return vec3(0);
}

vec3 ShadeRecursive2( vec3 Dir, envi Media, inout intr Intr, float Weight = 1 );

vec3 TraceRecursive2( ray R, envi Media, float Weight = 1.0 )
{
  if (Weight < THRESHOLD)
    return vec3(0);
  intr Intr;

  if (Intersection(R, Intr))
    return ShadeRecursive2(R.Dir, Media, Intr, Weight);

  return vec3(0);
}

vec3 ShadeRecursive1( vec3 Dir, envi Media, inout intr Intr, float Weight = 1 )
{
  light_info L;
  vec3 diff = vec3(0), specular = vec3(0), color = vec3(0);

  if (Weight < THRESHOLD)
    return vec3(0);

  float vn = dot(Intr.N, Dir);
  if (vn > 0)
  {
    vn = -vn;
    Intr.N = -Intr.N;
  }

  vec3 R = Dir - Intr.N * (2 * dot(Dir, Intr.N));

  for (int i = 0; i < LightsSize; i++)
  {
    L = Shadow(Lights[i], Intr.P);

    // Shadows
    intr ISh;
    ray Sh = ray(Intr.P + L.Dir * 0.001, L.Dir);
    
    if (Intersection(Sh, ISh))
      L.Color *= 0.1;

    // Diffuse
    float nl = dot(Intr.N, L.Dir);

    if (nl > THRESHOLD)
      diff += L.Color * nl * L.Att;

    // Specular
    float rl = dot(R, L.Dir);

    if (rl > THRESHOLD)
      specular += L.Color * pow(rl, Intr.Mtl.Ph);
  }

  color += Intr.Mtl.Ka + diff * Intr.Mtl.Kd + specular * Intr.Mtl.Ks;

  // Reflect
  float wr = Weight * 0.8;
  
  if (wr > THRESHOLD)
  {
    ray RR = ray(Intr.P + R * THRESHOLD, R);
  
    color += TraceRecursive2(RR, Media, wr) * 0.8;
  }

  return 10 * color / distance(Lights[0].Pos, Intr.P);
}

vec3 ShadeRecursive2( vec3 Dir, envi Media, inout intr Intr, float Weight = 1 )
{
  light_info L;
  vec3 diff = vec3(0), specular = vec3(0), color = vec3(0);

  if (Weight < THRESHOLD)
    return vec3(0);

  float vn = dot(Intr.N, Dir);
  if (vn > 0)
  {
    vn = -vn;
    Intr.N = -Intr.N;
  }

  vec3 R = Dir - Intr.N * (2 * dot(Dir, Intr.N));

  for (int i = 0; i < LightsSize; i++)
  {
    L = Shadow(Lights[i], Intr.P);

    // Shadows
    intr ISh;
    ray Sh = ray(Intr.P + L.Dir * 0.001, L.Dir);
    
    if (Intersection(Sh, ISh))
      L.Color *= 0.1;

    // Diffuse
    float nl = dot(Intr.N, L.Dir);

    if (nl > THRESHOLD)
      diff += L.Color * nl * L.Att;

    // Specular
    float rl = dot(R, L.Dir);

    if (rl > THRESHOLD)
      specular += L.Color * pow(rl, Intr.Mtl.Ph);
  }

  color += Intr.Mtl.Ka + diff * Intr.Mtl.Kd + specular * Intr.Mtl.Ks;

  return 10 * color / distance(Lights[0].Pos, Intr.P);
}

void main( void )
{
  vec2 Coord = DrawTexCoord;
  int X = int(Coord.x * FrameW);
  int Y = int((1 - Coord.y) * FrameH);
  ray CamRay = FrameRay(X + 0.5, Y + 0.5);

  //triangle Tr1 = MakeTriangle
  //
  //TrS[1] = Tr1;

  OutColor = vec4(Trace(CamRay, EnvLib[0]), CurrentTransparency);
}