/* FILE NAME  : mth_vec4.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 17.07.2022
 * PURPOSE    : Math 4D vector support module.
 */

#ifndef __mth_vec4_h_
#define __mth_vec4_h_

#include "mthdef.h"

 /* Math namespace */
namespace mth
{
  /* 3D vector class */
  template<typename Type>
  class vec4
  {
  private:
    Type X, Y, Z, W;
  public:
    /* Default constructor. */
    vec4( void ) : X(0), Y(0), Z(0), W(0)
    {
    } /* End of 'vec4' constructor */

    /* Constructor by 1 value.
     * ARGUMENTS:
     *   - value to equal:
     *       const Type Val;
     */
    vec4( const Type Val ) : X(Val), Y(Val), Z(Val), W(Val)
    {
    } /* End of 'vec4' constructor */

    /* Constructor by 3 value.
     * ARGUMENTS:
     *   - value x:
     *       const Type ValX;
     *   - value y:
     *       const Type ValY;
     *   - value z:
     *       const Type ValZ;
     *   - value w:
     *       const Type ValW;
     */
    vec4( const Type ValX, const Type ValY, const Type ValZ, const Type ValW ) :
      X(ValX), Y(ValY), Z(ValZ), W(ValW)
    {
    } /* End of 'vec4' constructor */

    /* Vector length operator !
     * ARGUMENTS: None.
     * RETURNS: (double) vector length
     */
    double operator ! ( void )
    {
      return sqrt(X * X + Y * Y + Z * Z + W * W);
    } /* End of 'operator !' function */

    /* Vector add operator +
     * ARGUMENTS:
     *   - vector to add:
     *       const vec4 &V;
     * RETURNS: (vec4) result vector
     */
    vec4 operator + ( const vec4 &V )
    {
      return vec4(V.X + this->X, V.Y + this->Y, V.Z + this->Z, V.W + this->W);
    } /* End of 'operator +' function */

    /* Negative vector operator - (prefix)
     * ARGUMENTS: None.
     * RETURNS: (vec4) result vector
     */
    vec4 operator - ( void )
    {
      return vec4(-X, -Y, -Z, -W);
    } /* End of 'operator -' function */

    /* Vector substraction operator -
     * ARGUMENTS:
     *   - vector to substract:
     *       const vec4 &V;
     * RETURNS: (vec4) result vector
     */
    vec4 operator - ( const vec4 &V )
    {
      return vec4(X - V.X, Y - V.Y, Z - V.Z, W - V.W);
    } /* End of 'operator -' function */

    /* Vector add & equal operator +=
     * ARGUMENTS:
     *   - vec4 to add & equal:
     *       const vec4 &V;
     * RETURNS: (vec4) this pointer;
     */
    vec4 operator += ( const vec4 &V )
    {
      this->X += V.X;
      this->Y += V.Y;
      this->Z += V.Z;
      this->W += V.W;

      return *this;
    } /* End of 'operator +=' function */

    /* Vector substract & equal operator +=
     * ARGUMENTS:
     *   - vec4 to substract & equal:
     *       const vec4 &V;
     * RETURNS: (vec4) this pointer;
     */
    vec4 operator -= ( const vec4 &V )
    {
      this->X -= V.X;
      this->Y -= V.Y;
      this->Z -= V.Z;
      this->W -= V.W;

      return *this;
    } /* End of 'operator -=' function */

    /* Multiply vectors by components operator *
     * ARGUMENTS:
     *   - vec4 to myltiply:
     *       const vec4 &V;
     * RETURNS: (vec4) result vector;
     */
    vec4 operator * ( const vec4 &V )
    {
      return vec4(X * V.X, Y * V.Y, Z * V.Z, W * V.W);
    } /* End of 'operator *' function  */

    /* Vector multiply & equal operator +=
     * ARGUMENTS:
     *   - vec4 to multiply & equal:
     *       const vec4 &V;
     * RETURNS: (vec4) this pointer;
     */
    vec4 operator *= ( const vec4 &V )
    {
      this->X *= V.X;
      this->Y *= V.Y;
      this->Z *= V.Z;
      this->W *= V.W;

      return *this;
    } /* End of 'operator *=' function */

    /* Multiply vector by number function
     * ARGUMENTS:
     *   - number to multiply:
     *       const Type N;
     * RETURNS: (vec4) result vector
     */
    vec4 operator * ( const Type N )
    {
      return vec4(X * N, Y * N, Z * N, W * N);
    } /* End of 'operator *' function */

    /* Multiply & equal vector by number function.
     * ARGUMENTS:
     *   - number to multiply & equal:
     *       const Type N;
     * RETURNS: (vec4) this pointer
     */
    vec4 operator *= ( const Type N )
    {
      this->X *= N;
      this->Y *= N;
      this->Z *= N;
      this->W *= N;

      return *this;
    } /* End of 'operator *=' function */

    /* Divide vector by num function.
     * ARGUMENTS:
     *   - number to divide:
     *       const Type N;
     * RETURNS: (vec4) result vector
     */
    vec4 operator / ( const Type N )
    {
      return vec4(X / N, Y / N, Z / N, W / N);
    } /* End of 'operator /' function */

    /* Divide & equal vector by number function.
     * ARGUMENTS:
     *   - number to divide & equal:
     *       const Type N;
     * RETURNS: (vec4) this pointer
     */
    vec4 operator /= ( const Type N )
    {
      this->X /= N;
      this->Y /= N;
      this->Z /= N;
      this->W /= N;

      return *this;
    } /* End of 'operator /=' function */

    /* Dot vectors product function.
     * ARGUMENTS:
     *   - vec4 to dot:
     *       const vec4 &V;
     * RETURNS: (Type) dot product.
     */
    Type operator & ( const vec4 &V )
    {
      return X * V.X + Y * V.Y + Z * V.Z + W * V.W;
    } /* End of 'operator &' function */

    /* Normalize this vector function.
     * ARGUMENTS: None.
     * RETURNS: None.
     */
    void Normalize( void )
    {
      *this /= !*this;
    } /* End of 'Normalize' function */

    /* Get this vector normalizing function
     * ARGUMENTS: None.
     * RETURNS: (vec4) this normalizing vector
     */
    vec4 Normalizing( void )
    {
      return *this / !*this;
    } /* End of 'Normalizing' function */

    /* Sqr of vector length function.
     * ARGUMENTS: None.
     * RETURNS: (Type) this length ^ 2
     */
    Type Length2( void )
    {
      return X * X + Y * Y + Z * Z + W * W;
    } /* End of 'Length2' function */

    /* Get distance between this vector and given function
     * ARGUMENTS:
     *   - vector to get distance:
     *       const vec4 &P;
     * RETURNS: (double) distance
     */
    double Distance( const vec4 &P ) const
    {
      return sqrt((X - P.X) * (X - P.X) + (Y - P.Y) * (Y - P.X) + (Z - P.Z) * (Z - P.Z) + (W - P.W) * (W - P.W));
    } /* End of 'Distance' function */

    /* Zero vector funciton.
     * ARGUMENTS: None.
     * RETURNS: (vec4) zero vector
     */
    vec4 Zero( void ) const
    {
      return vec4(0);
    } /* End of 'Zero' function */

    /* Index operator function (const)
     * ARGUMENTS:
     *   - index:
     *       int i;
     * RETURNS (Type) vector number by index
     */
    Type operator [] ( int i ) const
    {
      switch (i)
      {
      case 0:
        return this->X;
      case 1:
        return this->Y;
      case 2:
        return this->Z;
      case 3:
        return this->W;
      default:
        if (i < 0)
          return this->X;
        else
          return this->W;
      }
    } /* End of 'operator [] function */

    /* Index operator function (lvalue)
     * ARGUMENTS:
     *   - index:
     *       int i;
     * RETURNS (Type) vector number by index
     */
    Type & operator [] ( int i )
    {
      switch (i)
      {
      case 0:
        return this->X;
      case 1:
        return this->Y;
      case 2:
        return this->Z;
      case 3:
        return this->W;
      default:
        if (i < 0)
          return this->X;
        else
          return this->W;
      }
    } /* End of 'operator [] function */

    /* vec4 --> Type * function
     * ARGUMENTS: None.
     * RETURNS: (Type *) pointer to this->X
     */
    operator Type * ( void )
    {
      return &this->X;
    } /* End of 'operator Type *' function */

    /* Ostream rewrite to output vec4.
     * ARGUMENTS:
     *   - ostream to output:
     *       std::ostream &C;
     *   - vector to output:
     *       vec4 &V;
     * RETURNS: (ostream &) result ostream
     */
    friend std::ostream & operator << ( std::ostream &C, vec4 &V )
    {
      C << "{";
      C << V.X << ", ";
      C << V.Y << ", ";
      C << V.Z << ", ";
      C << V.W << "}";

      return C;
    } /* End of 'operator <<' function */

    /* Matrix friend */
    template <typename Type1>
    friend class matr;
  }; /* End of 'vec4' class */

  typedef vec4<double> dvec4;            /* Float vector */
  typedef vec4<float> fvec4;            /* Double vector */
  typedef vec4<int> ivec4;            /* Integer vector */
} /* End of 'mth' namespace */

#endif /* __mth_vec4_h_*/

/* END OF 'mth_vec4.h' FILE */