/* FILE NAME  : mth_vec2.h
 * PROGRAMMER : ND4
 * LAST UPDATE: 17.07.2022
 * PURPOSE    : Math 2D vector support module.
 */

#ifndef __mth_vec2_h_
#define __mth_vec2_h_

#include "mthdef.h"

 /* Math namespace */
namespace mth
{
  /* 2D vector class */
  template<typename Type>
  class vec2
  {
  private:
    Type X, Y;
  public:
    /* Default constructor. */
    vec2( void ) : X(0), Y(0)
    {
    } /* End of 'vec2' constructor */

    /* Constructor by 1 value.
     * ARGUMENTS:
     *   - value to equal:
     *       const Type Val;
     */
    vec2( const Type Val ) : X(Val), Y(Val)
    {
    } /* End of 'vec2' constructor */

    /* Constructor by 3 value.
     * ARGUMENTS:
     *   - value x:
     *       const Type ValX;
     *   - value y:
     *       const Type ValY;
     */
    vec2( const Type ValX, const Type ValY ) :
      X(ValX), Y(ValY)
    {
    } /* End of 'vec2' constructor */

    /* Vector length operator !
     * ARGUMENTS: None.
     * RETURNS: (double) vector length
     */
    double operator ! (void)
    {
      return sqrt(X * X + Y * Y);
    } /* End of 'operator !' function */

    /* Vector add operator +
     * ARGUMENTS:
     *   - vector to add:
     *       const vec2 &V;
     * RETURNS: (vec2) result vector
     */
    vec2 operator + ( const vec2 &V )
    {
      return vec2(V.X + this->X, V.Y + this->Y);
    } /* End of 'operator +' function */

    /* Negative vector operator - (prefix)
     * ARGUMENTS: None.
     * RETURNS: (vec2) result vector
     */
    vec2 operator - ( void )
    {
      return vec2(-X, -Y);
    } /* End of 'operator -' function */

    /* Vector substraction operator -
     * ARGUMENTS:
     *   - vector to substract:
     *       const vec2 &V;
     * RETURNS: (vec2) result vector
     */
    vec2 operator - ( const vec2 &V )
    {
      return vec2(X - V.X, Y - V.Y);
    } /* End of 'operator -' function */

    /* Vector add & equal operator +=
     * ARGUMENTS:
     *   - vec2 to add & equal:
     *       const vec2 &V;
     * RETURNS: (vec2) this pointer;
     */
    vec2 operator += ( const vec2 &V )
    {
      this->X += V.X;
      this->Y += V.Y;

      return *this;
    } /* End of 'operator +=' function */

    /* Vector substract & equal operator +=
     * ARGUMENTS:
     *   - vec2 to substract & equal:
     *       const vec2 &V;
     * RETURNS: (vec2) this pointer;
     */
    vec2 operator -= ( const vec2 &V )
    {
      this->X -= V.X;
      this->Y -= V.Y;

      return *this;
    } /* End of 'operator -=' function */

    /* Multiply vectors by components operator *
     * ARGUMENTS:
     *   - vec2 to myltiply:
     *       const vec2 &V;
     * RETURNS: (vec2) result vector;
     */
    vec2 operator * ( const vec2 &V )
    {
      return vec2(X * V.X, Y * V.Y);
    } /* End of 'operator *' function  */

    /* Vector multiply & equal operator +=
     * ARGUMENTS:
     *   - vec2 to multiply & equal:
     *       const vec2 &V;
     * RETURNS: (vec2) this pointer;
     */
    vec2 operator *= ( const vec2 &V )
    {
      this->X *= V.X;
      this->Y *= V.Y;

      return *this;
    } /* End of 'operator *=' function */

    /* Multiply vector by number function
     * ARGUMENTS:
     *   - number to multiply:
     *       const Type N;
     * RETURNS: (vec2) result vector
     */
    vec2 operator * ( const Type N )
    {
      return vec2(X * N, Y * N);
    } /* End of 'operator *' function */

    /* Multiply & equal vector by number function.
     * ARGUMENTS:
     *   - number to multiply & equal:
     *       const Type N;
     * RETURNS: (vec2) this pointer
     */
    vec2 operator *= ( const Type N )
    {
      this->X *= N;
      this->Y *= N;

      return *this;
    } /* End of 'operator *=' function */

    /* Divide vector by num function.
     * ARGUMENTS:
     *   - number to divide:
     *       const Type N;
     * RETURNS: (vec2) result vector
     */
    vec2 operator / ( const Type N )
    {
      return vec2(X / N, Y / N);
    } /* End of 'operator /' function */

    /* Divide & equal vector by number function.
     * ARGUMENTS:
     *   - number to divide & equal:
     *       const Type N;
     * RETURNS: (vec2) this pointer
     */
    vec2 operator /= ( const Type N )
    {
      this->X /= N;
      this->Y /= N;

      return *this;
    } /* End of 'operator /=' function */

    /* Dot vectors product function.
     * ARGUMENTS:
     *   - vec2 to dot:
     *       const vec2 &V;
     * RETURNS: (Type) dot product.
     */
    Type operator & ( const vec2 &V )
    {
      return X * V.X + Y * V.Y;
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
     * RETURNS: (vec2) this normalizing vector
     */
    vec2 Normalizing( void )
    {
      return *this / !*this;
    } /* End of 'Normalizing' function */

    /* Sqr of vector length function.
     * ARGUMENTS: None.
     * RETURNS: (Type) this length ^ 2
     */
    Type Length2( void )
    {
      return X * X + Y * Y;
    } /* End of 'Length2' function */

    /* Get distance between this vector and given function
     * ARGUMENTS:
     *   - vector to get distance:
     *       const vec2 &P;
     * RETURNS: (double) distance
     */
    double Distance( const vec2 &P ) const
    {
      return sqrt((X - P.X) * (X - P.X) + (Y - P.Y) * (Y - P.Y));
    } /* End of 'Distance' function */

    /* Zero vector funciton.
     * ARGUMENTS: None.
     * RETURNS: (vec2) zero vector
     */
    vec2 Zero( void ) const
    {
      return vec2(0);
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
      default:
        if (i < 0)
          return this->X;
        else
          return this->Y;
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
      default:
        if (i < 0)
          return this->X;
        else
          return this->Y;
      }
    } /* End of 'operator [] function */

    /* vec2 --> Type * function
     * ARGUMENTS: None.
     * RETURNS: (Type *) pointer to this->X
     */
    operator Type * ( void )
    {
      return &this->X;
    } /* End of 'operator Type *' function */

    /* Ostream rewrite to output vec2.
     * ARGUMENTS:
     *   - ostream to output:
     *       std::ostream &C;
     *   - vector to output:
     *       vec2 &V;
     * RETURNS: (ostream &) result ostream
     */
    friend std::ostream & operator << ( std::ostream &C, vec2 &V )
    {
      C << "{";
      C << V.X << ", ";
      C << V.Y << "}";

      return C;
    } /* End of 'operator <<' function */
  }; /* End of 'vec2' class */

  typedef vec2<double> dvec2;            /* Float vector */
  typedef vec2<float> fvec2;            /* Double vector */
  typedef vec2<int> ivec2;            /* Integer vector */
} /* End of 'mth' namespace */

#endif /* __mth_vec2_h_*/

/* END OF 'mth_vec2.h' FILE */