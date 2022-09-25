/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
*/

#include <math.h>
#include <float.h>
#include <assert.h>
#include <stdlib.h>

#include "mathutils.h"

// Return transpose of given matrix
struct Mat22 Transpose(struct Mat22 mat)
{
    struct Vec2 v1;
    v1.x=mat.col1.x;
    v1.y=mat.col2.x;

    struct Vec2 v2;
    v2.x=mat.col1.y;
    v2.y=mat.col2.y;
    
    struct Mat22 res;
    res.col1=v1;
    res.col2=v2;
    
    return res;
    
//    return Mat22(Vec2(col1.x, col2.x), Vec2(col1.y, col2.y));
}

// Initialise matrix to rotation matrix based on angle
void initMat22(struct Mat22 *mat, float angle)
{
    
    float c = cosf(angle), s = sinf(angle);
    mat->col1.x = c; mat->col2.x = -s;
    mat->col1.y = s; mat->col2.y = c;
}

// Return product of 2 matrix and vector Av
struct Vec2 matmul(struct Mat22 A, struct Vec2 v)
{
	struct Vec2 res;
    res.x = A.col1.x * v.x + A.col2.x * v.y;
    res.y = A.col1.y * v.x + A.col2.y * v.y;
    
    return res;
}

// Return product of matrix and matrix AB
struct Mat22 matmulMM(struct Mat22 A, struct Mat22 B)
{
    struct Mat22 res;
    res.col1 = matmul(A, B.col1);
    res.col2 = matmul(A, B.col2);
    
	return res;
}

// Add to vec in situ
void addtoVec2(struct Vec2 *vec, struct Vec2 other)
{
    vec->x += other.x;
    vec->y += other.y;        
}

// subtract from vec in situ
void subfromVec2(struct Vec2 *vec, struct Vec2 other)
{
    vec->x -= other.x;
    vec->y -= other.y;        
}

// scale vec in situ
void scaleVec2(struct Vec2 *vec, float a)
{
    vec->x *= a;
    vec->y *= a;        
}

// return scaled Vec2
struct Vec2 scaledVec2(float s, struct Vec2 v)
{
    struct Vec2 res;
    res.x = s * v.x;
    res.y = s * v.y;
    
	return res;
}

// return negated Vec2
struct Vec2 minusVec2(struct Vec2 v)
{
    struct Vec2 res;
    res.x = -v.x;
    res.y = -v.y;
    
	return res;
}

// return sum of Vec2s a+b
struct Vec2 sumVec2(struct Vec2 a, struct Vec2 b)
{
    struct Vec2 res;
    res.x = a.x + b.x;
    res.y = a.y + b.y;
    
	return res;
}

// return difference of Vec2s a-b
struct Vec2 diffVec2(struct Vec2 a, struct Vec2 b)
{
    struct Vec2 res;
    res.x = a.x - b.x;
    res.y = a.y - b.y;
    
	return res;
}

// return length of vec
float lengthVec2(struct Vec2 vec)
{
    return sqrtf(vec.x * vec.x + vec.y * vec.y);
}

// return dot product of two vecs
float Dot(struct Vec2 a, struct Vec2 b)
{
	return a.x * b.x + a.y * b.y;
}

// return (z comp) cross product of two vec2s
float Crossvv(struct Vec2 a, struct Vec2 b)
{
	return a.x * b.y - a.y * b.x;
}

// return vector scalar cross product
struct Vec2 Crossvs(struct Vec2 a, float s)
{
    struct Vec2 res;
    res.x=s * a.y;
    res.y=-s * a.x;
	return res;
}

// return scalar vector cross product
struct Vec2 Crosssv(float s, struct Vec2 a)
{
    struct Vec2 res;
    res.x=-s * a.y;
    res.y=s * a.x;
	return res;
}

// Return absolute value of scalar
float Abss(float a)
{
	return a > 0.0f ? a : -a;
}

// abs of vector
struct Vec2 Absv(struct Vec2 a)
{
    struct Vec2 res;
    res.x=fabsf(a.x);
    res.y=fabsf(a.y);
	return res;
}

// abs of matrix
struct Mat22 AbsM(struct Mat22 A)
{
    struct Mat22 res;
	res.col1 = Absv(A.col1), 
    res.col2 = Absv(A.col2);
    return res;
}


// return +1 if x>0 else -1
float Sign(float x)
{
	return x < 0.0f ? -1.0f : 1.0f;
}

// min of two scalars
float Min(float a, float b)
{
	return a < b ? a : b;
}

// max of two scalars
float Max(float a, float b)
{
	return a > b ? a : b;
}

// a clamped in [low,high]
float Clamp(float a, float low, float high)
{
	return Max(low, Min(a, high));
}

// random number [-1,1]
float Random()
{
	float r = (float)rand();
	r /= RAND_MAX;
	r = 2.0f * r - 1.0f;
	return r;
}

// random number in [lo,hi]
float RandomRange(float lo, float hi)
{
	float r = (float)rand();
	r /= RAND_MAX;
	r = (hi - lo) * r + lo;
	return r;
}
