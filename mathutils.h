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


#ifndef MATHUTILS_H
#define MATHUTILS_H

struct Vec2
{
	float x, y;
};

struct Mat22
{
    struct Vec2 col1, col2;
};

struct Mat22 Transpose(struct Mat22 mat);
void initMat22(struct Mat22 *mat, float angle);
struct Vec2 matmul(struct Mat22 A, struct Vec2 v);
struct Mat22 matmulMM(struct Mat22 A, struct Mat22 B);

void addtoVec2(struct Vec2 *vec, struct Vec2 other);
void subfromVec2(struct Vec2 *vec, struct Vec2 other);
void scaleVec2(struct Vec2 *vec, float a);
struct Vec2 scaledVec2(float s, struct Vec2 v);
struct Vec2 minusVec2(struct Vec2 v);
struct Vec2 sumVec2(struct Vec2 a, struct Vec2 b);
struct Vec2 diffVec2(struct Vec2 a, struct Vec2 b);

float Dot(struct Vec2 a, struct Vec2 b);

float Crossvv(struct Vec2 a, struct Vec2 b);
struct Vec2 Crossvs(struct Vec2 a, float s);
struct Vec2 Crosssv(float s, struct Vec2 a);

struct Vec2 Absv(struct Vec2 a);
struct Mat22 AbsM(struct Mat22 A);

float Min(float a, float b);
float Max(float a, float b);
float Clamp(float a, float low, float high);
float Random();
float RandomRange(float lo, float hi);

#endif
