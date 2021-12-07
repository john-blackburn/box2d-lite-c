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


#ifndef B2D_LITE_H
#define B2D_LITE_H

#include "collide.h"

void initBody(struct Body* b, struct Vec2 w, float m);
void Step(float dt);
void addBody(struct Body *b);
struct Body* getBody(int i);

extern struct Vec2 gravity;
extern int iterations;

#endif
