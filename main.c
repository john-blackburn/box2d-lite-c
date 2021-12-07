// compile to ASM:
// use cdecl calling convention (right to left on stack), callee can use EAX, ECX, EDX
// and return through EAX.
// but return everything through registers
// functions at worst return a Vec2 struct so pass back as EAX:EDX (or fp version)
// only exception is matmulMM but this is only called once so inline it
// (this is Microsoft cdecl for x86 32 bit)
// for z80 return Vec2 as HL:DE (16 bit fixed point)
// callee can use all registers

#include <float.h>

#include <stdio.h>
#include "b2d_lite.h"

int main()
{
    printf("start\n");
    
    iterations = 10;
	gravity.x=0.0f;
    gravity.y=-10.0f;
    
    struct Body ground;

    struct Vec2 size;
    size.x=100.f;
    size.y=20.0f;
    
    struct Vec2 pos;
    pos.x=0;
    pos.y=-0.5f * size.y;
    
    initBody(&ground, size, FLT_MAX);
    ground.position = pos;
    
    addBody(&ground);
    
    struct Body crate;

    size.x=1.0f;
    size.y=1.0f;

    pos.x=0.0f;
    pos.y=4.0f;

    initBody(&crate, size, 200.0f);
    crate.position = pos;

    addBody(&crate);
        
    float timeStep=1.0f/60.0f;
    
    for (int it=0; it<100; it++)
    {
        Step(timeStep);
        printf("%f %f\n", crate.position.x, crate.position.y);
    }
}
