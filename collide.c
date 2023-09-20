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


// Box vertex and edge numbering:
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  --> x
//    |        |
//   v3 ------ v4
//        e3

#include <stdio.h>

#include "mathutils.h"
#include "collide.h"

int debug;

enum Axis
{
	FACE_A_X,
	FACE_A_Y,
	FACE_B_X,
	FACE_B_Y
};

enum EdgeNumbers
{
	NO_EDGE = 0,
	EDGE1,
	EDGE2,
	EDGE3,
	EDGE4
};

struct ClipVertex
{
//	ClipVertex() { fp.value = 0; }
	struct Vec2 v;
	struct Edges fp;
};

// z80: IX=CV
void initClipVertex(struct ClipVertex *cv)
{
    cv->fp.inEdge1 = 0;
    cv->fp.outEdge1 = 0;
    cv->fp.inEdge2 = 0;
    cv->fp.outEdge2 = 0;
}

// swap two chars provided
// Z80: IX=a, IY=b
void Swap(char* a, char* b)
{
	char tmp = *a;
	*a = *b;
	*b = tmp;
}

// flip a feature pair in situ
// Z80: IX=fp
// { char inEdge1, char outEdge1, char inEdge2, char outEdge2 }
// {1,2,3,4} -> {3, 4, 1, 2}
void Flip(struct Edges *fp)
{
	Swap(&(fp->inEdge1), &(fp->inEdge2));
	Swap(&(fp->outEdge1), &(fp->outEdge2));
}

// set vOut. Return number of output points (0, 1 or 2)
// z80: IX=vOut, IY=vIn
int ClipSegmentToLine(struct ClipVertex vOut[2], struct ClipVertex vIn[2],
					  struct Vec2 normal, float offset, char clipEdge)
{
	// Start with no output points
	int numOut = 0;

	// Calculate the distance of end points to the line
	float distance0 = Dot(normal, vIn[0].v) - offset;
	float distance1 = Dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		// Find intersection point of edge and plane
		float interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = sumVec2(vIn[0].v, scaledVec2(interp , diffVec2(vIn[1].v, vIn[0].v)));
		if (distance0 > 0.0f)
		{
			vOut[numOut].fp = vIn[0].fp;
			vOut[numOut].fp.inEdge1 = clipEdge;
			vOut[numOut].fp.inEdge2 = NO_EDGE;
		}
		else
		{
			vOut[numOut].fp = vIn[1].fp;
			vOut[numOut].fp.outEdge1 = clipEdge;
			vOut[numOut].fp.outEdge2 = NO_EDGE;
		}
		++numOut;
	}

	return numOut;
}

// set c
// Z80: IX = c
static void ComputeIncidentEdge(struct ClipVertex c[2], struct Vec2 h, struct Vec2 pos,
								struct Mat22 Rot, struct Vec2 normal)
{
    
    if (debug)
    {
        printf("ComputeIncidentEdge:\n");
        printf("%13.5e %13.5e %13.5e %13.5e\n", h.x, h.y, pos.x, pos.y);
        printf("%13.5e %13.5e\n", Rot.col1.x, Rot.col1.y);
        printf("%13.5e %13.5e\n", Rot.col2.x, Rot.col2.y);
        printf("%13.5e %13.5e\n", normal.x, normal.y);   
    }
    
	// The normal is from the reference box. Convert it
	// to the incident boxe's frame and flip sign.
	struct Mat22 RotT = Transpose(Rot);
	struct Vec2 n = minusVec2(matmul(RotT, normal));
   	struct Vec2 nAbs = Absv(n);

	if (nAbs.x > nAbs.y)
	{
		if (n.x > 0.0f)
		{

			c[0].v.x = h.x;
			c[0].v.y = -h.y;
			c[0].fp.inEdge2 = EDGE3;
			c[0].fp.outEdge2 = EDGE4;

			c[1].v.x = h.x;
			c[1].v.y =h.y;
			c[1].fp.inEdge2 = EDGE4;
			c[1].fp.outEdge2 = EDGE1;
		}
		else
		{
			c[0].v.x = -h.x;
            c[0].v.y = h.y;
			c[0].fp.inEdge2 = EDGE1;
			c[0].fp.outEdge2 = EDGE2;

			c[1].v.x = -h.x;
			c[1].v.y = -h.y;
			c[1].fp.inEdge2 = EDGE2;
			c[1].fp.outEdge2 = EDGE3;
		}
	}
	else
	{
		if (n.y > 0.0f)
		{
			c[0].v.x = h.x;
			c[0].v.y = h.y;
			c[0].fp.inEdge2 = EDGE4;
			c[0].fp.outEdge2 = EDGE1;

			c[1].v.x = -h.x;
			c[1].v.y = h.y;
			c[1].fp.inEdge2 = EDGE1;
			c[1].fp.outEdge2 = EDGE2;
		}
		else
		{
			c[0].v.x = -h.x;
			c[0].v.y = -h.y;
			c[0].fp.inEdge2 = EDGE2;
			c[0].fp.outEdge2 = EDGE3;

			c[1].v.x = h.x;
			c[1].v.y = -h.y;
			c[1].fp.inEdge2 = EDGE3;
			c[1].fp.outEdge2 = EDGE4;
		}
	}

	c[0].v = sumVec2(pos, matmul(Rot, c[0].v));
	c[1].v = sumVec2(pos, matmul(Rot, c[1].v));
    
    if (debug)
    {
        for (int i=0; i<2; i++)
        {
            printf("%13.5e %13.5e\n", c[i].v.x, c[i].v.y);
            printf("%d %d %d %d\n", c[i].fp.inEdge1, c[i].fp.outEdge1, c[i].fp.inEdge2, c[i].fp.outEdge2);
        }
    }

}

void writeBody(struct Body* body)
{
	printf("%13.5e %13.5e %13.5e\n", body->position.x, body->position.y, body->rotation);	
	printf("%13.5e %13.5e %13.5e\n", body->velocity.x, body->velocity.y, body->angularVelocity);
	
    printf("%13.5e %13.5e\n", body->width.x, body->width.y);
	
	printf("%13.5e %13.5e %13.5e %13.5e %13.5e\n", body->friction, body->mass, body->invMass, body->I, body->invI);
}

// The normal points from A to B
// Set the contacts provided based on supplied bodies. return number of contacts (might be zero)
// in z80 contacts = HL, bodyA = IX, bodyB = IY
int Collide(struct Contact* contacts, struct Body* bodyA, struct Body* bodyB)
{
	if (debug)
	{
        printf("bodyA\n");
		writeBody(bodyA);
        
        printf("bodyB\n");
		writeBody(bodyB);
	}
	
	// Setup
	struct Vec2 hA = scaledVec2(0.5f, bodyA->width);
	struct Vec2 hB = scaledVec2(0.5f, bodyB->width);

	struct Vec2 posA = bodyA->position;
	struct Vec2 posB = bodyB->position;

	struct Mat22 RotA, RotB;
    initMat22(&RotA, bodyA->rotation); initMat22(&RotB, bodyB->rotation);

	struct Mat22 RotAT = Transpose(RotA);
	struct Mat22 RotBT = Transpose(RotB);

	struct Vec2 a1 = RotA.col1, a2 = RotA.col2;
	struct Vec2 b1 = RotB.col1, b2 = RotB.col2;

	struct Vec2 dp = diffVec2(posB, posA);
	struct Vec2 dA = matmul(RotAT, dp);
	struct Vec2 dB = matmul(RotBT, dp);

	struct Mat22 C = matmulMM(RotAT, RotB);
	struct Mat22 absC = AbsM(C);
	struct Mat22 absCT = Transpose(absC);

	// Box A faces
	struct Vec2 faceA = diffVec2(Absv(dA), hA);
    subfromVec2(&faceA, matmul(absC, hB));
    
	if (faceA.x > 0.0f || faceA.y > 0.0f)
		return 0;

	// Box B faces
//	Vec2 faceB = Absv(dB) - absCT * hA - hB;

	struct Vec2 faceB = diffVec2(Absv(dB), hB);
    subfromVec2(&faceB, matmul(absCT, hA));
    
	if (faceB.x > 0.0f || faceB.y > 0.0f)
		return 0;

	// Find best axis
	enum Axis axis;
	float separation;
	struct Vec2 normal;

	// Box A faces
	axis = FACE_A_X;
	separation = faceA.x;
	normal = dA.x > 0.0f ? RotA.col1 : minusVec2(RotA.col1);

	const float relativeTol = 0.95f;
	const float absoluteTol = 0.01f;

	if (faceA.y > relativeTol * separation + absoluteTol * hA.y)
	{
		axis = FACE_A_Y;
		separation = faceA.y;
		normal = dA.y > 0.0f ? RotA.col2 : minusVec2(RotA.col2);
	}

	// Box B faces
	if (faceB.x > relativeTol * separation + absoluteTol * hB.x)
	{
		axis = FACE_B_X;
		separation = faceB.x;
		normal = dB.x > 0.0f ? RotB.col1 : minusVec2(RotB.col1);
	}

	if (faceB.y > relativeTol * separation + absoluteTol * hB.y)
	{
		axis = FACE_B_Y;
		separation = faceB.y;
		normal = dB.y > 0.0f ? RotB.col2 : minusVec2(RotB.col2);
	}

	// Setup clipping plane data based on the separating axis
	struct Vec2 frontNormal, sideNormal;
	struct ClipVertex incidentEdge[2];
    initClipVertex(&incidentEdge[0]);
    initClipVertex(&incidentEdge[1]);

	float front, negSide, posSide;
	char negEdge, posEdge;

	// Compute the clipping lines and the line segment to be clipped.
	switch (axis)
	{
	case FACE_A_X:
		{
			frontNormal = normal;
			front = Dot(posA, frontNormal) + hA.x;
			sideNormal = RotA.col2;
			float side = Dot(posA, sideNormal);
			negSide = -side + hA.y;
			posSide =  side + hA.y;
			negEdge = EDGE3;
			posEdge = EDGE1;
			ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
		}
		break;

	case FACE_A_Y:
		{
			frontNormal = normal;
			front = Dot(posA, frontNormal) + hA.y;
			sideNormal = RotA.col1;
			float side = Dot(posA, sideNormal);
			negSide = -side + hA.x;
			posSide =  side + hA.x;
			negEdge = EDGE2;
			posEdge = EDGE4;
			ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
		}
		break;

	case FACE_B_X:
		{
			frontNormal = minusVec2(normal);
			front = Dot(posB, frontNormal) + hB.x;
			sideNormal = RotB.col2;
			float side = Dot(posB, sideNormal);
			negSide = -side + hB.y;
			posSide =  side + hB.y;
			negEdge = EDGE3;
			posEdge = EDGE1;
			ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
		}
		break;

	case FACE_B_Y:
		{
			frontNormal = minusVec2(normal);
			front = Dot(posB, frontNormal) + hB.y;
			sideNormal = RotB.col1;
			float side = Dot(posB, sideNormal);
			negSide = -side + hB.x;
			posSide =  side + hB.x;
			negEdge = EDGE2;
			posEdge = EDGE4;
			ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
		}
		break;
	}

	// clip other face with 5 box planes (1 face plane, 4 edge planes)

	struct ClipVertex clipPoints1[2];
	struct ClipVertex clipPoints2[2];
    
    initClipVertex(&clipPoints1[0]);
    initClipVertex(&clipPoints1[1]);
    initClipVertex(&clipPoints2[0]);
    initClipVertex(&clipPoints2[1]);
    
	int np;

	// Clip to box side 1
	np = ClipSegmentToLine(clipPoints1, incidentEdge, minusVec2(sideNormal), negSide, negEdge);

	if (np < 2)
		return 0;

	// Clip to negative box side 1
	np = ClipSegmentToLine(clipPoints2, clipPoints1,  sideNormal, posSide, posEdge);

	if (np < 2)
		return 0;

	// Now clipPoints2 contains the clipping points.
	// Due to roundoff, it is possible that clipping removes all points.

	int numContacts = 0;
	for (int i = 0; i < 2; ++i)
	{
		float separation = Dot(frontNormal, clipPoints2[i].v) - front;

		if (separation <= 0)
		{
			contacts[numContacts].separation = separation;
			contacts[numContacts].normal = normal;
			// slide contact point onto reference face (easy to cull)
			contacts[numContacts].position = diffVec2(clipPoints2[i].v, scaledVec2(separation, frontNormal));
			contacts[numContacts].feature = clipPoints2[i].fp;
			if (axis == FACE_B_X || axis == FACE_B_Y)
				Flip(&contacts[numContacts].feature);
			++numContacts;
		}
	}

    if (debug)
	{
        printf("numContacts=%d\n",numContacts);
		for (int i=0; i<numContacts; i++)
		{
            struct Contact con=contacts[i];
            printf("i=%d\n",i);
			printf("%13.5e %13.5e\n", con.position.x, con.position.y);
			printf("%13.5e %13.5e\n", con.normal.x, con.normal.y);
	        printf("%13.5e %13.5e\n", con.r1.x, con.r1.y);
			printf("%13.5e %13.5e\n", con.r2.x, con.r2.y);
            printf("%13.5e\n", con.separation);
			printf("%13.5e %13.5e %13.5e\n", con.Pn, con.Pt, con.Pnb);
			printf("%13.5e %13.5e\n", con.massNormal, con.massTangent);
			printf("%13.5e\n", con.bias);
			printf("%d %d %d %d\n", con.feature.inEdge1, con.feature.outEdge1, con.feature.inEdge2, con.feature.outEdge2);
		}
	}

	return numContacts;
}