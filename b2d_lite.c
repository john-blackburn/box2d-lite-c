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

#include <stdio.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>

#include "mathutils.h"
#include "collide.h"

struct Arbiter
{
	struct Contact contacts[2];
	int numContacts;

	struct Body* body1;
	struct Body* body2;

	// Combined friction
	float friction;
};

struct ArbiterKey
{
	struct Body* body1;
	struct Body* body2;
};

// was in World

struct Body *g_bodies[100];
int g_numBodies;

struct Arbiter g_arbiters[100];
struct ArbiterKey g_arbiterKeys[100];
int g_numArbiters;

struct Vec2 gravity;
int iterations;
int accumulateImpulses = 1;
int warmStarting = 1;
int positionCorrection = 1;

struct Arbiter* findArbiter(struct ArbiterKey key)
{
    for (int i=0; i<g_numArbiters; i++)
    {    
        if (g_arbiterKeys[i].body1 == key.body1 && 
            g_arbiterKeys[i].body2 == key.body2)
           return &g_arbiters[i];
    }

    return NULL;
}

void insertArbiter(struct ArbiterKey key, struct Arbiter value)
{
    if (g_numArbiters >= 100)    
    {
        printf("Exceeded max number of arbiters!\n");
        exit(1);
    }
    
    g_arbiterKeys[g_numArbiters]=key;
    g_arbiters[g_numArbiters]=value;
    g_numArbiters++;
}

void eraseArbiter(struct ArbiterKey key)
{
    for (int i=0; i < g_numArbiters; i++)
    {    
        if (g_arbiterKeys[i].body1 == key.body1 && 
            g_arbiterKeys[i].body2 == key.body2)
        {
            for (int j=i; j < g_numArbiters-1; j++)
            {
                g_arbiterKeys[j]=g_arbiterKeys[j+1];
                g_arbiters[j]=g_arbiters[j+1];
            }
            g_numArbiters--;
        }
    }                                        
}

void initContact(struct Contact *c)
{
    c->Pn=0.0f;
    c->Pt=0.0f;
    c->Pnb=0.0f;
}

void UpdateArbiter(struct Arbiter *a, struct Contact* newContacts, int numNewContacts)
{
	struct Contact mergedContacts[2];
    initContact(&mergedContacts[0]);
    initContact(&mergedContacts[1]);

	for (int i = 0; i < numNewContacts; ++i)
	{
		struct Contact* cNew = newContacts + i;
		int k = -1;
		for (int j = 0; j < a->numContacts; ++j)
		{
			struct Contact* cOld = a->contacts + j;
			if (cNew->feature.value == cOld->feature.value)
			{
				k = j;
				break;
			}
		}

		if (k > -1)
		{
			struct Contact* c = mergedContacts + i;
			struct Contact* cOld = a->contacts + k;
			*c = *cNew;
			if (warmStarting)
			{
				c->Pn = cOld->Pn;
				c->Pt = cOld->Pt;
				c->Pnb = cOld->Pnb;
			}
			else
			{
				c->Pn = 0.0f;
				c->Pt = 0.0f;
				c->Pnb = 0.0f;
			}
		}
		else
		{
			mergedContacts[i] = newContacts[i];
		}
	}

	for (int i = 0; i < numNewContacts; ++i)
		a->contacts[i] = mergedContacts[i];

	a->numContacts = numNewContacts;
}


void BroadPhase()
{
    for (int i=0; i< g_numBodies; i++)
    {
        struct Body* bi = g_bodies[i];
        
        for (int j=i+1; j < g_numBodies; j++)
        {
            struct Body* bj = g_bodies[j];

            if (bi->invMass == 0.0f && bj->invMass == 0.0f)
                continue;

            struct Arbiter newArb;
            struct ArbiterKey key;

            if (bi<bj)
            {
                newArb.body1=bi;
                newArb.body2=bj;
                key.body1=bi;
                key.body2=bj;    
            }
            else
            {
                newArb.body1=bj;
                newArb.body2=bi;
                key.body1=bj;
                key.body2=bi;
            }

            initContact(&newArb.contacts[0]);
            initContact(&newArb.contacts[1]);
            
            newArb.numContacts = Collide(newArb.contacts, newArb.body1, newArb.body2);  
            newArb.friction = sqrtf(newArb.body1->friction * newArb.body2->friction);
            
            if (newArb.numContacts > 0)
            {
                struct Arbiter *a = findArbiter(key);
                if (a == NULL)
                {
                    insertArbiter(key, newArb);
                }
                else
                {
                    UpdateArbiter(a, newArb.contacts, newArb.numContacts);
                }
            }
            else
            {
                eraseArbiter(key);
            }
        }
    }      
}

void PreStep(struct Arbiter *a, float inv_dt)
{
	const float k_allowedPenetration = 0.01f;
	float k_biasFactor = positionCorrection ? 0.2f : 0.0f;

	for (int i = 0; i < a->numContacts; ++i)
	{
		struct Contact* c = a->contacts + i;

		struct Vec2 r1 = diffVec2(c->position, a->body1->position);
		struct Vec2 r2 = diffVec2(c->position, a->body2->position);

		// Precompute normal mass, tangent mass, and bias.
		float rn1 = Dot(r1, c->normal);
		float rn2 = Dot(r2, c->normal);
		float kNormal = a->body1->invMass + a->body2->invMass;
		kNormal += a->body1->invI * (Dot(r1, r1) - rn1 * rn1) + a->body2->invI * (Dot(r2, r2) - rn2 * rn2);
		c->massNormal = 1.0f / kNormal;

		struct Vec2 tangent = Crossvs(c->normal, 1.0f);
		float rt1 = Dot(r1, tangent);
		float rt2 = Dot(r2, tangent);
		float kTangent = a->body1->invMass + a->body2->invMass;
		kTangent += a->body1->invI * (Dot(r1, r1) - rt1 * rt1) + a->body2->invI * (Dot(r2, r2) - rt2 * rt2);
		c->massTangent = 1.0f /  kTangent;

		c->bias = -k_biasFactor * inv_dt * Min(0.0f, c->separation + k_allowedPenetration);

		if (accumulateImpulses)
		{
			// Apply normal + friction impulse
			struct Vec2 P = sumVec2(scaledVec2(c->Pn, c->normal), scaledVec2(c->Pt, tangent));

			subfromVec2(&a->body1->velocity, scaledVec2(a->body1->invMass, P));
			a->body1->angularVelocity -= a->body1->invI * Crossvv(r1, P);

			addtoVec2(&a->body2->velocity, scaledVec2(a->body2->invMass, P));
			a->body2->angularVelocity += a->body2->invI * Crossvv(r2, P);
		}
	}
}

void ApplyImpulse(struct Arbiter *a)
{
	struct Body* b1 = a->body1;
	struct Body* b2 = a->body2;

	for (int i = 0; i < a->numContacts; ++i)
	{
		struct Contact* c = a->contacts + i;
		c->r1 = diffVec2(c->position, b1->position);
		c->r2 = diffVec2(c->position, b2->position);

		// Relative velocity at contact
		struct Vec2 dv = diffVec2(sumVec2(b2->velocity, Crosssv(b2->angularVelocity, c->r2)),
                                  sumVec2(b1->velocity, Crosssv(b1->angularVelocity, c->r1)));

		// Compute normal impulse
		float vn = Dot(dv, c->normal);

		float dPn = c->massNormal * (-vn + c->bias);

		if (accumulateImpulses)
		{
			// Clamp the accumulated impulse
			float Pn0 = c->Pn;
			c->Pn = Max(Pn0 + dPn, 0.0f);
			dPn = c->Pn - Pn0;
		}
		else
		{
			dPn = Max(dPn, 0.0f);
		}

		// Apply contact impulse
		struct Vec2 Pn = scaledVec2(dPn, c->normal);

        subfromVec2(&b1->velocity, scaledVec2(b1->invMass, Pn));
		b1->angularVelocity -= b1->invI * Crossvv(c->r1, Pn);

        addtoVec2(&b2->velocity, scaledVec2(b2->invMass, Pn));
		b2->angularVelocity += b2->invI * Crossvv(c->r2, Pn);

		// Relative velocity at contact
		dv = diffVec2(sumVec2(b2->velocity, Crosssv(b2->angularVelocity, c->r2)),
                      sumVec2(b1->velocity, Crosssv(b1->angularVelocity, c->r1)));

		struct Vec2 tangent = Crossvs(c->normal, 1.0f);
		float vt = Dot(dv, tangent);
		float dPt = c->massTangent * (-vt);

		if (accumulateImpulses)
		{
			// Compute friction impulse
			float maxPt = a->friction * c->Pn;

			// Clamp friction
			float oldTangentImpulse = c->Pt;
			c->Pt = Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
			dPt = c->Pt - oldTangentImpulse;
		}
		else
		{
			float maxPt = a->friction * dPn;
			dPt = Clamp(dPt, -maxPt, maxPt);
		}

		// Apply contact impulse
		struct Vec2 Pt = scaledVec2(dPt, tangent);

		subfromVec2(&b1->velocity, scaledVec2(b1->invMass, Pt));
		b1->angularVelocity -= b1->invI * Crossvv(c->r1, Pt);

		addtoVec2(&b2->velocity, scaledVec2(b2->invMass, Pt));
		b2->angularVelocity += b2->invI * Crossvv(c->r2, Pt);
	}
}

void Step(float dt)
{
	float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

	// Determine overlapping bodies and update contact points.
	BroadPhase();

	// Integrate forces.
	for (int i = 0; i < g_numBodies; ++i)
	{
		struct Body* b = g_bodies[i];

		if (b->invMass == 0.0f)
			continue;

		addtoVec2(&b->velocity, scaledVec2(dt, sumVec2(gravity, scaledVec2(b->invMass, b->force))));
		b->angularVelocity += dt * b->invI * b->torque;
	}

	// Perform pre-steps.
    
    for (int i=0; i < g_numArbiters; i++)
    {
        PreStep(&g_arbiters[i], inv_dt);
    }

	// Perform iterations
	for (int it = 0; it < iterations; ++it)
	{
        for (int i=0; i < g_numArbiters; i++)
            ApplyImpulse(&g_arbiters[i]);
	}

	// Integrate Velocities
	for (int i = 0; i < g_numBodies; ++i)
	{
		struct Body* b = g_bodies[i];

		addtoVec2(&b->position, scaledVec2(dt, b->velocity));
		b->rotation += dt * b->angularVelocity;

		b->force.x=0;
		b->force.y=0;
		b->torque = 0.0f;
	}
}

void initBody(struct Body* b, struct Vec2 w, float m)
{
        
    b->position.x=0;
    b->position.y=0;
    b->rotation=0;
    b->velocity.x=0;
    b->velocity.y=0;
    b->angularVelocity=0;
    b->force.x=0;
    b->force.y=0;
    b->torque=0.0f;
    b->friction=0.2f;

    b->width = w;
    b->mass = m;

	if (b->mass < FLT_MAX)
	{
		b->invMass = 1.0f / b->mass;
		b->I = b->mass * (b->width.x * b->width.x + b->width.y * b->width.y) / 12.0f;
		b->invI = 1.0f / b->I;
	}
	else
	{
		b->invMass = 0.0f;
		b->I = FLT_MAX;
		b->invI = 0.0f;
	}
}    

void addBody(struct Body *b)
{
    g_bodies[g_numBodies]=b;
    g_numBodies++;
}

struct Body* getBody(int i)
{
    return g_bodies[i];
}
