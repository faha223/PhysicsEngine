#ifndef _PHYSICS_ENTITY_
#define _PHYSICS_ENTITY_

#include "vec3.h"
#include "quaternion.h"

class PhysicsEntity
{
public:
	real invMass;
	real charge;
	real coeffOfRestitution;
	vec3 position;
	quaternion rotation;
	vec3 velocity;
	vec3 angularVelocity;
	vec3 acceleration;
	vec3 angularAcceleration;
	vec3 netForce;
	virtual void ResolveCollision(PhysicsEntity &pe);
	void integrate(const real &timeElapsed);
	PhysicsEntity();
	virtual ~PhysicsEntity();
};

#endif