#ifndef _PHYS_ENG_H_
#define _PHYS_ENG_H_

#include "types.h"

#include <cstdio>
#include <vector>
#include <thread>
#include <mutex>
#include <PxPhysicsAPI.h>

#ifdef _WIN32
#pragma comment(lib, "x86\\PhysX3_x86.lib")
#pragma comment(lib, "x86\\PhysX3Common_x86.lib")
#pragma comment(lib, "x86\\PhysX3Extensions.lib")
#endif

class PhysicsEngine
{
private:
	std::mutex engineMutex;
	std::thread *updateThread;

	physx::PxPhysics* physics;
	physx::PxFoundation* foundation;
	physx::PxTolerancesScale tolScale;

	physx::PxMaterial *mtls[6];

	physx::PxScene *scene;						// Default Scene

	uint32_t engineFrequency;					// DEFAULT: 360 Hz
	physx::PxReal simulationPeriod;					// DEFAULT: 1/360 (updated whenever engineFrequency is updated)

	bool quit;									// DEFAULT: false

	static void updateLoop(PhysicsEngine *pe);	// The static function that calls the update method at regular intervals
	void update();

public:
	enum Material
	{
		Wood, SolidPVC, HollowPVC, SolidSteel, HollowSteel, Concrete
	};
	PhysicsEngine();

	physx::PxRigidActor* addCollisionSphere(vec3 position, real radius, real Mass = 1.0f, vec3 initialLinearVelocity = vec3(0.0f, 0.0f, 0.0f), vec3 initialAngularVelocity = vec3(0.0f, 0.0f, 0.0f), Material mat = Wood, bool isDynamic = true);		// Adds a collision Sphere to the scene, and returns a pointer to it so that the user can access its data

	void setGravity(vec3 gravity);

	void setFrequency(uint32_t frequency);

	~PhysicsEngine();
};

#endif