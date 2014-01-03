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
#pragma comment(lib, "x86\\PhysX3Cooking_x86.lib")
#endif

class PhysicsEngine
{
private:
	std::mutex engineMutex;
	std::thread *updateThread;

	physx::PxPhysics* physics;
	physx::PxCooking *cooking;
	physx::PxFoundation* foundation;
	physx::PxTolerancesScale tolScale;

	physx::PxMaterial *mtls[6];

	physx::PxScene *scene;						// Default Scene

	uint32_t engineFrequency;					// DEFAULT: 360 Hz
	physx::PxReal simulationPeriod;				// DEFAULT: 1/360 (updated whenever engineFrequency is updated)

	bool quit;									// DEFAULT: false

	static void updateLoop(PhysicsEngine *pe);	// The static function that calls the update method at regular intervals
	void update();

public:
	enum Material
	{
		Wood, SolidPVC, HollowPVC, SolidSteel, HollowSteel, Concrete
	};
	PhysicsEngine();

	physx::PxRigidActor* addCollisionSphere(vec3 position, quaternion orientation, real radius, real Mass = 1.0f, vec3 massSpaceInertiaTensor = vec3(1.0f, 1.0f, 1.0f), vec3 initialLinearVelocity = vec3(0.0f, 0.0f, 0.0f), vec3 initialAngularVelocity = vec3(0.0f, 0.0f, 0.0f), Material mat = Wood, bool isDynamic = true);		// Adds a collision Sphere to the scene, and returns a pointer to it so that the user can access its data
	
	physx::PxRigidActor* addCollisionCapsule(vec3 position, quaternion orientation, real halfHeight = 1.0f, real radius = 1.0f, real Mass = 1.0f, vec3 massSpaceInertiaTensor = vec3(1.0f, 1.0f, 1.0f), vec3 initialLinearVelocity = vec3(0.0f, 0.0f, 0.0f), vec3 initialAngularVelocity = vec3(0.0f, 0.0f, 0.0f), Material mat = Wood, bool isDynamic = true);
	
	// Make sure all vertices are relative to the center of mass or else there will be odd behavior
	physx::PxRigidActor* addCollisionMesh(vec3 position, quaternion orientation, vec3 *vertices, uint32_t numVertices, real mass = 1.0f, vec3 massSpaceInertiaTensor = vec3(1.0f, 1.0f, 1.0f), vec3 initialLinearVelocity = vec3(0.0f, 0.0f, 0.0f), vec3 initialAngularVelocity = vec3(0.0f, 0.0f, 0.0f), Material mat = Wood, bool isDynamic = true);

	void getActors(std::vector<physx::PxRigidActor*> &actors);

	void setGravity(vec3 gravity);

	void setFrequency(uint32_t frequency);

	static vec3 InertiaTensorSolidCube(physx::PxReal width, physx::PxReal mass);

	static vec3 InertiaTensorSolidSphere(physx::PxReal radius, physx::PxReal mass);
	
	static vec3 InertiaTensorHollowSphere(physx::PxReal radius, physx::PxReal mass);

	static vec3 InertiaTensorSolidCapsule(physx::PxReal radius, physx::PxReal halfHeight, physx::PxReal mass);

	~PhysicsEngine();
};

#endif