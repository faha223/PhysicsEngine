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

	physx::PxSphereGeometry createSphereGeometry(physx::PxReal radius);

	physx::PxCapsuleGeometry createCapsuleGeometry(physx::PxReal radius, physx::PxReal halfHeight);

	physx::PxConvexMesh *createConvexMesh(physx::PxVec3 *pointCloud, physx::PxU32 numVertices);

	physx::PxConvexMeshGeometry createConvexMeshGeometry(physx::PxVec3 *pointCloud, physx::PxU32 numVertices);

	physx::PxRigidDynamic* addRigidDynamic(physx::PxVec3 position, physx::PxQuat orientation, physx::PxGeometry *components, physx::PxU32 numComponents, physx::PxReal Mass, physx::PxVec3 MomentOfInertia, physx::PxVec3 initialLinearVelocity, physx::PxVec3 initialAngularVelocity, Material mat, physx::PxReal linearDamping = 0.0f, physx::PxReal angularDamping = 0.0f);

	physx::PxRigidStatic* addRigidStatic(physx::PxVec3 position, physx::PxQuat orientation, physx::PxGeometry *components, physx::PxU32 numComponents, Material mat);

	void getActors(std::vector<physx::PxRigidActor*> &actors);

	void setGravity(vec3 gravity);

	void setFrequency(uint32_t frequency);

	static vec3 InertiaTensorSolidCube(physx::PxReal width, physx::PxReal mass);

	static vec3 InertiaTensorSolidSphere(physx::PxReal radius, physx::PxReal mass);
	
	static vec3 InertiaTensorHollowSphere(physx::PxReal radius, physx::PxReal mass);

	static vec3 InertiaTensorSolidCapsule(physx::PxReal radius, physx::PxReal halfHeight, physx::PxReal mass);

	physx::PxPhysics *PhysicsEngine::getPhysics();

	physx::PxCooking *PhysicsEngine::getCooking();

	~PhysicsEngine();
};

#endif