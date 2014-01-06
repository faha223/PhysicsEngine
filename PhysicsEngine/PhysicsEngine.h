#ifndef _PHYS_ENG_H_
#define _PHYS_ENG_H_

#include "types.h"

#include <cstdio>
#include <vector>
#include <thread>
#include <mutex>
#include <PxPhysicsAPI.h>
#include <atomic>

#ifdef _WIN32
#pragma comment(lib, "x86\\PhysX3_x86.lib")
#pragma comment(lib, "x86\\PhysX3Common_x86.lib")
#pragma comment(lib, "x86\\PhysX3Extensions.lib")
#pragma comment(lib, "x86\\PhysX3Cooking_x86.lib")
#endif

class PhysicsEngine
{
private:
	// Multithreading support
	std::mutex engineMutex;
	std::thread *updateThread;

	struct PxRigidAerodynamic
	{
		physx::PxRigidDynamic *actor;
		physx::PxReal LiftCoefficient;
		physx::PxReal DragCoefficient;
		void ApplyLiftAndDrag();
	};

	// PhysX classes necessary for interacting with the engine
	physx::PxPhysics* physics;
	physx::PxCooking *cooking;
	physx::PxFoundation* foundation;
	physx::PxTolerancesScale tolScale;
	physx::PxMaterial *mtls[6];
	physx::PxScene *scene;						// Default Scene

	// The frequency at which the engine is running
	uint32_t engineFrequency;					// DEFAULT: 360 Hz
	// The simulation period (calculated from the engine frequency)
	physx::PxReal simulationPeriod;				// DEFAULT: 1/360 s

	// Tells the simulation loop to quit
	std::atomic_int32_t quit;									// DEFAULT: false

	// A list to keep track of all aerodynamic actors
	std::vector<PxRigidAerodynamic> aeroActors;

	static void updateLoop(PhysicsEngine *pe);	// The static function that calls the update method at regular intervals
	void update();

public:
	// The materials currently allocated in the engine
	enum Material
	{
		Wood, SolidPVC, HollowPVC, SolidSteel, HollowSteel, Concrete
	};

	// Constructor
	PhysicsEngine();

	// Returns a SphereGeometry object
	physx::PxSphereGeometry createSphereGeometry(physx::PxReal radius);

	// Returns a CapsuleGeometry object
	physx::PxCapsuleGeometry createCapsuleGeometry(physx::PxReal radius, physx::PxReal halfHeight);

	// Returns a Cooked ConvexMesh Object
	physx::PxConvexMesh *createConvexMesh(physx::PxVec3 *pointCloud, physx::PxU32 numVertices);

	// Returns a ConvexMeshGeometry object (calls createConvexMesh)
	physx::PxConvexMeshGeometry createConvexMeshGeometry(physx::PxVec3 *pointCloud, physx::PxU32 numVertices);

	// Returns a ConvexMeshGeometry object (uses the ConvexMesh passed)
	physx::PxConvexMeshGeometry createConvexMeshGeometry(physx::PxConvexMesh &mesh);

	// Returns a height field
	physx::PxHeightField *createHeightField(physx::PxHeightFieldSample *field, uint32_t nbRows, uint32_t nbCols, physx::PxReal convexEdgeThreshold = 0.0f, physx::PxReal thickness = -1.0f);

	// Returns a heightfield geometry
	physx::PxHeightFieldGeometry createHeightFieldGeometry(physx::PxHeightField *heightField);

	// Returns a triangle mesh
	physx::PxTriangleMesh *createTriangleMesh(physx::PxVec3 *vertices, physx::PxU32 numVertices, physx::PxU32 *indices, physx::PxU32 numIndices);

	// Returns a triangle mesh geometry (good for use as level geometry)
	physx::PxTriangleMeshGeometry createTriangleMeshGeometry(physx::PxTriangleMesh* mesh);

	// Adds a rigid dynamic actor to the scene, and returns a pointer reference to it
	physx::PxRigidDynamic* addRigidDynamic(physx::PxVec3 position, physx::PxQuat orientation, physx::PxGeometry **components, physx::PxVec3 *componentLinearOffsets, physx::PxQuat *componentAngularOffsets, physx::PxU32 numComponents, physx::PxReal Mass, physx::PxVec3 MomentOfInertia, physx::PxVec3 initialLinearVelocity, physx::PxVec3 initialAngularVelocity, Material mat, physx::PxReal linearDamping = 0.0f, physx::PxReal angularDamping = 0.0f);

	// Adds a rigid dynamic actor to the scene and applies aerodynamics to it at each update
	physx::PxRigidDynamic *addRigidAerodynamic(physx::PxVec3 position, physx::PxQuat orientation, physx::PxGeometry **components, physx::PxVec3 *componentLinearOffsets, physx::PxQuat *componentAngularOffsets, physx::PxU32 numComponents, physx::PxReal Mass, physx::PxVec3 MomentOfInertia, physx::PxVec3 initialLinearVelocity, physx::PxVec3 initialAngularVelocity, Material mat, physx::PxReal linearDamping = 0.0f, physx::PxReal angularDamping = 0.0f, physx::PxReal lift = 0.0f, physx::PxReal drag = 0.0f);

	// Adds a rigid static actor to the scene, and returns a pointer reference to it
	physx::PxRigidStatic* addRigidStatic(physx::PxVec3 position, physx::PxQuat orientation, physx::PxGeometry **components, physx::PxVec3 *componentLinearOffsets, physx::PxQuat *componentAngularOffsets, physx::PxU32 numComponents, Material mat);

	// Sets the array of rigid actors to contain all of the actors in the scene
	void getActors(std::vector<physx::PxRigidActor*> &actors);

	// Sets the gravitational force in the scene
	void setGravity(vec3 gravity);

	// Sets the frequency of the engine (in Hz)
	void setFrequency(uint32_t frequency);

	// Returns the inertia tensor of an axis-aligned cube centered on the origin
	static vec3 InertiaTensorSolidCube(physx::PxReal width, physx::PxReal mass);

	// Returns the inertia tensor of a solid sphere centered about the origin
	static vec3 InertiaTensorSolidSphere(physx::PxReal radius, physx::PxReal mass);
	
	// Returns the inertia tensor of a hollow sphere centered about the origin
	static vec3 InertiaTensorHollowSphere(physx::PxReal radius, physx::PxReal mass);

	// Reurns the inertia tensor of a capsule centered about the origin and aligned with the x axis
	static vec3 InertiaTensorSolidCapsule(physx::PxReal radius, physx::PxReal halfHeight, physx::PxReal mass);

	// returns the value of physics (do not use unless absolutely necessary)
	physx::PxPhysics *PhysicsEngine::getPhysics();

	// returns the values of cooking (do not use unless absolutely necessary)
	physx::PxCooking *PhysicsEngine::getCooking();

	// Destructor
	~PhysicsEngine();
};

#endif