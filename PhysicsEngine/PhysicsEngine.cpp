#include "PhysicsEngine.h"
#include "MaterialProperties.h"

using namespace physx;
using namespace std;

PhysicsEngine::PhysicsEngine():
	updateThread(nullptr),
	physics(nullptr),
	foundation(nullptr),
	scene(nullptr),
	engineFrequency(360),
	quit(false)
{
	static PxDefaultErrorCallback gDefaultErrorCallback;
	static PxDefaultAllocator gDefaultAllocatorCallback;
	
	tolScale = PxTolerancesScale();
	foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
	if (!foundation)
	{
		printf("Error: PxCreateFoundation Failed\n");
		return;
	}

	cooking = PxCreateCooking(PX_PHYSICS_VERSION, *foundation, PxCookingParams(tolScale));
	if (!cooking)
	{
		printf("Error: PxCreateCooking Failed\n");
		return;
	}

	physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, tolScale);
	if (!physics)
	{
		printf("Error: PxCreatePhysics Failed\n");
		return;
	}

	PxSceneDesc sceneDesc = PxSceneDesc(tolScale);

	if (!sceneDesc.cpuDispatcher)
	{
		PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
		sceneDesc.cpuDispatcher = mCpuDispatcher;
	}

	if (!sceneDesc.filterShader)
	{
		sceneDesc.filterShader = PxDefaultSimulationFilterShader;  //Collision filter mechanism, strange name for it, very misleading
	}

	scene = physics->createScene(sceneDesc);
	if (!scene)
	{
		printf("Error: createScene Failed\n");
		return;
	}

	mtls[Wood] = physics->createMaterial(WOOD_STATIC_FRICTION, WOOD_DYNAMIC_FRICTION, WOOD_RESTITUTION);
	mtls[HollowPVC] = physics->createMaterial(HOLLOWPVC_STATIC_FRICTION, HOLLOWPVC_DYNAMIC_FRICTION, HOLLOWPVC_RESTITUTION);
	mtls[SolidPVC] = physics->createMaterial(SOLIDPVC_STATIC_FRICTION, SOLIDPVC_DYNAMIC_FRICTION, SOLIDPVC_RESTITUTION);
	mtls[HollowSteel] = physics->createMaterial(HOLLOWSTEEL_STATIC_FRICTION, HOLLOWSTEEL_DYNAMIC_FRICTION, HOLLOWSTEEL_RESTITUTION);
	mtls[SolidSteel] = physics->createMaterial(SOLIDSTEEL_STATIC_FRICTION, SOLIDSTEEL_DYNAMIC_FRICTION, SOLIDSTEEL_RESTITUTION);
	mtls[Concrete] = physics->createMaterial(CONCRETE_STATIC_FRICTION, CONCRETE_DYNAMIC_FRICTION, CONCRETE_RESTITUTION);

	simulationPeriod = 6.0f / float(engineFrequency);

	updateThread = new thread(updateLoop, this);
}

void PhysicsEngine::updateLoop(PhysicsEngine *pe)
{
	if (pe == nullptr)
		return;
	unique_lock<mutex> lock(pe->engineMutex);
	while (!pe->quit)
	{
		lock.unlock();
		chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
		pe->update();
		chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();
		this_thread::sleep_for(chrono::milliseconds(long(1000 * pe->simulationPeriod)) - (end-start));

		lock.lock();
	}
}

void PhysicsEngine::update()
{
	unique_lock<mutex> lock(engineMutex);
	if (scene != nullptr)
	{
		scene->simulate(simulationPeriod);
		scene->fetchResults(true);
	}
}

void PhysicsEngine::getActors(vector<PxRigidActor*> &actors)
{
	unique_lock<mutex> lock(engineMutex);
	if (scene != nullptr)
	{
		uint32_t count = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
		PxActor **aa = new PxActor*[count];
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, aa, count);
		actors.clear();
		for (uint32_t i = 0; i < count; i++)
		{
			actors.push_back(aa[i]->isRigidActor());
		}
		delete[] aa;
	}
}

PxRigidActor* PhysicsEngine::addCollisionSphere(vec3 position, quaternion orientation, real radius, real Mass, vec3 massSpaceInertiaTensor, vec3 initialLinearVelocity, vec3 initialAngularVelocity, Material mat, bool isDynamic)
{
	// Lock the thread while we add a collision sphere to the simulation
	unique_lock<mutex> lock(engineMutex);
	if (scene != nullptr)
	{
		PxSphereGeometry geometry(radius);
		switch (mat)
		{
		case Wood:
		case HollowPVC:
		case SolidPVC:
		case HollowSteel:
		case SolidSteel:
		case Concrete:
			break;
		default:
			return nullptr;
		}
		if (isDynamic)
		{
			PxRigidDynamic *newActor = physics->createRigidDynamic(PxTransform(position, orientation));
			newActor->createShape(geometry, *mtls[mat]);
			newActor->setMass(Mass);
			newActor->setMassSpaceInertiaTensor(massSpaceInertiaTensor);
			newActor->setLinearVelocity(initialLinearVelocity, true);
			newActor->setAngularVelocity(initialAngularVelocity, true);
			scene->addActor(*newActor);
			return newActor;
		}
		else
		{
			PxRigidStatic *newActor = physics->createRigidStatic(PxTransform(position, orientation));
			newActor->createShape(geometry, *mtls[mat]);
			scene->addActor(*newActor);
			return newActor;
		}
	}
	return nullptr;
}

PxRigidActor* PhysicsEngine::addCollisionCapsule(vec3 position, quaternion orientation, real halfHeight, real radius, real Mass, vec3 massSpaceInertiaTensor, vec3 initialLinearVelocity, vec3 initialAngularVelocity, Material mat, bool isDynamic)
{
	unique_lock<mutex> lock(engineMutex);
	if (scene != nullptr)
	{
		PxCapsuleGeometry geometry(radius, halfHeight);
		switch (mat)
		{
		case Wood:
		case HollowPVC:
		case SolidPVC:
		case HollowSteel:
		case SolidSteel:
		case Concrete:
			break;
		default:
			return nullptr;
		}
		if (isDynamic)
		{
			PxRigidDynamic *newActor = physics->createRigidDynamic(PxTransform(position, orientation));
			newActor->createShape(geometry, *mtls[mat]);
			newActor->setMass(Mass);
			newActor->setMassSpaceInertiaTensor(massSpaceInertiaTensor);
			newActor->setLinearVelocity(initialLinearVelocity, true);
			newActor->setAngularVelocity(initialAngularVelocity, true);
			scene->addActor(*newActor);
			return newActor;
		}
		else
		{
			PxRigidStatic *newActor = physics->createRigidStatic(PxTransform(position, orientation));
			newActor->createShape(geometry, *mtls[mat]);
			scene->addActor(*newActor);
			return newActor;
		}
	}
	return nullptr;
}

PxRigidActor* PhysicsEngine::addCollisionMesh(vec3 position, quaternion orientation, vec3 *vertices, uint32_t numVertices, real mass, vec3 massSpaceInertiaTensor, vec3 initialLinearVelocity, vec3 initialAngularVelocity, Material mat, bool isDynamic)
{
	unique_lock<mutex> lock(engineMutex);
	if (scene != nullptr)
	{
		switch (mat)
		{
		case Wood:
		case HollowPVC:
		case SolidPVC:
		case HollowSteel:
		case SolidSteel:
		case Concrete:
			break;
		default:
			return nullptr;
		}
		PxConvexMeshDesc meshDesc;
		meshDesc.flags			= PxConvexFlag::eCOMPUTE_CONVEX;
		meshDesc.vertexLimit	= 256;
		meshDesc.points.count	= numVertices;
		meshDesc.points.data	= vertices;
		meshDesc.points.stride	= sizeof(PxVec3);
		PxDefaultMemoryOutputStream buf;
		cooking->cookConvexMesh(meshDesc, buf);
		PxConvexMesh *mesh = physics->createConvexMesh(PxDefaultMemoryInputData(buf.getData(), buf.getSize()));
		PxConvexMeshGeometry geometry;
		geometry.convexMesh = mesh;

		if (isDynamic)
		{
			PxRigidDynamic *newActor = physics->createRigidDynamic(PxTransform(position, orientation));
			newActor->createShape(geometry, *mtls[mat]);
			newActor->setMass(mass);
			newActor->setMassSpaceInertiaTensor(massSpaceInertiaTensor);
			newActor->setLinearVelocity(initialLinearVelocity, true);
			newActor->setAngularVelocity(initialAngularVelocity, true);
			scene->addActor(*newActor);
			return newActor;
		}
		else
		{
			PxRigidStatic *newActor = physics->createRigidStatic(PxTransform(position, orientation));
			newActor->createShape(geometry, *mtls[mat]);
			scene->addActor(*newActor);
			return newActor;
		}
	}
	return nullptr;
}

void PhysicsEngine::setGravity(vec3 gravity)
{
	unique_lock<mutex> lock(engineMutex);
	if (scene != nullptr)
	{
		scene->setGravity(gravity);
	}
}

vec3 PhysicsEngine::InertiaTensorSolidSphere(PxReal radius, PxReal mass)
{
	return vec3((mass*radius*radius)*0.4f);
}

vec3 PhysicsEngine::InertiaTensorHollowSphere(PxReal radius, PxReal mass)
{
	return vec3((mass*radius*radius)*(0.2f/0.3f));
}

vec3 PhysicsEngine::InertiaTensorSolidCube(PxReal width, PxReal mass)
{
	return vec3((mass*width*width)/6.0f);
}

vec3 PhysicsEngine::InertiaTensorSolidCapsule(PxReal radius, PxReal halfHeight, PxReal mass)
{
	// TODO: Replace with correct values
	
	PxReal capmass = mass*(((4.0f / 3.0f)*radius) / (2.0f*halfHeight + (4.0f / 3.0f)*radius));
	PxReal bodymass = mass*((2.0f*halfHeight) / (2.0f*halfHeight + (4.0f / 3.0f)*radius));

	return vec3(radius*radius*(0.4f*capmass + 0.5f*bodymass),
		bodymass*((0.25f*radius*radius) + (halfHeight*halfHeight / 3.0f)) + capmass*radius*(halfHeight + 0.375f + (0.259f*radius)),
		bodymass*((0.25f*radius*radius) + (halfHeight*halfHeight / 3.0f)) + capmass*radius*(halfHeight + 0.375f + (0.259f*radius)));
}

PhysicsEngine::~PhysicsEngine()
{
	if (updateThread != nullptr)
	{
		{
			unique_lock<mutex> lock(engineMutex);
			quit = true;
		}
		updateThread->join();
	}

	if (physics != nullptr)
	{
		physics->release();
	}

	if (cooking != nullptr)
	{
		cooking->release();
	}

	if (foundation != nullptr)
	{
		printf("Releasing PhysX Foundation\n");
		foundation->release();
	}
}