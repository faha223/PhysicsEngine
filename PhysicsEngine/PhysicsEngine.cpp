#include "PhysicsEngine.h"
#include "MaterialProperties.h"

using namespace physx;
using namespace std;

PhysicsEngine::PhysicsEngine():
	updateThread(nullptr),
	physics(nullptr),
	foundation(nullptr),
	scene(nullptr),
	engineFrequency(360)
{
	static PxDefaultErrorCallback gDefaultErrorCallback;
	static PxDefaultAllocator gDefaultAllocatorCallback;
	
	quit.store(0, std::memory_order_release);

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
	while (0 == pe->quit.load(std::memory_order_acquire))
	{
		chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
		pe->update();
		chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();
		this_thread::sleep_for(chrono::milliseconds(long(1000 * pe->simulationPeriod)) - (end-start));
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

#pragma region Add Actors
PxRigidDynamic* PhysicsEngine::addRigidDynamic(PxVec3 position, PxQuat orientation, PxGeometry **components, PxVec3 *componentLinearOffsets, PxQuat *componentAngularOffsets, PxU32 numComponents, PxReal Mass, PxVec3 MomentOfInertia, PxVec3 initialLinearVelocity, PxVec3 initialAngularVelocity, Material mat, PxReal linearDamping, PxReal angularDamping)
{
	unique_lock<mutex> lock(engineMutex);
	if ((physics == nullptr) || (scene == nullptr))
		return nullptr;
	
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

	PxRigidDynamic *newActor = physics->createRigidDynamic(PxTransform(position, orientation));
	// If the designer requested Infinite mass, set the mass to 1 and make the actor kinematic (animated, dynamic, behaves as though it has infinite mass)
	if (Mass < FLT_MAX)
		newActor->setMass(Mass);
	else
	{
		newActor->setMass(1.0f);
		newActor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	}
	newActor->setGlobalPose(PxTransform(position, orientation));
	newActor->setMassSpaceInertiaTensor(MomentOfInertia);
	newActor->setLinearVelocity(initialLinearVelocity);
	newActor->setAngularVelocity(initialAngularVelocity);
	newActor->setLinearDamping(linearDamping);
	newActor->setAngularDamping(angularDamping);
	for (PxU32 i = 0; i < numComponents; i++)
	{
		PxShape* shape = newActor->createShape(*components[i], *mtls[mat]);
		shape->setLocalPose(PxTransform(componentLinearOffsets[i], componentAngularOffsets[i]));
	}
	scene->addActor(*newActor);
	return newActor;
}

PxRigidStatic* PhysicsEngine::addRigidStatic(PxVec3 position, PxQuat orientation, PxGeometry **components, PxVec3 *componentLinearOffsets, PxQuat *componentAngularOffsets, PxU32 numComponents, Material mat)
{
	unique_lock<mutex> lock(engineMutex);
	if ((physics == nullptr) || (scene == nullptr))
		return nullptr;
	
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

	PxRigidStatic *newActor = physics->createRigidStatic(PxTransform(position, orientation));
	for (PxU32 i = 0; i < numComponents; i++)
	{
		PxShape *shape = newActor->createShape(*components[i], *mtls[mat]);
		shape->setLocalPose(PxTransform(componentLinearOffsets[i], componentAngularOffsets[i]));
	}
	scene->addActor(*newActor);
	return newActor;
}
#pragma endregion

PxSphereGeometry PhysicsEngine::createSphereGeometry(PxReal radius)
{
	unique_lock<mutex> lock(engineMutex);
	return PxSphereGeometry(radius);
}

PxConvexMesh *PhysicsEngine::createConvexMesh(PxVec3 *pointCloud, PxU32 numVertices)
{
	unique_lock<mutex> lock(engineMutex);
	if ((physics == nullptr) || (cooking == nullptr))
		return nullptr;
	PxConvexMeshDesc meshDesc;
	meshDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
	meshDesc.vertexLimit = 256;
	meshDesc.points.count = numVertices;
	meshDesc.points.data = pointCloud;
	meshDesc.points.stride = sizeof(PxVec3);
	PxDefaultMemoryOutputStream buf;
	if (!cooking->cookConvexMesh(meshDesc, buf))
		return nullptr;
	return physics->createConvexMesh(PxDefaultMemoryInputData(buf.getData(), buf.getSize()));
}

PxConvexMeshGeometry PhysicsEngine::createConvexMeshGeometry(PxConvexMesh &mesh)
{
	unique_lock<mutex> lock(engineMutex);
	PxConvexMeshGeometry geometry;
		geometry.convexMesh = &mesh;
	return geometry;
}

PxConvexMeshGeometry PhysicsEngine::createConvexMeshGeometry(PxVec3 *pointCloud, PxU32 numVertices)
{
	unique_lock<mutex> lock(engineMutex);
	PxConvexMeshGeometry geometry;
	lock.unlock();
	PxConvexMesh *mesh = createConvexMesh(pointCloud, numVertices);
	lock.lock();
	if (mesh)
		geometry.convexMesh = mesh;
	return geometry;
}

PxCapsuleGeometry PhysicsEngine::createCapsuleGeometry(PxReal radius, PxReal halfHeight)
{
	unique_lock<mutex> lock(engineMutex);
	return PxCapsuleGeometry(radius, halfHeight);
}

PxHeightField *PhysicsEngine::createHeightField(uint8_t *field, uint32_t width, uint32_t height, float scale, float bias)
{
	unique_lock<mutex> lock(engineMutex);
	if ((physics == nullptr) || (cooking == nullptr))
		return nullptr;
	return nullptr;
	PxHeightFieldDesc heightfieldDesc;
	// TODO: Fill the heightfield with the proper values
	PxDefaultMemoryOutputStream buf;
	cooking->cookHeightField(heightfieldDesc, buf);
	return physics->createHeightField(PxDefaultMemoryInputData(buf.getData(), buf.getSize()));
}

PxHeightFieldGeometry PhysicsEngine::createHeightFieldGeometry(PxHeightField *heightField)
{
	unique_lock<mutex> lock(engineMutex);
	PxHeightFieldGeometry geom;
	geom.heightField = heightField;
	return geom;
}

PxTriangleMesh *PhysicsEngine::createTriangleMesh(PxVec3 *vertices, PxU32 numVertices, PxU32 *indices, PxU32 numIndices)
{
	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = numIndices;
	meshDesc.points.data = vertices;
	meshDesc.points.stride = sizeof(PxVec3);
	meshDesc.triangles.count = numIndices / 3;
	meshDesc.triangles.data = indices;
	meshDesc.triangles.stride = 3 * sizeof(PxU32);
	PxDefaultMemoryOutputStream buf;
	if (!cooking->cookTriangleMesh(meshDesc, buf))
		return nullptr;
	return physics->createTriangleMesh(PxDefaultMemoryInputData(buf.getData(), buf.getSize()));
}

PxTriangleMeshGeometry PhysicsEngine::createTriangleMeshGeometry(PxTriangleMesh* mesh)
{
	unique_lock<mutex> lock(engineMutex);
	PxTriangleMeshGeometry geometry;
	geometry.triangleMesh = mesh;
	return geometry;
}

void PhysicsEngine::setGravity(vec3 gravity)
{
	unique_lock<mutex> lock(engineMutex);
	if (scene != nullptr)
	{
		scene->setGravity(gravity);
	}
}

#pragma region Common Inertia Tensors
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
#pragma endregion

PxPhysics *PhysicsEngine::getPhysics()
{
	unique_lock<mutex> lock(engineMutex);
	return physics;
}

PxCooking *PhysicsEngine::getCooking()
{
	unique_lock<mutex> lock(engineMutex);
	return cooking;
}

PhysicsEngine::~PhysicsEngine()
{
	if (updateThread != nullptr)
	{
		{
			unique_lock<mutex> lock(engineMutex);
			quit.store(1, std::memory_order_release);
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
		foundation->release();
	}
}