#include "PhysicsEngine.h"
using namespace physx;

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

	printf("Creating Tolerances Scale\n");
	tolScale = PxTolerancesScale();
	printf("Creating Foundation\n");
	foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
	printf("Creating Physics\n");
	physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, tolScale);
	printf("PhysX Initialized\n");

	simulationPeriod = 1.0f / float(engineFrequency);

	scene = physics->createScene(PxSceneDesc(tolScale));

	updateThread = new std::thread(updateLoop, this);
}

void PhysicsEngine::updateLoop(PhysicsEngine *pe)
{
	if (pe == nullptr)
		return;
	std::unique_lock<std::mutex> lock(pe->engineMutex);
	while (!pe->quit)
	{
		lock.unlock();
		std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		pe->update();
		std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(long(1000 * pe->simulationPeriod)) - (end-start));

		lock.lock();
	}
}

void PhysicsEngine::update()
{
	std::unique_lock<std::mutex> lock(engineMutex);
	if (scene != nullptr)
	{
		printf("Updating Scene\n");
		scene->simulate(simulationPeriod);
		scene->fetchResults();
	}
}

PhysicsEngine::~PhysicsEngine()
{
	if (updateThread != nullptr)
	{
		printf("Shutting Down Update Thread\n");
		{
			std::unique_lock<std::mutex> lock(engineMutex);
			quit = true;
		}
		updateThread->join();
	}

	if (physics != nullptr)
	{
		printf("Releasing Physics\n");
		physics->release();
	}

	if (foundation != nullptr)
	{
		printf("Releasing PhysX Foundation\n");
		foundation->release();
	}
}