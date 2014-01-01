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

	physx::PxScene *scene;						// Default Scene

	uint32_t engineFrequency;					// DEFAULT: 360 Hz
	physx::PxReal simulationPeriod;					// DEFAULT: 1/360 (updated whenever engineFrequency is updated)

	bool quit;									// DEFAULT: false

	static void updateLoop(PhysicsEngine *pe);	// The static function that calls the update method at regular intervals
	void update();

public:
	PhysicsEngine();

	void AddPhysicsEntity();

	void setFrequency(uint32_t frequency);

	~PhysicsEngine();
};

#endif