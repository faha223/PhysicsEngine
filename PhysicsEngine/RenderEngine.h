#ifndef _RENDER_ENGINE_H_
#define _RENDER_ENGINE_H_

#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>
#include <SDL/SDL.h>
#include <PxPhysicsAPI.h>

class RxMesh;
class RxActor;
class RxTexture;

class RxMesh
{
private:
	uint32_t references;
	uint32_t vertexBuffer;
	uint32_t indexBuffer;
public:
	uint32_t getVertexBuffer() const;
	uint32_t getIndexBuffer() const;
	void Draw() const;
};

class RxActor
{
private:
	uint32_t references;
	RxMesh *mesh;
	physx::PxActor *actor;
public:
	RxMesh *getMesh() const;
	physx::PxActor *getActor() const;
};

class RenderEngine
{
private:
	std::thread *drawThread;
	std::mutex engineMutex;
	std::atomic_int quit;
	SDL_Window *window;
	SDL_GLContext context;

	uint32_t updateFrequency;
	float updatePeriod;

	static void threadFunc(RenderEngine *re);
	void Draw();
public:
	RenderEngine(uint32_t width, uint32_t height, uint32_t MSAA = 16, bool fullscreen = true);

	~RenderEngine();
};

#endif