#include "RenderEngine.h"
#include <glew/glew.h>

using namespace std;
using namespace physx; 

#pragma comment(lib, "glew32.lib")

RenderEngine::RenderEngine(uint32_t width, uint32_t height, uint32_t MSAA, bool fullscreen) :
drawThread(nullptr),
window(nullptr),
updateFrequency(120)
{
	if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
	{
		printf("SDL_Init Error: %s\n", SDL_GetError());
		return;
	}

	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
	SDL_GL_SetAttribute(SDL_GL_ACCELERATED_VISUAL, 1);	// Force hardware acceleration
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);			// 24 bit depth buffer
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);		// Enable Double Buffering
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, (MSAA>0)?1:0);	// Enable Multisampling
	if (MSAA > 0)
		SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, MSAA);	// 4x MSAA

	window = SDL_CreateWindow("Physics Engine Driver", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL | ((fullscreen)?SDL_WINDOW_FULLSCREEN:0));
	if (window == nullptr)
	{
		printf("SDL_CreateWindow Error: %s\n", SDL_GetError());
		SDL_Quit();
		return;
	}

	context = SDL_GL_CreateContext(window);

	if (SDL_GL_SetSwapInterval(-1) < 0)
	{
		printf("Late Frame Tearing not available\n");
		SDL_GL_SetSwapInterval(1);
	}
	glewInit();

	quit.store(0, memory_order_release);

	drawThread = new thread(threadFunc, this);
}

void RenderEngine::threadFunc(RenderEngine *re)
{
	if (re == nullptr)
		return;
	while (!re->quit.load(memory_order_acquire))
	{
		chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
		re->Draw();
		chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();
		this_thread::sleep_for(chrono::milliseconds(uint32_t(1000 * re->updatePeriod)) - (end - start));
	}
}

void RenderEngine::Draw()
{
	if (window)
	{
		unique_lock<mutex> lock(engineMutex);
		// Clear Buffers
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// TODO: Draw Things

		// Swap Buffers
		SDL_GL_SwapWindow(window);
	}
}

RenderEngine::~RenderEngine()
{
	if (drawThread)
	{
		quit.store(1, memory_order_release);
		drawThread->join();
	}

	SDL_GL_DeleteContext(context);
	SDL_DestroyWindow(window);
	SDL_Quit();
}