#include <SDL/SDL.h>
#ifdef _WIN32
#include <glew/glew.h>
#else
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#endif
#include <cstdio>
#include <map>

#include "PhysicsEngine.h"

void InitGL();
void Display();
void checkGLErrors();
void LoadTexture();

int main(int argc, char *argv[])
{
	// Create a Window and an OpenGL Context to render the simulation
	SDL_Window *window;
	SDL_GLContext context;
	bool quit = false;
	std::map<int, bool> Keyboard;

	PhysicsEngine engine;
	printf("Engine Created\n");
	
	if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
	{
		printf("SDL_Init Error: %s\n", SDL_GetError());
		return 0;
	}

	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
	SDL_GL_SetAttribute(SDL_GL_ACCELERATED_VISUAL, 1);	// Force hardware acceleration
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);			// 24 bit depth buffer
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);		// Enable Double Buffering
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);	// Enable Multisampling
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);	// 4x MSAA

	window = SDL_CreateWindow("Physics Engine Driver", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL);// | SDL_WINDOW_FULLSCREEN);
	if (window == nullptr)
	{
		printf("SDL_CreateWindow Error: %s\n", SDL_GetError());
		SDL_Quit();
		return false;
	}

	context = SDL_GL_CreateContext(window);

	if (SDL_GL_SetSwapInterval(-1) < 0)
	{
		printf("Late Frame Tearing not available\n");
		SDL_GL_SetSwapInterval(1);
	}
	glewInit();
	LoadTexture();
	InitGL();

	while (!quit)
	{
		SDL_Event e;
		while (SDL_PollEvent(&e))
		{
			switch (e.type)
			{
			case SDL_QUIT:
				quit = true;
				break;
			case SDL_KEYDOWN:
				Keyboard[e.key.keysym.sym] = true;
				break;
			case SDL_KEYUP:
				Keyboard[e.key.keysym.sym] = false;
				break;
			}
			if (Keyboard[SDLK_ESCAPE])
				quit = true;
		}
		
		Display();
		SDL_GL_SwapWindow(window);
		checkGLErrors();
	}

	SDL_GL_DeleteContext(context);
	SDL_DestroyWindow(window);
	SDL_Quit();
	return 0;
}

void InitGL()
{
	real aspectRatio = 1280.0f / 720.0f;
	glClearColor(0.5, 0.5, 1.0, 1.0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-1.0f, 1.0f, -1.0/aspectRatio, 1.0/aspectRatio, 1.0, 1000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
}

void Display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

/*
void drawSphere(const BoundingSphere &s)
{
	GLUquadric *w = gluNewQuadric();

	glPushMatrix();
	glTranslatef(s.position.x, s.position.y, s.position.z);

	gluQuadricTexture(w, true);
	gluSphere(w, s.radius, 32, 32);

	glPopMatrix();

	gluDeleteQuadric(w);
}
*/
void checkGLErrors()
{
	int glError = glGetError();
	do
	{
		if (glError > 0)
		{
			printf("OpenGL Error: %s\n", gluErrorString(glError));
			glError = glGetError();
		}
	} while (glError > 0);
}

void LoadTexture()
{
	const unsigned char texturePixels[] = 
	{	
		0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 
		255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 
		0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 
		255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0,
		0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 
		255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0,
		0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 
		255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0, 255, 255, 255, 0, 0, 0
	};

	GLuint textureHandle = 0;
	glGenTextures(1, &textureHandle);
	if (textureHandle == 0)
	{
		printf("Unable to generate Texture: %s\n", gluErrorString(glGetError()));
		return;
	}
	glBindTexture(GL_TEXTURE_2D, textureHandle);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 16);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 8, 8, 0, GL_RGB, GL_UNSIGNED_BYTE, texturePixels);
	glGenerateMipmap(GL_TEXTURE_2D);
}