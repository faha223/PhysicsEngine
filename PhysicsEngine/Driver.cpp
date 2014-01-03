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

#define radToDeg($1) ($1*57.295779513082320876798154814105f)

using namespace physx;
using namespace std;

void InitGL();
void Display();
void drawSphere(PxSphereGeometry sphere, PxTransform transform);
void drawCapsule(PxCapsuleGeometry cap, PxTransform transform);
void drawMesh(PxConvexMeshGeometry mesh, PxTransform transform);
void drawMesh(PxTriangleMeshGeometry mesh, PxTransform transform);
void checkGLErrors();
void LoadTexture();

int main(int argc, char *argv[])
{
	// Create a Window and an OpenGL Context to render the simulation
	vector<PxRigidActor*> actors;
	SDL_Window *window;
	SDL_GLContext context;
	bool quit = false;
	map<int, bool> Keyboard;

	PhysicsEngine engine;
	printf("Engine Created\n");

	// The hard part is getting the inertial tensors right
	vec3 cubeVerts[] = { vec3(-1, -1, -1), vec3(1, -1, -1), vec3(-1, 1, -1), vec3(1, 1, -1), vec3(-1, -1, 1), vec3(1, -1, 1), vec3(-1, 1, 1), vec3(1, 1, 1) };
	actors.push_back(engine.addCollisionMesh(vec3(10.0f, 5.0f, -10.0f), quaternion(0, 0, 0, 1), cubeVerts, 8, 10.0f, PhysicsEngine::InertiaTensorSolidCube(2.0f, 10.0f), vec3(-4.0f, -1.0f, 0.0f), vec3(0.0f, 2.0f, 2.0f), PhysicsEngine::Wood));
	actors.push_back(engine.addCollisionCapsule(vec3(0.0f, 0.0f, -10.0f), quaternion(0, 0, 1, 0), 1.0f, 1.0f, 1.0f, vec3(1.0f, 1.0f, 1.0f), vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f, 0.0f), PhysicsEngine::SolidPVC));
	actors.push_back(engine.addCollisionSphere(vec3(-5.0f, 0.0f, -10.0f), quaternion(0, 0, 0, 1), 1.0f, 1.0f, PhysicsEngine::InertiaTensorSolidSphere(1.0f, 1.0f), vec3(2.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f, 0.0f), PhysicsEngine::SolidPVC));
	actors.push_back(engine.addCollisionSphere(vec3(5.0f, 0.5f, -10.0f), quaternion(0, 0, 0, 1), 1.0f, 1.0f, PhysicsEngine::InertiaTensorHollowSphere(1.0f, 1.0f), vec3(-2.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f, -2.0f), PhysicsEngine::HollowPVC));
	
	//engine.setGravity(vec3(0.0f, -9.81f, 0.0f));

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
		//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		Display();
		for (size_t i = 0; i < actors.size(); i++)
		{
			if (actors[i] != nullptr)
			{
				PxTransform transform = actors[i]->getGlobalPose();
				uint32_t numShapes = actors[i]->getNbShapes();
				if (numShapes != 0)
				{
					PxShape** shapes = new PxShape*[numShapes];
					actors[i]->getShapes(shapes, numShapes, 0);
					for (uint32_t j = 0; j < numShapes; j++)
					{
						PxSphereGeometry sphere;
						PxCapsuleGeometry capsule;
						PxConvexMeshGeometry c_mesh;
						PxTriangleMeshGeometry t_mesh;
						if (shapes[j]->getSphereGeometry(sphere))
						{
							drawSphere(sphere, transform);
						}
						else if (shapes[j]->getCapsuleGeometry(capsule))
						{
							drawCapsule(capsule, transform);
						}
						else if (shapes[j]->getConvexMeshGeometry(c_mesh))
						{
							drawMesh(c_mesh, transform);
						}
						else if (shapes[j]->getTriangleMeshGeometry(t_mesh))
						{
							drawMesh(t_mesh, transform);
						}
					}
				}
			}
		}
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


void drawSphere(PxSphereGeometry sphere, PxTransform transform)
{
	GLUquadric *w = gluNewQuadric();

	glPushMatrix();
	
	glTranslatef(transform.p.x, transform.p.y, transform.p.z);
	vec3 axis = transform.q.getImaginaryPart();
	glRotatef(radToDeg(transform.q.getAngle()), axis.x, axis.y, axis.z);

	gluQuadricTexture(w, true);
		
	// Draw a standard sphere of the correct radius
	gluSphere(w, sphere.radius, 32, 32);

	glPopMatrix();

	gluDeleteQuadric(w);
}

void drawCapsule(PxCapsuleGeometry cap, PxTransform transform)
{
	GLUquadric *w = gluNewQuadric();

	glPushMatrix();

	glTranslatef(transform.p.x, transform.p.y, transform.p.z);
	vec3 axis = transform.q.getImaginaryPart();
	glRotatef(radToDeg(transform.q.getAngle()), axis.x, axis.y, axis.z);

	gluQuadricTexture(w, true);

	// Draw a standard capsule
	glPushMatrix();
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	glTranslatef(0.0f, 0.0f, cap.halfHeight);
	gluSphere(w, cap.radius, 32, 32);
	glTranslatef(0.0f, 0.0f, -2.0f*cap.halfHeight);
	gluSphere(w, cap.radius, 32, 32);
	gluCylinder(w, cap.radius, cap.radius, 2.0f*cap.halfHeight, 32, 1);
	glPopMatrix();

	glPopMatrix();

	gluDeleteQuadric(w);

}

void drawMesh(PxConvexMeshGeometry mesh, PxTransform transform)
{
	glPushMatrix();

	glTranslatef(transform.p.x, transform.p.y, transform.p.z);
	vec3 axis = transform.q.getImaginaryPart();
	glRotatef(radToDeg(transform.q.getAngle()), axis.x, axis.y, axis.z);

	// Draw the mesh here
	PxU32 nbVerts = mesh.convexMesh->getNbVertices();
	const PxVec3* convexVerts = mesh.convexMesh->getVertices();
	const PxU8* indexBuffer = mesh.convexMesh->getIndexBuffer();

	vector<vec3> vertices;

	PxU32 offset = 0;
	for (PxU32 i = 0; i<mesh.convexMesh->getNbPolygons(); i++)
	{
		PxHullPolygon face;
		bool status = mesh.convexMesh->getPolygonData(i, face);
		PX_ASSERT(status);

		/*for (int j = 2; j < face.mNbVerts; j++)
		{
			glTexCoord2f(0.0f, 0.0f);
			glVertex3f(vertices[face.mIndexBase].x, vertices[face.mIndexBase].y, vertices[face.mIndexBase].z);
			glTexCoord2f(1.0f, 0.0f);
			glVertex3f(vertices[face.mIndexBase + j].x, vertices[face.mIndexBase + j].y, vertices[face.mIndexBase + j].z);
			glTexCoord2f(0.0f, 1.0f);
			glVertex3f(vertices[face.mIndexBase + j - 1].x, vertices[face.mIndexBase + j - 1].y, vertices[face.mIndexBase + j - 1].z);
		}*/

		const PxU8* faceIndices = indexBuffer + face.mIndexBase;
		for (PxU32 j = 0; j<face.mNbVerts; j++)
		{
			vertices.push_back(convexVerts[faceIndices[j]]);
		}

		glBegin(GL_TRIANGLES);
		for (PxU32 j = 2; j<face.mNbVerts; j++)
		{
			glTexCoord2f(0.0f, 0.0f);
			glVertex3f(vertices[offset].x, vertices[offset].y, vertices[offset].z);
			glTexCoord2f(1.0f, 0.0f);
			glVertex3f(vertices[offset+j].x, vertices[offset+j].y, vertices[offset+j].z);
			glTexCoord2f(0.0f, 1.0f);
			glVertex3f(vertices[offset+j-1].x, vertices[offset+j-1].y, vertices[offset+j-1].z);
		}
		glEnd();

		offset += face.mNbVerts;
	}


	
	glPopMatrix();
}

void drawMesh(PxTriangleMeshGeometry mesh, PxTransform transform)
{
	glPushMatrix();

	glTranslatef(transform.p.x, transform.p.y, transform.p.z);
	vec3 axis = transform.q.getImaginaryPart();
	glRotatef(radToDeg(transform.q.getAngle()), axis.x, axis.y, axis.z);

	glBegin(GL_TRIANGLES);
	const vec3* vertices = mesh.triangleMesh->getVertices();
	if (mesh.triangleMesh->getTriangleMeshFlags() & physx::PxTriangleMeshFlag::eHAS_16BIT_TRIANGLE_INDICES)
	{
		physx::PxU16 *indices = (physx::PxU16*)mesh.triangleMesh->getTriangles();
		for (unsigned short i = 0; i < mesh.triangleMesh->getNbTriangles(); i++)
		{
			glVertex3f(vertices[indices[i]].x, vertices[indices[i]].y, vertices[indices[i]].z);
		}
	}
	else
	{
		physx::PxU32 *indices = (physx::PxU32*)mesh.triangleMesh->getTriangles();
		for (unsigned int i = 0; i < mesh.triangleMesh->getNbTriangles(); i++)
		{
			glVertex3f(vertices[indices[i]].x, vertices[indices[i]].y, vertices[indices[i]].z);
		}
	}
	glEnd();
	 

	glPopMatrix();
}

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