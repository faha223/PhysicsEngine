#ifndef __PXGL_H__
#define __PXGL_H__

#include <glew/glew.h>
#include "types.h"
#define radToDeg($1) ($1*57.295779513082320876798154814105f)
#define degToRad($1) ($1/57.295779513082320876798154814105f)

void glTransformPx(physx::PxTransform transform)
{
	glTranslatef(transform.p.x, transform.p.y, transform.p.z);
	vec3 axis;
	float angle;
	transform.q.toRadiansAndUnitAxis(angle, axis);
	glRotatef(radToDeg(angle), axis.x, axis.y, axis.z);
}

#endif