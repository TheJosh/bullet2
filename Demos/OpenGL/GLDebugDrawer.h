#ifndef GL_DEBUG_DRAWER_H
#define GL_DEBUG_DRAWER_H

#include "LinearMath/GenIDebugDraw.h"



class GLDebugDrawer : public btIDebugDraw
{
	int m_debugMode;

public:

	GLDebugDrawer();

	virtual void	DrawLine(const btSimdVector3& from,const btSimdVector3& to,const btSimdVector3& color);

	virtual void	DrawContactPoint(const btSimdVector3& PointOnB,const btSimdVector3& normalOnB,float distance,int lifeTime,const btSimdVector3& color);

	virtual void	SetDebugMode(int debugMode);

	virtual int		GetDebugMode() const { return m_debugMode;}

};

#endif//GL_DEBUG_DRAWER_H
