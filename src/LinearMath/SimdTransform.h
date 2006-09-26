/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef btSimdTransform_H
#define btSimdTransform_H

#include "LinearMath/SimdVector3.h"
#include "LinearMath/SimdMatrix3x3.h"



class btSimdTransform {
	

public:
	
	enum { 
		TRANSLATION = 0x01,
		ROTATION    = 0x02,
		RIGID       = TRANSLATION | ROTATION,  
		SCALING     = 0x04,
		LINEAR      = ROTATION | SCALING,
		AFFINE      = TRANSLATION | LINEAR
	};

	btSimdTransform() {}

	explicit SIMD_FORCE_INLINE btSimdTransform(const btSimdQuaternion& q, 
		const btSimdVector3& c = btSimdVector3(SimdScalar(0), SimdScalar(0), SimdScalar(0))) 
		: m_basis(q),
		m_origin(c),
		m_type(RIGID)
	{}

	explicit SIMD_FORCE_INLINE btSimdTransform(const btSimdMatrix3x3& b, 
		const btSimdVector3& c = btSimdVector3(SimdScalar(0), SimdScalar(0), SimdScalar(0)), 
		unsigned int type = AFFINE)
		: m_basis(b),
		m_origin(c),
		m_type(type)
	{}


		SIMD_FORCE_INLINE void mult(const btSimdTransform& t1, const btSimdTransform& t2) {
			m_basis = t1.m_basis * t2.m_basis;
			m_origin = t1(t2.m_origin);
			m_type = t1.m_type | t2.m_type;
		}

		void multInverseLeft(const btSimdTransform& t1, const btSimdTransform& t2) {
			btSimdVector3 v = t2.m_origin - t1.m_origin;
			if (t1.m_type & SCALING) {
				btSimdMatrix3x3 inv = t1.m_basis.inverse();
				m_basis = inv * t2.m_basis;
				m_origin = inv * v;
			}
			else {
				m_basis = SimdMultTransposeLeft(t1.m_basis, t2.m_basis);
				m_origin = v * t1.m_basis;
			}
			m_type = t1.m_type | t2.m_type;
		}

	SIMD_FORCE_INLINE btSimdVector3 operator()(const btSimdVector3& x) const
	{
		return btSimdVector3(m_basis[0].dot(x) + m_origin[0], 
			m_basis[1].dot(x) + m_origin[1], 
			m_basis[2].dot(x) + m_origin[2]);
	}

	SIMD_FORCE_INLINE btSimdVector3 operator*(const btSimdVector3& x) const
	{
		return (*this)(x);
	}

	SIMD_FORCE_INLINE btSimdMatrix3x3&       getBasis()          { return m_basis; }
	SIMD_FORCE_INLINE const btSimdMatrix3x3& getBasis()    const { return m_basis; }

	SIMD_FORCE_INLINE btSimdVector3&         getOrigin()         { return m_origin; }
	SIMD_FORCE_INLINE const btSimdVector3&   getOrigin()   const { return m_origin; }

	btSimdQuaternion getRotation() const { 
		btSimdQuaternion q;
		m_basis.getRotation(q);
		return q;
	}
	template <typename Scalar2>
		void setValue(const Scalar2 *m) 
	{
		m_basis.setValue(m);
		m_origin.setValue(&m[12]);
		m_type = AFFINE;
	}

	
	void setFromOpenGLMatrix(const SimdScalar *m)
	{
		m_basis.setFromOpenGLSubMatrix(m);
		m_origin[0] = m[12];
		m_origin[1] = m[13];
		m_origin[2] = m[14];
	}

	void getOpenGLMatrix(SimdScalar *m) const 
	{
		m_basis.getOpenGLSubMatrix(m);
		m[12] = m_origin[0];
		m[13] = m_origin[1];
		m[14] = m_origin[2];
		m[15] = SimdScalar(1.0f);
	}

	SIMD_FORCE_INLINE void setOrigin(const btSimdVector3& origin) 
	{ 
		m_origin = origin;
		m_type |= TRANSLATION;
	}

	SIMD_FORCE_INLINE btSimdVector3 invXform(const btSimdVector3& inVec) const;



	SIMD_FORCE_INLINE void setBasis(const btSimdMatrix3x3& basis)
	{ 
		m_basis = basis;
		m_type |= LINEAR;
	}

	SIMD_FORCE_INLINE void setRotation(const btSimdQuaternion& q)
	{
		m_basis.setRotation(q);
		m_type = (m_type & ~LINEAR) | ROTATION;
	}

	SIMD_FORCE_INLINE void scale(const btSimdVector3& scaling)
	{
		m_basis = m_basis.scaled(scaling);
		m_type |= SCALING;
	}

	void setIdentity()
	{
		m_basis.setIdentity();
		m_origin.setValue(SimdScalar(0.0), SimdScalar(0.0), SimdScalar(0.0));
		m_type = 0x0;
	}

	SIMD_FORCE_INLINE bool isIdentity() const { return m_type == 0x0; }

	btSimdTransform& operator*=(const btSimdTransform& t) 
	{
		m_origin += m_basis * t.m_origin;
		m_basis *= t.m_basis;
		m_type |= t.m_type; 
		return *this;
	}

	btSimdTransform inverse() const
	{ 
		if (m_type)
		{
			btSimdMatrix3x3 inv = (m_type & SCALING) ? 
				m_basis.inverse() : 
			m_basis.transpose();

			return btSimdTransform(inv, inv * -m_origin, m_type);
		}

		return *this;
	}

	btSimdTransform inverseTimes(const btSimdTransform& t) const;  

	btSimdTransform operator*(const btSimdTransform& t) const;

private:

	btSimdMatrix3x3 m_basis;
	btSimdVector3   m_origin;
	unsigned int      m_type;
};


SIMD_FORCE_INLINE btSimdVector3
btSimdTransform::invXform(const btSimdVector3& inVec) const
{
	btSimdVector3 v = inVec - m_origin;
	return (m_basis.transpose() * v);
}

SIMD_FORCE_INLINE btSimdTransform 
btSimdTransform::inverseTimes(const btSimdTransform& t) const  
{
	btSimdVector3 v = t.getOrigin() - m_origin;
	if (m_type & SCALING) 
	{
		btSimdMatrix3x3 inv = m_basis.inverse();
		return btSimdTransform(inv * t.getBasis(), inv * v, 
			m_type | t.m_type);
	}
	else 
	{
		return btSimdTransform(m_basis.transposeTimes(t.m_basis),
			v * m_basis, m_type | t.m_type);
	}
}

SIMD_FORCE_INLINE btSimdTransform 
btSimdTransform::operator*(const btSimdTransform& t) const
{
	return btSimdTransform(m_basis * t.m_basis, 
		(*this)(t.m_origin), 
		m_type | t.m_type);
}	



#endif





