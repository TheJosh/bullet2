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



#ifndef SIMD__QUATERNION_H_
#define SIMD__QUATERNION_H_

#include "LinearMath/SimdVector3.h"

class btSimdQuaternion : public btSimdQuadWord {
public:
	btSimdQuaternion() {}

	//		template <typename SimdScalar>
	//		explicit Quaternion(const SimdScalar *v) : Tuple4<SimdScalar>(v) {}

	btSimdQuaternion(const SimdScalar& x, const SimdScalar& y, const SimdScalar& z, const SimdScalar& w) 
		: btSimdQuadWord(x, y, z, w) 
	{}

	btSimdQuaternion(const btSimdVector3& axis, const SimdScalar& angle) 
	{ 
		setRotation(axis, angle); 
	}

	btSimdQuaternion(const SimdScalar& yaw, const SimdScalar& pitch, const SimdScalar& roll)
	{ 
		setEuler(yaw, pitch, roll); 
	}

	void setRotation(const btSimdVector3& axis, const SimdScalar& angle)
	{
		SimdScalar d = axis.length();
		assert(d != SimdScalar(0.0));
		SimdScalar s = SimdSin(angle * SimdScalar(0.5)) / d;
		setValue(axis.x() * s, axis.y() * s, axis.z() * s, 
			SimdCos(angle * SimdScalar(0.5)));
	}

	void setEuler(const SimdScalar& yaw, const SimdScalar& pitch, const SimdScalar& roll)
	{
		SimdScalar halfYaw = SimdScalar(yaw) * SimdScalar(0.5);  
		SimdScalar halfPitch = SimdScalar(pitch) * SimdScalar(0.5);  
		SimdScalar halfRoll = SimdScalar(roll) * SimdScalar(0.5);  
		SimdScalar cosYaw = SimdCos(halfYaw);
		SimdScalar sinYaw = SimdSin(halfYaw);
		SimdScalar cosPitch = SimdCos(halfPitch);
		SimdScalar sinPitch = SimdSin(halfPitch);
		SimdScalar cosRoll = SimdCos(halfRoll);
		SimdScalar sinRoll = SimdSin(halfRoll);
		setValue(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
			cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
			sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
			cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
	}

	btSimdQuaternion& operator+=(const btSimdQuaternion& q)
	{
		m_x += q.x(); m_y += q.y(); m_z += q.z(); m_unusedW += q[3];
		return *this;
	}

	btSimdQuaternion& operator-=(const btSimdQuaternion& q) 
	{
		m_x -= q.x(); m_y -= q.y(); m_z -= q.z(); m_unusedW -= q[3];
		return *this;
	}

	btSimdQuaternion& operator*=(const SimdScalar& s)
	{
		m_x *= s; m_y *= s; m_z *= s; m_unusedW *= s;
		return *this;
	}


	btSimdQuaternion& operator*=(const btSimdQuaternion& q)
	{
		setValue(m_unusedW * q.x() + m_x * q[3] + m_y * q.z() - m_z * q.y(),
			m_unusedW * q.y() + m_y * q[3] + m_z * q.x() - m_x * q.z(),
			m_unusedW * q.z() + m_z * q[3] + m_x * q.y() - m_y * q.x(),
			m_unusedW * q[3] - m_x * q.x() - m_y * q.y() - m_z * q.z());
		return *this;
	}

	SimdScalar dot(const btSimdQuaternion& q) const
	{
		return m_x * q.x() + m_y * q.y() + m_z * q.z() + m_unusedW * q[3];
	}

	SimdScalar length2() const
	{
		return dot(*this);
	}

	SimdScalar length() const
	{
		return SimdSqrt(length2());
	}

	btSimdQuaternion& normalize() 
	{
		return *this /= length();
	}

	SIMD_FORCE_INLINE btSimdQuaternion
	operator*(const SimdScalar& s) const
	{
		return btSimdQuaternion(x() * s, y() * s, z() * s, m_unusedW * s);
	}



	btSimdQuaternion operator/(const SimdScalar& s) const
	{
		assert(s != SimdScalar(0.0));
		return *this * (SimdScalar(1.0) / s);
	}


	btSimdQuaternion& operator/=(const SimdScalar& s) 
	{
		assert(s != SimdScalar(0.0));
		return *this *= SimdScalar(1.0) / s;
	}


	btSimdQuaternion normalized() const 
	{
		return *this / length();
	} 

	SimdScalar angle(const btSimdQuaternion& q) const 
	{
		SimdScalar s = SimdSqrt(length2() * q.length2());
		assert(s != SimdScalar(0.0));
		return SimdAcos(dot(q) / s);
	}

	SimdScalar getAngle() const 
	{
		SimdScalar s = 2.f * SimdAcos(m_unusedW);
		return s;
	}



	btSimdQuaternion inverse() const
	{
		return btSimdQuaternion(m_x, m_y, m_z, -m_unusedW);
	}

	SIMD_FORCE_INLINE btSimdQuaternion
	operator+(const btSimdQuaternion& q2) const
	{
		const btSimdQuaternion& q1 = *this;
		return btSimdQuaternion(q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z(), q1[3] + q2[3]);
	}

	SIMD_FORCE_INLINE btSimdQuaternion
	operator-(const btSimdQuaternion& q2) const
	{
		const btSimdQuaternion& q1 = *this;
		return btSimdQuaternion(q1.x() - q2.x(), q1.y() - q2.y(), q1.z() - q2.z(), q1[3] - q2[3]);
	}

	SIMD_FORCE_INLINE btSimdQuaternion operator-() const
	{
		const btSimdQuaternion& q2 = *this;
		return btSimdQuaternion( - q2.x(), - q2.y(),  - q2.z(),  - q2[3]);
	}

	SIMD_FORCE_INLINE btSimdQuaternion farthest( const btSimdQuaternion& qd) const 
	{
		btSimdQuaternion diff,sum;
		diff = *this - qd;
		sum = *this + qd;
		if( diff.dot(diff) > sum.dot(sum) )
			return qd;
		return (-qd);
	}

	btSimdQuaternion slerp(const btSimdQuaternion& q, const SimdScalar& t) const
	{
		SimdScalar theta = angle(q);
		if (theta != SimdScalar(0.0))
		{
			SimdScalar d = SimdScalar(1.0) / SimdSin(theta);
			SimdScalar s0 = SimdSin((SimdScalar(1.0) - t) * theta);
			SimdScalar s1 = SimdSin(t * theta);   
			return btSimdQuaternion((m_x * s0 + q.x() * s1) * d,
				(m_y * s0 + q.y() * s1) * d,
				(m_z * s0 + q.z() * s1) * d,
				(m_unusedW * s0 + q[3] * s1) * d);
		}
		else
		{
			return *this;
		}
	}

	

};



SIMD_FORCE_INLINE btSimdQuaternion
operator-(const btSimdQuaternion& q)
{
	return btSimdQuaternion(-q.x(), -q.y(), -q.z(), -q[3]);
}




SIMD_FORCE_INLINE btSimdQuaternion
operator*(const btSimdQuaternion& q1, const btSimdQuaternion& q2) {
	return btSimdQuaternion(q1[3] * q2.x() + q1.x() * q2[3] + q1.y() * q2.z() - q1.z() * q2.y(),
		q1[3] * q2.y() + q1.y() * q2[3] + q1.z() * q2.x() - q1.x() * q2.z(),
		q1[3] * q2.z() + q1.z() * q2[3] + q1.x() * q2.y() - q1.y() * q2.x(),
		q1[3] * q2[3] - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z()); 
}

SIMD_FORCE_INLINE btSimdQuaternion
operator*(const btSimdQuaternion& q, const btSimdVector3& w)
{
	return btSimdQuaternion( q[3] * w.x() + q.y() * w.z() - q.z() * w.y(),
		q[3] * w.y() + q.z() * w.x() - q.x() * w.z(),
		q[3] * w.z() + q.x() * w.y() - q.y() * w.x(),
		-q.x() * w.x() - q.y() * w.y() - q.z() * w.z()); 
}

SIMD_FORCE_INLINE btSimdQuaternion
operator*(const btSimdVector3& w, const btSimdQuaternion& q)
{
	return btSimdQuaternion( w.x() * q[3] + w.y() * q.z() - w.z() * q.y(),
		w.y() * q[3] + w.z() * q.x() - w.x() * q.z(),
		w.z() * q[3] + w.x() * q.y() - w.y() * q.x(),
		-w.x() * q.x() - w.y() * q.y() - w.z() * q.z()); 
}

SIMD_FORCE_INLINE SimdScalar 
dot(const btSimdQuaternion& q1, const btSimdQuaternion& q2) 
{ 
	return q1.dot(q2); 
}


SIMD_FORCE_INLINE SimdScalar
length(const btSimdQuaternion& q) 
{ 
	return q.length(); 
}

SIMD_FORCE_INLINE SimdScalar
angle(const btSimdQuaternion& q1, const btSimdQuaternion& q2) 
{ 
	return q1.angle(q2); 
}


SIMD_FORCE_INLINE btSimdQuaternion
inverse(const btSimdQuaternion& q) 
{
	return q.inverse();
}

SIMD_FORCE_INLINE btSimdQuaternion
slerp(const btSimdQuaternion& q1, const btSimdQuaternion& q2, const SimdScalar& t) 
{
	return q1.slerp(q2, t);
}


#endif



