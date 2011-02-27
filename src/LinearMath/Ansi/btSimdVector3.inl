
SIMD_FORCE_INLINE btSimdVector3::btSimdVector3()
{
}

SIMD_FORCE_INLINE btSimdVector3::btSimdVector3(const btSimdVector3& v)
{
	m_floats[0] = v.m_floats[0];
	m_floats[1] = v.m_floats[1];
	m_floats[2] = v.m_floats[2];
	m_floats[3] = btScalar(0.);
}

SIMD_FORCE_INLINE btSimdVector3::btSimdVector3(const btVector3& v)
{
	m_floats[0] = v.m_floats[0];
	m_floats[1] = v.m_floats[1];
	m_floats[2] = v.m_floats[2];
	m_floats[3] = btScalar(0.);
}

SIMD_FORCE_INLINE btSimdVector3::btSimdVector3(const btScalar& x, const btScalar& y, const btScalar& z)
{
	m_floats[0] = x;
	m_floats[1] = y;
	m_floats[2] = z;
	m_floats[3] = btScalar(0.);
}

SIMD_FORCE_INLINE btSimdVector3& btSimdVector3::operator += (const btSimdVector3& v)
{
	m_floats[0] += v.m_floats[0]; m_floats[1] += v.m_floats[1]; m_floats[2] += v.m_floats[2];
	return *this;
}

SIMD_FORCE_INLINE btSimdVector3& btSimdVector3::operator -= (const btSimdVector3& v) 
{
	m_floats[0] -= v.m_floats[0]; m_floats[1] -= v.m_floats[1]; m_floats[2] -= v.m_floats[2];
	return *this;
}

SIMD_FORCE_INLINE btSimdVector3& btSimdVector3::operator *= (const btSimdVector3& v)
{
	m_floats[0] *= v.m_floats[0]; m_floats[1] *= v.m_floats[1]; m_floats[2] *= v.m_floats[2];
	return *this;
}

SIMD_FORCE_INLINE btSimdVector3& btSimdVector3::operator *= (const btScalar& s)
{
	m_floats[0] *= s; m_floats[1] *= s;m_floats[2] *= s;
	return *this;
}

SIMD_FORCE_INLINE btSimdVector3& btSimdVector3::operator /= (const btSimdVector3& v)
{
	m_floats[0] /= v.m_floats[0]; m_floats[1] /= v.m_floats[1]; m_floats[2] /= v.m_floats[2];
	return *this;
}

SIMD_FORCE_INLINE btSimdVector3& btSimdVector3::operator /= (const btScalar& s) 
{
	btFullAssert(s != btScalar(0.0));
	return *this *= btScalar(1.0) / s;
}

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::dot(const btSimdVector3& v) const
{
	float d = m_floats[0] * v.m_floats[0] + m_floats[1] * v.m_floats[1] + m_floats[2] * v.m_floats[2];
	return btSimdVector3(d, d, d);
}

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::length2() const
{
	return dot(*this);
}

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::length() const
{
	float ln = btSqrt(m_floats[0] * m_floats[0] + m_floats[1] * m_floats[1] + m_floats[2] * m_floats[2]);
	return btSimdVector3(ln, ln, ln);
}

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::distance2(const btSimdVector3& v) const
{
	return (v - *this).length2();
}

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::distance(const btSimdVector3& v) const
{
	return (v - *this).length();
}

SIMD_FORCE_INLINE btSimdVector3& btSimdVector3::normalize()
{
	return *this /= length();
}

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::normalized() const
{
	return *this / length();
} 

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::absolute() const 
{
	return btSimdVector3(
		btFabs(m_floats[0]), 
		btFabs(m_floats[1]), 
		btFabs(m_floats[2])
	);
}

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::cross(const btSimdVector3& v) const
{
	return btSimdVector3(
		m_floats[1] * v.m_floats[2] - m_floats[2] * v.m_floats[1],
		m_floats[2] * v.m_floats[0] - m_floats[0] * v.m_floats[2],
		m_floats[0] * v.m_floats[1] - m_floats[1] * v.m_floats[0]
	);
}

SIMD_FORCE_INLINE int btSimdVector3::minAxis() const
{
	return m_floats[0] < m_floats[1] ? (m_floats[0] <m_floats[2] ? 0 : 2) : (m_floats[1] <m_floats[2] ? 1 : 2);
}

SIMD_FORCE_INLINE int btSimdVector3::maxAxis() const 
{
	return m_floats[0] < m_floats[1] ? (m_floats[1] <m_floats[2] ? 2 : 1) : (m_floats[0] <m_floats[2] ? 2 : 0);
}

SIMD_FORCE_INLINE int btSimdVector3::furthestAxis() const
{
	return absolute().minAxis();
}

SIMD_FORCE_INLINE int btSimdVector3::closestAxis() const 
{
	return absolute().maxAxis();
}

SIMD_FORCE_INLINE void btSimdVector3::setInterpolate3(const btSimdVector3& v0, const btSimdVector3& v1, btScalar rt)
{
	btScalar s = btScalar(1.0) - rt;
	m_floats[0] = s * v0.m_floats[0] + rt * v1.m_floats[0];
	m_floats[1] = s * v0.m_floats[1] + rt * v1.m_floats[1];
	m_floats[2] = s * v0.m_floats[2] + rt * v1.m_floats[2];
}

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::lerp(const btSimdVector3& v, const btScalar& t) const 
{
	return btSimdVector3(
		m_floats[0] + (v.m_floats[0] - m_floats[0]) * t,
		m_floats[1] + (v.m_floats[1] - m_floats[1]) * t,
		m_floats[2] + (v.m_floats[2] - m_floats[2]) * t
	);
}

SIMD_FORCE_INLINE const btScalar& btSimdVector3::getX() const
{
	return m_floats[0];
}

SIMD_FORCE_INLINE const btScalar& btSimdVector3::getY() const
{
	return m_floats[1];
}

SIMD_FORCE_INLINE const btScalar& btSimdVector3::getZ() const
{
	return m_floats[2];
}

SIMD_FORCE_INLINE void btSimdVector3::setX(btScalar x)
{
	m_floats[0] = x;
}

SIMD_FORCE_INLINE void btSimdVector3::setY(btScalar y)
{
	m_floats[1] = y;
}

SIMD_FORCE_INLINE void btSimdVector3::setZ(btScalar z)
{
	m_floats[2] = z;
}

SIMD_FORCE_INLINE void btSimdVector3::setW(btScalar w)
{
	m_floats[3] = w;
}

SIMD_FORCE_INLINE const btScalar& btSimdVector3::x() const
{
	return m_floats[0];
}

SIMD_FORCE_INLINE const btScalar& btSimdVector3::y() const
{
	return m_floats[1];
}

SIMD_FORCE_INLINE const btScalar& btSimdVector3::z() const
{
	return m_floats[2];
}

SIMD_FORCE_INLINE const btScalar& btSimdVector3::w() const
{
	return m_floats[3];
}

SIMD_FORCE_INLINE bool btSimdVector3::operator == (const btSimdVector3& other) const
{
	return ((m_floats[3]==other.m_floats[3]) && (m_floats[2]==other.m_floats[2]) && (m_floats[1]==other.m_floats[1]) && (m_floats[0]==other.m_floats[0]));
}

SIMD_FORCE_INLINE bool btSimdVector3::operator != (const btSimdVector3& other) const
{
	return !(*this == other);
}

SIMD_FORCE_INLINE void btSimdVector3::setMax(const btSimdVector3& other)
{
	btSetMax(m_floats[0], other.m_floats[0]);
	btSetMax(m_floats[1], other.m_floats[1]);
	btSetMax(m_floats[2], other.m_floats[2]);
	btSetMax(m_floats[3], other.w());
}

SIMD_FORCE_INLINE void btSimdVector3::setMin(const btSimdVector3& other)
{
	btSetMin(m_floats[0], other.m_floats[0]);
	btSetMin(m_floats[1], other.m_floats[1]);
	btSetMin(m_floats[2], other.m_floats[2]);
	btSetMin(m_floats[3], other.w());
}

SIMD_FORCE_INLINE void btSimdVector3::setValue(const btScalar& x, const btScalar& y, const btScalar& z)
{
	m_floats[0]=x;
	m_floats[1]=y;
	m_floats[2]=z;
	m_floats[3] = btScalar(0.);
}

SIMD_FORCE_INLINE void btSimdVector3::setElement(int element, const btScalar& value)
{
	m_floats[element] = value;
}

SIMD_FORCE_INLINE void btSimdVector3::setZero()
{
	setValue(btScalar(0.),btScalar(0.),btScalar(0.));
}

SIMD_FORCE_INLINE bool btSimdVector3::isZero() const
{
	return m_floats[0] == btScalar(0) && m_floats[1] == btScalar(0) && m_floats[2] == btScalar(0);
}

SIMD_FORCE_INLINE btVector3 btSimdVector3::asVector3() const
{
	return btVector3(m_floats[0], m_floats[1], m_floats[2]);
}

/**@brief Return the sum of two vectors (Point symantics) */
SIMD_FORCE_INLINE btSimdVector3 operator + (const btSimdVector3& v1, const btSimdVector3& v2) 
{
	return btSimdVector3(v1.m_floats[0] + v2.m_floats[0], v1.m_floats[1] + v2.m_floats[1], v1.m_floats[2] + v2.m_floats[2]);
}

/**@brief Return the elementwise product of two vectors */
SIMD_FORCE_INLINE btSimdVector3 operator * (const btSimdVector3& v1, const btSimdVector3& v2)
{
	return btSimdVector3(v1.m_floats[0] * v2.m_floats[0], v1.m_floats[1] * v2.m_floats[1], v1.m_floats[2] * v2.m_floats[2]);
}

/**@brief Return the difference between two vectors */
SIMD_FORCE_INLINE btSimdVector3 operator - (const btSimdVector3& v1, const btSimdVector3& v2)
{
	return btSimdVector3(v1.m_floats[0] - v2.m_floats[0], v1.m_floats[1] - v2.m_floats[1], v1.m_floats[2] - v2.m_floats[2]);
}
/**@brief Return the negative of the vector */
SIMD_FORCE_INLINE btSimdVector3 operator - (const btSimdVector3& v)
{
	return btSimdVector3(-v.m_floats[0], -v.m_floats[1], -v.m_floats[2]);
}

/**@brief Return the vector scaled by s */
SIMD_FORCE_INLINE btSimdVector3 operator * (const btSimdVector3& v, const btScalar& s)
{
	return btSimdVector3(v.m_floats[0] * s, v.m_floats[1] * s, v.m_floats[2] * s);
}

/**@brief Return the vector scaled by s */
SIMD_FORCE_INLINE btSimdVector3 operator * (const btScalar& s, const btSimdVector3& v)
{ 
	return v * s; 
}

/**@brief Return the vector inversely scaled by s */
SIMD_FORCE_INLINE btSimdVector3 operator / (const btSimdVector3& v, const btScalar& s)
{
	btFullAssert(s != btScalar(0.0));
	return v * (btScalar(1.0) / s);
}

/**@brief Return the vector inversely scaled by s */
SIMD_FORCE_INLINE btSimdVector3 operator / (const btSimdVector3& v1, const btSimdVector3& v2)
{
	return btSimdVector3(v1.m_floats[0] / v2.m_floats[0],v1.m_floats[1] / v2.m_floats[1],v1.m_floats[2] / v2.m_floats[2]);
}

/**@brief Return the dot product between two vectors */
SIMD_FORCE_INLINE btSimdVector3 btDot(const btSimdVector3& v1, const btSimdVector3& v2)
{ 
	return v1.dot(v2); 
}

/**@brief Return the distance squared between two vectors */
SIMD_FORCE_INLINE btSimdVector3 btDistance2(const btSimdVector3& v1, const btSimdVector3& v2)
{ 
	return v1.distance2(v2); 
}

/**@brief Return the distance between two vectors */
SIMD_FORCE_INLINE btSimdVector3 btDistance(const btSimdVector3& v1, const btSimdVector3& v2)
{ 
	return v1.distance(v2); 
}

/**@brief Return the cross product of two vectors */
SIMD_FORCE_INLINE btSimdVector3 btCross(const btSimdVector3& v1, const btSimdVector3& v2)
{ 
	return v1.cross(v2); 
}

/**@brief Return the linear interpolation between two vectors
* @param v1 One vector 
* @param v2 The other vector 
* @param t The ration of this to v (t = 0 => return v1, t=1 => return v2) */
SIMD_FORCE_INLINE btSimdVector3 lerp(const btSimdVector3& v1, const btSimdVector3& v2, const btScalar& t)
{
	return v1.lerp(v2, t);
}

SIMD_FORCE_INLINE btSimdVector3 recipSquareRoot(const btSimdVector3& v)
{
	// Assume all elements equal.
	float r = 1.0f / sqrt(v.m_floats[0]);
	return btSimdVector3(r, r, r);
}

SIMD_FORCE_INLINE bool compareSingleLess(const btSimdVector3& v1, const btSimdVector3& v2)
{
	return v1.m_floats[0] < v2.m_floats[0];
}

SIMD_FORCE_INLINE bool compareSingleGreater(const btSimdVector3& v1, const btSimdVector3& v2)
{
	return v1.m_floats[0] > v2.m_floats[0];
}

SIMD_FORCE_INLINE bool compareAllLess(const btSimdVector3& v1, const btSimdVector3& v2)
{
	return
		v1.m_floats[0] < v2.m_floats[0] &&
		v1.m_floats[1] < v2.m_floats[1] &&
		v1.m_floats[2] < v2.m_floats[2] &&
		v1.m_floats[3] < v2.m_floats[3];
}

SIMD_FORCE_INLINE bool compareAllGreater(const btSimdVector3& v1, const btSimdVector3& v2)
{
	return
		v1.m_floats[0] > v2.m_floats[0] &&
		v1.m_floats[1] > v2.m_floats[1] &&
		v1.m_floats[2] > v2.m_floats[2] &&
		v1.m_floats[3] > v2.m_floats[3];
}

SIMD_FORCE_INLINE btSimdVector3 select(const btSimdVector3& s, const btSimdVector3& ltz, const btSimdVector3& gtz)
{
	return btSimdVector3(
		s.m_floats[0] < 0.0f ? ltz.m_floats[0] : gtz.m_floats[0],
		s.m_floats[1] < 0.0f ? ltz.m_floats[1] : gtz.m_floats[1],
		s.m_floats[2] < 0.0f ? ltz.m_floats[2] : gtz.m_floats[2]
	);
}
