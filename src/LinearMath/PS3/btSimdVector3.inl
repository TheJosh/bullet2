#include <vec_types.h>

namespace
{

SIMD_FORCE_INLINE vec_float4 v_vec_div(vec_float4 divend, vec_float4 denom)
{
	vec_float4 idenom = vec_re(denom);
	vec_float4 res0 = vec_madd(divend, idenom, (vec_float4)(0.0f));
	return vec_madd(
		vec_nmsub(denom, idenom, (vec_float4)(1.0f)),
		res0,
		res0
	);
}

SIMD_FORCE_INLINE vec_float4 v_vec_dot3(vec_float4 v1, vec_float4 v2)
{
	vec_float4 result;
	result = vec_madd(v1, v2, (vec_float4)(0.0f));
	result = vec_madd(vec_sld(v1, v1, 4), vec_sld(v2, v2, 4), result);
	result = vec_madd(vec_sld(v1, v1, 8), vec_sld(v2, v2, 8), result);
	return result;
}

SIMD_FORCE_INLINE vec_float4 v_vec_rsqrte(vec_float4 v)
{
	const static vec_float4 zero = (vec_float4)(0.0f);
	const static vec_float4 half = (vec_float4)(0.5f);
	const static vec_float4 one = (vec_float4)(1.0f);

	vec_float4 estimate = vec_rsqrte(v);
	vec_float4 t0, t1, t2;
	
	t0 = vec_madd(estimate, estimate, zero);	// estimate^2
	t1 = vec_nmsub(v, t0, one);					// 1 - v * estimate^2
	t2 = vec_madd(t1, half, one);				// 0.5 * (1 - v * estimate^2) + 1
	estimate = vec_madd(t2, estimate, zero);	// estimate * (0.5 * (1 - v * estimate^2) + 1)

	//t0 = vec_madd(estimate, estimate, zero);	// estimate^2
	//t1 = vec_nmsub(v, t0, one);					// 1 - v * estimate^2
	//t2 = vec_madd(t1, half, one);				// 0.5 * (1 - v * estimate^2) + 1
	//estimate = vec_madd(t2, estimate, zero);	// estimate * (0.5 * (1 - v * estimate^2) + 1)

	//t0 = vec_madd(estimate, estimate, zero);	// estimate^2
	//t1 = vec_nmsub(v, t0, one);					// 1 - v * estimate^2
	//t2 = vec_madd(t1, half, one);				// 0.5 * (1 - v * estimate^2) + 1
	//estimate = vec_madd(t2, estimate, zero);	// estimate * (0.5 * (1 - v * estimate^2) + 1)

	//t0 = vec_madd(estimate, estimate, zero);	// estimate^2
	//t1 = vec_nmsub(v, t0, one);					// 1 - v * estimate^2
	//t2 = vec_madd(t1, half, one);				// 0.5 * (1 - v * estimate^2) + 1
	//estimate = vec_madd(t2, estimate, zero);	// estimate * (0.5 * (1 - v * estimate^2) + 1)

	return estimate;
}

}

SIMD_FORCE_INLINE btSimdVector3::btSimdVector3(vec_float4 floats)
:	m_floats(floats)
{
}

SIMD_FORCE_INLINE btSimdVector3::btSimdVector3()
{
}

SIMD_FORCE_INLINE btSimdVector3::btSimdVector3(const btSimdVector3& v)
:	m_floats(v.m_floats)
{
}

SIMD_FORCE_INLINE btSimdVector3::btSimdVector3(const btVector3& v)
{
	m_floats = (vec_float4)
	{
		v.m_floats[0],
		v.m_floats[1],
		v.m_floats[2],
		v.m_floats[3]
	};
}

SIMD_FORCE_INLINE btSimdVector3::btSimdVector3(const btScalar& x, const btScalar& y, const btScalar& z)
{
	m_floats = (vec_float4){ x, y, z, 0.0f };
}

SIMD_FORCE_INLINE btSimdVector3& btSimdVector3::operator += (const btSimdVector3& v)
{
	m_floats = vec_add(m_floats, v.m_floats);
	return *this;
}

SIMD_FORCE_INLINE btSimdVector3& btSimdVector3::operator -= (const btSimdVector3& v) 
{
	m_floats = vec_sub(m_floats, v.m_floats);
	return *this;
}

SIMD_FORCE_INLINE btSimdVector3& btSimdVector3::operator *= (const btSimdVector3& v)
{
	m_floats = vec_madd(m_floats, v.m_floats, (vec_float4)(0.0f));
	return *this;
}

SIMD_FORCE_INLINE btSimdVector3& btSimdVector3::operator *= (const btScalar& s)
{
	vec_float4 ss = (vec_float4){ s, s, s, 0.0f };
	m_floats = vec_madd(m_floats, ss, (vec_float4)(0.0f));
	return *this;
}

SIMD_FORCE_INLINE btSimdVector3& btSimdVector3::operator /= (const btSimdVector3& v)
{
	m_floats = v_vec_div(m_floats, v.m_floats);
	return *this;
}

SIMD_FORCE_INLINE btSimdVector3& btSimdVector3::operator /= (const btScalar& s) 
{
	m_floats = v_vec_div(m_floats, (vec_float4){ s, s, s, s });
	return *this;
}

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::dot(const btSimdVector3& v) const
{
	return btSimdVector3(v_vec_dot3(m_floats, v.m_floats));
}

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::length2() const
{
	return btSimdVector3(v_vec_dot3(m_floats, m_floats));
}

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::length() const
{
	vec_float4 rln = v_vec_rsqrte(length2().m_floats);
	return btSimdVector3(v_vec_div((vec_float4)(1.0f), rln));
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
	return btSimdVector3(fabsf4(m_floats));
}

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::cross(const btSimdVector3& v) const
{
#define _VECTORMATH_PERM_X 0x00010203
#define _VECTORMATH_PERM_Y 0x04050607
#define _VECTORMATH_PERM_Z 0x08090a0b
#define _VECTORMATH_PERM_W 0x0c0d0e0f
#define _VECTORMATH_PERM_YZXW (vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Y, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_X, _VECTORMATH_PERM_W }
#define _VECTORMATH_PERM_ZXYW (vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Z, _VECTORMATH_PERM_X, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_W }

	vec_float4 v1 = m_floats;
	vec_float4 v2 = v.m_floats;

	vec_float4 tmp0, tmp1, tmp2, tmp3, result;
	tmp0 = vec_perm(v1, v1, _VECTORMATH_PERM_YZXW);
	tmp1 = vec_perm(v2, v2, _VECTORMATH_PERM_ZXYW);
	tmp2 = vec_perm(v1, v1, _VECTORMATH_PERM_ZXYW);
	tmp3 = vec_perm(v2, v2, _VECTORMATH_PERM_YZXW);
	result = vec_madd(tmp0, tmp1, (vec_float4)(0.0f));
	result = vec_nmsub(tmp2, tmp3, result);

	return btSimdVector3(result);
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
	m_floats = (v0 * s + v1 * rt).m_floats;
}

SIMD_FORCE_INLINE btSimdVector3 btSimdVector3::lerp(const btSimdVector3& v, const btScalar& t) const 
{
	return *this + (v - *this) * t;
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
	const vec_uint4 mask = (vec_uint4){ ~0UL, ~0UL, ~0UL, 0UL };
	vec_uint4 cr = vec_and(vec_cmpeq(m_floats, other.m_floats), mask);
	return cr == mask;
}

SIMD_FORCE_INLINE bool btSimdVector3::operator != (const btSimdVector3& other) const
{
	return !(*this == other);
}

SIMD_FORCE_INLINE void btSimdVector3::setMax(const btSimdVector3& other)
{
	m_floats = vec_max(m_floats, other.m_floats);
}

SIMD_FORCE_INLINE void btSimdVector3::setMin(const btSimdVector3& other)
{
	m_floats = vec_min(m_floats, other.m_floats);
}

SIMD_FORCE_INLINE void btSimdVector3::setValue(const btScalar& x, const btScalar& y, const btScalar& z)
{
	m_floats = (vec_float4){ x, y, z, 0.0f };
}

SIMD_FORCE_INLINE void btSimdVector3::setElement(int element, const btScalar& value)
{
	m_floats[element] = value;
}

SIMD_FORCE_INLINE void btSimdVector3::setZero()
{
	m_floats = (vec_float4)(0.0f);
}

SIMD_FORCE_INLINE bool btSimdVector3::isZero() const
{
	return vec_all_eq(m_floats, (vec_float4)(0.0f)) != 0;
}

SIMD_FORCE_INLINE btVector3 btSimdVector3::asVector3() const
{
	return btVector3(m_floats[0], m_floats[1], m_floats[2]);
}

/**@brief Return the sum of two vectors (Point semantics) */
SIMD_FORCE_INLINE btSimdVector3 operator + (const btSimdVector3& v1, const btSimdVector3& v2) 
{
	return btSimdVector3(vec_add(v1.m_floats, v2.m_floats));
}

/**@brief Return the element wise product of two vectors */
SIMD_FORCE_INLINE btSimdVector3 operator * (const btSimdVector3& v1, const btSimdVector3& v2)
{
	return btSimdVector3(vec_madd(v1.m_floats, v2.m_floats, (vec_float4)(0.0f)));
}

/**@brief Return the difference between two vectors */
SIMD_FORCE_INLINE btSimdVector3 operator - (const btSimdVector3& v1, const btSimdVector3& v2)
{
	return btSimdVector3(vec_sub(v1.m_floats, v2.m_floats));
}
/**@brief Return the negative of the vector */
SIMD_FORCE_INLINE btSimdVector3 operator - (const btSimdVector3& v)
{
	return btSimdVector3(negatef4(v.m_floats));
}

/**@brief Return the vector scaled by s */
SIMD_FORCE_INLINE btSimdVector3 operator * (const btSimdVector3& v, const btScalar& s)
{
	vec_float4 ss = (vec_float4){ s, s, s, 0.0f };
	return btSimdVector3(vec_madd(v.m_floats, ss, (vec_float4)(0.0f)));
}

/**@brief Return the vector scaled by s */
SIMD_FORCE_INLINE btSimdVector3 operator * (const btScalar& s, const btSimdVector3& v)
{ 
	return v * s; 
}

/**@brief Return the vector inversely scaled by s */
SIMD_FORCE_INLINE btSimdVector3 operator / (const btSimdVector3& v, const btScalar& s)
{
	return btSimdVector3(v_vec_div(v.m_floats, (vec_float4){ s, s, s, s }));
}

/**@brief Return the vector inversely scaled by s */
SIMD_FORCE_INLINE btSimdVector3 operator / (const btSimdVector3& v1, const btSimdVector3& v2)
{
	return btSimdVector3(v_vec_div(v1.m_floats, v2.m_floats));
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
	return btSimdVector3(v_vec_rsqrte(v.m_floats));
}

SIMD_FORCE_INLINE bool compareSingleLess(const btSimdVector3& v1, const btSimdVector3& v2)
{
	//return vec_all_lt(v1.m_floats, v2.m_floats) != 0;
	//return vec_vcmpgtuw((vec_uint4)v2.m_floats, (vec_uint4)v1.m_floats) != 0;
	return v1.m_floats[0] < v2.m_floats[0];
}

SIMD_FORCE_INLINE bool compareSingleGreater(const btSimdVector3& v1, const btSimdVector3& v2)
{
	//return vec_all_gt(v1.m_floats, v2.m_floats) != 0;
	//return vec_vcmpgtuw((vec_uint4)v1.m_floats, (vec_uint4)v2.m_floats) != 0;
	return v1.m_floats[0] > v2.m_floats[0];
}

SIMD_FORCE_INLINE bool compareAllLess(const btSimdVector3& v1, const btSimdVector3& v2)
{
	return vec_all_lt(v1.m_floats, v2.m_floats) != 0;
}

SIMD_FORCE_INLINE bool compareAllGreater(const btSimdVector3& v1, const btSimdVector3& v2)
{
	return vec_all_gt(v1.m_floats, v2.m_floats) != 0;
}

SIMD_FORCE_INLINE btSimdVector3 select(const btSimdVector3& s, const btSimdVector3& ltz, const btSimdVector3& gtz)
{
	vec_uint4 mask = (vec_uint4)vec_cmpgt(s.m_floats, (vec_float4)(0.0f));
	return btSimdVector3(vec_sel(ltz.m_floats, gtz.m_floats, mask));
}
