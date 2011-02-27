#ifndef _btSimdVector3_H
#define _btSimdVector3_H

#include "btSimdVector3.h"

#if defined(_PS3) || defined(__CELLOS_LV2__)
#	define BT_PS3
#endif

ATTRIBUTE_ALIGNED16(class) btSimdVector3
{
public:
#if defined(BT_PS3) && defined(__SPU__)

	btScalar m_floats[4];

	SIMD_FORCE_INLINE const vec_float4&	get128() const
	{
		return *((const vec_float4*)&m_floats[0]);
	}

#elif defined(BT_PS3) && !defined(__SPU__)

	vec_float4 m_floats;

	SIMD_FORCE_INLINE explicit btSimdVector3(vec_float4 floats);

#elif defined(BT_USE_SSE)

	union
	{
		__m128 mVec128;
		btScalar m_floats[4];
	};
	SIMD_FORCE_INLINE __m128 get128() const
	{
		return mVec128;
	}
	SIMD_FORCE_INLINE void set128(__m128 v128)
	{
		mVec128 = v128;
	}

#else
	btScalar m_floats[4];
#endif

	/**@brief No initialization constructor */
	SIMD_FORCE_INLINE btSimdVector3();

	SIMD_FORCE_INLINE btSimdVector3(const btSimdVector3& v);

	SIMD_FORCE_INLINE btSimdVector3(const btVector3& v);
	
	/**@brief Constructor from scalars 
	 * @param x X value
	 * @param y Y value 
	 * @param z Z value 
	 */
	SIMD_FORCE_INLINE btSimdVector3(const btScalar& x, const btScalar& y, const btScalar& z);
	
	/**@brief Add a vector to this one 
	 * @param The vector to add to this one */
	SIMD_FORCE_INLINE btSimdVector3& operator += (const btSimdVector3& v);

	/**@brief Subtract a vector from this one
	 * @param The vector to subtract */
	SIMD_FORCE_INLINE btSimdVector3& operator -= (const btSimdVector3& v);

	SIMD_FORCE_INLINE btSimdVector3& operator *= (const btSimdVector3& v);

	SIMD_FORCE_INLINE btSimdVector3& operator *= (const btScalar& s);

	SIMD_FORCE_INLINE btSimdVector3& operator /= (const btSimdVector3& v);

	SIMD_FORCE_INLINE btSimdVector3& operator /= (const btScalar& s);

	SIMD_FORCE_INLINE btSimdVector3 dot(const btSimdVector3& v) const;

	SIMD_FORCE_INLINE btSimdVector3 length2() const;

	/**@brief Return the length of the vector */
	SIMD_FORCE_INLINE btSimdVector3 length() const;

	/**@brief Return the distance squared between the ends of this and another vector
	 * This is semantically treating the vector like a point */
	SIMD_FORCE_INLINE btSimdVector3 distance2(const btSimdVector3& v) const;

	/**@brief Return the distance between the ends of this and another vector
	 * This is semantically treating the vector like a point */
	SIMD_FORCE_INLINE btSimdVector3 distance(const btSimdVector3& v) const;

	/**@brief Normalize this vector 
	 * x^2 + y^2 + z^2 = 1 */
	SIMD_FORCE_INLINE btSimdVector3& normalize();

	/**@brief Return a normalized version of this vector */
	SIMD_FORCE_INLINE btSimdVector3 normalized() const;

	/**@brief Return a vector will the absolute values of each element */
	SIMD_FORCE_INLINE btSimdVector3 absolute() const;

	/**@brief Return the cross product between this and another vector 
	 * @param v The other vector */
	SIMD_FORCE_INLINE btSimdVector3 cross(const btSimdVector3& v) const;

	/**@brief Return the axis with the smallest value 
	 * Note return values are 0,1,2 for x, y, or z */
	SIMD_FORCE_INLINE int minAxis() const;

	/**@brief Return the axis with the largest value 
	 * Note return values are 0,1,2 for x, y, or z */
	SIMD_FORCE_INLINE int maxAxis() const;

	SIMD_FORCE_INLINE int furthestAxis() const;

	SIMD_FORCE_INLINE int closestAxis() const;

	SIMD_FORCE_INLINE void setInterpolate3(const btSimdVector3& v0, const btSimdVector3& v1, btScalar rt);

	/**@brief Return the linear interpolation between this and another vector 
	 * @param v The other vector 
	 * @param t The ration of this to v (t = 0 => return this, t=1 => return other) */
	SIMD_FORCE_INLINE btSimdVector3 lerp(const btSimdVector3& v, const btScalar& t) const;

	/**@brief Return the x value */
	SIMD_FORCE_INLINE const btScalar& getX() const;

	/**@brief Return the y value */
	SIMD_FORCE_INLINE const btScalar& getY() const;

	/**@brief Return the z value */
	SIMD_FORCE_INLINE const btScalar& getZ() const;

	/**@brief Set the x value */
	SIMD_FORCE_INLINE void setX(btScalar x);

	/**@brief Set the y value */
	SIMD_FORCE_INLINE void setY(btScalar y);

	/**@brief Set the z value */
	SIMD_FORCE_INLINE void setZ(btScalar z);

	/**@brief Set the w value */
	SIMD_FORCE_INLINE void setW(btScalar w);

	/**@brief Return the x value */
	SIMD_FORCE_INLINE const btScalar& x() const;

	/**@brief Return the y value */
	SIMD_FORCE_INLINE const btScalar& y() const;

	/**@brief Return the z value */
	SIMD_FORCE_INLINE const btScalar& z() const;

	/**@brief Return the w value */
	SIMD_FORCE_INLINE const btScalar& w() const;

	SIMD_FORCE_INLINE bool operator == (const btSimdVector3& other) const;

	SIMD_FORCE_INLINE bool operator != (const btSimdVector3& other) const;

	/**@brief Set each element to the max of the current values and the values of another btSimdVector3
	 * @param other The other btSimdVector3 to compare with 
	 */
	SIMD_FORCE_INLINE void setMax(const btSimdVector3& other);

	/**@brief Set each element to the min of the current values and the values of another btSimdVector3
	 * @param other The other btSimdVector3 to compare with 
	 */
	SIMD_FORCE_INLINE void setMin(const btSimdVector3& other);

	SIMD_FORCE_INLINE void setValue(const btScalar& x, const btScalar& y, const btScalar& z);

	SIMD_FORCE_INLINE void setElement(int element, const btScalar& value);

	SIMD_FORCE_INLINE void setZero();

	SIMD_FORCE_INLINE bool isZero() const;

	SIMD_FORCE_INLINE btVector3 asVector3() const;
};

SIMD_FORCE_INLINE btSimdVector3 operator + (const btSimdVector3& v1, const btSimdVector3& v2);

SIMD_FORCE_INLINE btSimdVector3 operator * (const btSimdVector3& v1, const btSimdVector3& v2);

SIMD_FORCE_INLINE btSimdVector3 operator - (const btSimdVector3& v1, const btSimdVector3& v2);

SIMD_FORCE_INLINE btSimdVector3 operator - (const btSimdVector3& v);

SIMD_FORCE_INLINE btSimdVector3 operator * (const btSimdVector3& v, const btScalar& s);

SIMD_FORCE_INLINE btSimdVector3 operator * (const btScalar& s, const btSimdVector3& v);

SIMD_FORCE_INLINE btSimdVector3 operator / (const btSimdVector3& v, const btScalar& s);

SIMD_FORCE_INLINE btSimdVector3 operator / (const btSimdVector3& v1, const btSimdVector3& v2);

SIMD_FORCE_INLINE btSimdVector3 btDot(const btSimdVector3& v1, const btSimdVector3& v2);

SIMD_FORCE_INLINE btSimdVector3 btDistance2(const btSimdVector3& v1, const btSimdVector3& v2);

SIMD_FORCE_INLINE btSimdVector3 btDistance(const btSimdVector3& v1, const btSimdVector3& v2);

SIMD_FORCE_INLINE btSimdVector3 btCross(const btSimdVector3& v1, const btSimdVector3& v2);

SIMD_FORCE_INLINE btSimdVector3 lerp(const btSimdVector3& v1, const btSimdVector3& v2, const btScalar& t);

SIMD_FORCE_INLINE btSimdVector3 recipSquareRoot(const btSimdVector3& v);

// Assume all elements in vector are equal thus able to decide most efficient implementation.
SIMD_FORCE_INLINE bool compareSingleLess(const btSimdVector3& v1, const btSimdVector3& v2);

// Assume all elements in vector are equal thus able to decide most efficient implementation.
SIMD_FORCE_INLINE bool compareSingleGreater(const btSimdVector3& v1, const btSimdVector3& v2);

// All elements less than compare.
SIMD_FORCE_INLINE bool compareAllLess(const btSimdVector3& v1, const btSimdVector3& v2);

// All elements greater than compare.
SIMD_FORCE_INLINE bool compareAllGreater(const btSimdVector3& v1, const btSimdVector3& v2);

// Vector select.
SIMD_FORCE_INLINE btSimdVector3 select(const btSimdVector3& s, const btSimdVector3& ltz, const btSimdVector3& gtz);

#if defined(BT_PS3) && !defined(__SPU__)
#	include "LinearMath/PS3/btSimdVector3.inl"
#else
#	include "LinearMath/Ansi/btSimdVector3.inl"
#endif

#endif //_btSimdVector3_H
