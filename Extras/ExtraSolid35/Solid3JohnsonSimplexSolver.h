/*
 * SOLID - Software Library for Interference Detection
 * 
 * Copyright (C) 2001-2003  Dtecta.  All rights reserved.
 *
 * This library may be distributed under the terms of the Q Public License
 * (QPL) as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE.QPL included in the packaging of this file.
 *
 * This library may be distributed and/or modified under the terms of the
 * GNU bteral Public License (GPL) version 2 as published by the Free Software
 * Foundation and appearing in the file LICENSE.GPL included in the
 * packaging of this file.
 *
 * This library is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Commercial use or any other use of this library not covered by either 
 * the QPL or the GPL requires an additional license from Dtecta. 
 * Please contact info@dtecta.com for enquiries about the terms of commercial
 * use of this library.
 */

#ifndef SOLID3JOHNSON_SIMPLEX_SOLVER_H
#define SOLID3JOHNSON_SIMPLEX_SOLVER_H

#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"

//#define JOHNSON_ROBUST


/// Solid3JohnsonSimplexSolver contains Johnson subdistance algorithm from Solid 3.5 library
class Solid3JohnsonSimplexSolver : public btSimplexSolverInterface
{

private:
	typedef unsigned int T_Bits;
	inline static bool subseteq(T_Bits a, T_Bits b) { return (a & b) == a; }
	inline static bool contains(T_Bits a, T_Bits b) { return (a & b) != 0x0; }

	void update_cache();
	void compute_det();
	bool valid(T_Bits s);
	bool proper(T_Bits s);
	void compute_vector(T_Bits s, btVector3& v);


	btScalar	m_det[16][4]; // cached sub-determinants
    btVector3	m_edge[4][4];

#ifdef JOHNSON_ROBUST
    btScalar	m_norm[4][4];
#endif

	btPoint3	m_p[4];    // support points of object A in local coordinates 
	btPoint3	m_q[4];    // support points of object B in local coordinates 
	btVector3	m_y[4];   // support points of A - B in world coordinates
	btScalar	m_ylen2[4];   // Squared lengths support points y

	btScalar	m_maxlen2; // Maximum squared length to a vertex of the current 
	                      // simplex
	T_Bits		m_bits1;      // identifies current simplex
	T_Bits		m_last;      // identifies last found support point
	T_Bits		m_last_bit;  // m_last_bit == 0x1 << last
	T_Bits		m_all_bits;  // m_all_bits == m_bits  | m_last_bit 



private:
	

	void addVertex(const btVector3& w);

public:
	Solid3JohnsonSimplexSolver();

	virtual ~Solid3JohnsonSimplexSolver();

	virtual void reset();

	virtual void addVertex(const btVector3& w, const btPoint3& p, const btPoint3& q);
	
	virtual bool closest(btVector3& v);

	virtual btScalar maxVertex();

	virtual bool fullSimplex() const;

	virtual int getSimplex(btPoint3 *pBuf, btPoint3 *qBuf, btVector3 *yBuf) const;

	virtual bool inSimplex(const btVector3& w);
	
	virtual void backup_closest(btVector3& v) ;

	virtual bool emptySimplex() const ;

	virtual void compute_points(btPoint3& p1, btPoint3& p2) ;

	virtual int numVertices() const ;


};


#endif //SOLID3JOHNSON_SIMPLEX_SOLVER_H
