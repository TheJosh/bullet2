
MSTRINGIFY(



typedef struct 
{
	float4		m_deltaLinearVelocity;
	float4		m_deltaAngularVelocity;
	float4		m_angularFactor;
	float4		m_invMass;
	float		m_friction;
	void*		m_originalBody;
	float m_pad[2];  // alignment safety
	float4		m_pushVelocity;
	float4		m_turnVelocity;
} btBCSOCLSolverBody;

typedef struct
{
	float4		m_relpos1CrossNormal;
	float4		m_contactNormal;

	float4		m_relpos2CrossNormal;
	//btVector3		m_contactNormal2;//usually m_contactNormal2 == -m_contactNormal

	float4		m_angularComponentA;
	float4		m_angularComponentB;
	
	float4	m_appliedPushImpulse;
	float4	m_appliedImpulse;
	
	
	float	m_friction;
	float	m_jacDiagABInv;
	int	m_numConsecutiveRowsPerKernel;
	int			m_frictionIndex;
	int			m_solverBodyIdA;
	int			m_solverBodyIdB;
	void*		m_originalContactPoint;

	float		m_rhs;
	float		m_cfm;
	float		m_lowerLimit;
	float		m_upperLimit;

	float	m_rhsPenetration;

} btBCSOCLSolverConstraint;

///*
__kernel void kSolveConstraintRow(	int numConstrInBatch, 
									__global btBCSOCLSolverBody* pBodies, 
									__global btBCSOCLSolverConstraint* pConstraints,
									__global int*	pConstrIdx,
									int				constrOffs GUID_ARG)
{
	__local float4 deltaLinearVelocity1[128];
	__local float4 deltaAngularVelocity1[128]; 
	__local float4 deltaLinearVelocity2[128];
	__local float4 deltaAngularVelocity2[128]; 
    int index = get_global_id(0);
    if(index >= numConstrInBatch)
	{
		return;
	}
	int lidx = get_local_id(0);
	int currConstrIndex = pConstrIdx[constrOffs + index];
	int numRows = pConstraints[currConstrIndex].m_numConsecutiveRowsPerKernel;
	__global btBCSOCLSolverBody* pBody1 = pBodies + pConstraints[currConstrIndex].m_solverBodyIdA;
	__global btBCSOCLSolverBody* pBody2 = pBodies + pConstraints[currConstrIndex].m_solverBodyIdB;
	
	deltaLinearVelocity1[lidx] = pBody1->m_deltaLinearVelocity;
	deltaAngularVelocity1[lidx] = pBody1->m_deltaAngularVelocity; 
	deltaLinearVelocity2[lidx] = pBody2->m_deltaLinearVelocity;
	deltaAngularVelocity2[lidx] = pBody2->m_deltaAngularVelocity; 
	
	for(int nRow = 0; nRow < numRows; nRow++)
	{
		__global btBCSOCLSolverConstraint* pCt = &pConstraints[currConstrIndex + nRow];
		float4 contactNormal = pCt->m_contactNormal;
		float4 relpos1CrossNormal = pCt->m_relpos1CrossNormal;
		float4 relpos2CrossNormal = pCt->m_relpos2CrossNormal;
		float jacDiagABInv = pCt->m_jacDiagABInv;
		
		float deltaImpulse = pCt->m_rhs - pCt->m_appliedImpulse.x * pCt->m_cfm;
		float deltaVel1Dotn	=	contactNormal.x * deltaLinearVelocity1[lidx].x 
							  + contactNormal.y * deltaLinearVelocity1[lidx].y 
							  + contactNormal.z * deltaLinearVelocity1[lidx].z 
							  + relpos1CrossNormal.x * deltaAngularVelocity1[lidx].x
							  + relpos1CrossNormal.y * deltaAngularVelocity1[lidx].y
							  + relpos1CrossNormal.z * deltaAngularVelocity1[lidx].z;
		float deltaVel2Dotn	= - contactNormal.x * deltaLinearVelocity2[lidx].x 
							  - contactNormal.y * deltaLinearVelocity2[lidx].y 
							  - contactNormal.z * deltaLinearVelocity2[lidx].z 
							  + relpos2CrossNormal.x * deltaAngularVelocity2[lidx].x
							  + relpos2CrossNormal.y * deltaAngularVelocity2[lidx].y
							  + relpos2CrossNormal.z * deltaAngularVelocity2[lidx].z;

		deltaImpulse	-=	deltaVel1Dotn * jacDiagABInv;
		deltaImpulse	-=	deltaVel2Dotn * jacDiagABInv;
		float sum = pCt->m_appliedImpulse.x + deltaImpulse;
		if (sum < pCt->m_lowerLimit)
		{
			deltaImpulse = pCt->m_lowerLimit - pCt->m_appliedImpulse.x;
			pCt->m_appliedImpulse.x = pCt->m_lowerLimit;
		}
		else if (sum > pCt->m_upperLimit) 
		{
			deltaImpulse = pCt->m_upperLimit - pCt->m_appliedImpulse.x;
			pCt->m_appliedImpulse.x = pCt->m_upperLimit;
		}
		else
		{
			pCt->m_appliedImpulse.x = sum;
		}
		deltaLinearVelocity1[lidx] +=  deltaImpulse * pCt->m_contactNormal * pBody1->m_invMass;
		deltaAngularVelocity1[lidx] += pCt->m_angularComponentA * (deltaImpulse * pBody1->m_angularFactor);
		deltaLinearVelocity2[lidx] += -1.0f * pCt->m_contactNormal * pBody2->m_invMass * deltaImpulse;
		deltaAngularVelocity2[lidx] +=  pCt->m_angularComponentB * (deltaImpulse * pBody2->m_angularFactor);
	}
	pBody1->m_deltaLinearVelocity = deltaLinearVelocity1[lidx];
	pBody1->m_deltaAngularVelocity = deltaAngularVelocity1[lidx];
	pBody2->m_deltaLinearVelocity = deltaLinearVelocity2[lidx];
	pBody2->m_deltaAngularVelocity = deltaAngularVelocity2[lidx];
}
//*/


/*
__kernel void kSolveConstraintRow(	int numConstrInBatch, 
									__global btBCSOCLSolverBody* pBodies, 
									__global btBCSOCLSolverConstraint* pConstraints,
									__global int*	pConstrIdx,
									int				constrOffs GUID_ARG)
{
	__local float4 locStore[65536];
    int index = get_global_id(0);
    if(index >= numConstrInBatch)
	{
		return;
	}
	int currConstrIndex = pConstrIdx[constrOffs + index];
	int numRows = pConstraints[currConstrIndex].m_numConsecutiveRowsPerKernel;
	for(int nRow = 0; nRow < numRows; nRow++)
	{
		__global btBCSOCLSolverConstraint* pCt = &pConstraints[currConstrIndex + nRow];
		__global btBCSOCLSolverBody* pBody1 = pBodies + pCt->m_solverBodyIdA;
		__global btBCSOCLSolverBody* pBody2 = pBodies + pCt->m_solverBodyIdB;
		float deltaImpulse = pCt->m_rhs - pCt->m_appliedImpulse.x * pCt->m_cfm;
		float deltaVel1Dotn	=	pCt->m_contactNormal.x * pBody1->m_deltaLinearVelocity.x 
							  + pCt->m_contactNormal.y * pBody1->m_deltaLinearVelocity.y 
							  + pCt->m_contactNormal.z * pBody1->m_deltaLinearVelocity.z 
							  + pCt->m_relpos1CrossNormal.x * pBody1->m_deltaAngularVelocity.x
							  + pCt->m_relpos1CrossNormal.y * pBody1->m_deltaAngularVelocity.y
							  + pCt->m_relpos1CrossNormal.z * pBody1->m_deltaAngularVelocity.z;
		float deltaVel2Dotn	= - pCt->m_contactNormal.x * pBody2->m_deltaLinearVelocity.x 
							  - pCt->m_contactNormal.y * pBody2->m_deltaLinearVelocity.y 
							  - pCt->m_contactNormal.z * pBody2->m_deltaLinearVelocity.z 
							  + pCt->m_relpos2CrossNormal.x * pBody2->m_deltaAngularVelocity.x
							  + pCt->m_relpos2CrossNormal.y * pBody2->m_deltaAngularVelocity.y
							  + pCt->m_relpos2CrossNormal.z * pBody2->m_deltaAngularVelocity.z;

		deltaImpulse	-=	deltaVel1Dotn * pCt->m_jacDiagABInv;
		deltaImpulse	-=	deltaVel2Dotn * pCt->m_jacDiagABInv;
		float sum = pCt->m_appliedImpulse.x + deltaImpulse;
		if (sum < pCt->m_lowerLimit)
		{
			deltaImpulse = pCt->m_lowerLimit - pCt->m_appliedImpulse.x;
			pCt->m_appliedImpulse.x = pCt->m_lowerLimit;
		}
		else if (sum > pCt->m_upperLimit) 
		{
			deltaImpulse = pCt->m_upperLimit - pCt->m_appliedImpulse.x;
			pCt->m_appliedImpulse.x = pCt->m_upperLimit;
		}
		else
		{
			pCt->m_appliedImpulse.x = sum;
		}
		if (pBody1->m_invMass.x > 0.f)
		{
			pBody1->m_deltaLinearVelocity +=  deltaImpulse * pCt->m_contactNormal * pBody1->m_invMass;
			pBody1->m_deltaAngularVelocity += pCt->m_angularComponentA * (deltaImpulse * pBody1->m_angularFactor);
		}
		if (pBody2->m_invMass.x > 0.f)
		{
			pBody2->m_deltaLinearVelocity += -1.0f * pCt->m_contactNormal * pBody2->m_invMass * deltaImpulse;
			pBody2->m_deltaAngularVelocity +=  pCt->m_angularComponentB * (deltaImpulse * pBody2->m_angularFactor);
		}
	}
}
*/	

__kernel void kSetupFrictionConstraint(	int numFrictionConstr, 
										__global btBCSOCLSolverConstraint* pConstraints,
										int frictionConstrOffs,
										int contactConstrOffs GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numFrictionConstr)
    {
		return;
	}
	__global btBCSOCLSolverConstraint* pFrictCt = &pConstraints[frictionConstrOffs + index];
	int contIndex = pFrictCt->m_frictionIndex;
	__global btBCSOCLSolverConstraint* pContCt = &pConstraints[contactConstrOffs + contIndex];
	float totalImpulse = pContCt->m_appliedImpulse.x;
	if (totalImpulse > 0.f)
	{
		pFrictCt->m_lowerLimit = -pFrictCt->m_friction * totalImpulse;
		pFrictCt->m_upperLimit =  pFrictCt->m_friction * totalImpulse;
	}
}


__kernel void kSolveConstrRowLowLim(int numConstrInBatch, 
									__global btBCSOCLSolverBody* pBodies, 
									__global btBCSOCLSolverConstraint* pConstraints,
									__global int*	pConstrIdx,
									int				constrOffs GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numConstrInBatch)
	{
		return;
	}
	int currConstrIndex = pConstrIdx[constrOffs + index];
	int numRows = pConstraints[currConstrIndex].m_numConsecutiveRowsPerKernel;
	for(int nRow = 0; nRow < numRows; nRow++)
	{
		__global btBCSOCLSolverConstraint* pCt = &pConstraints[currConstrIndex + nRow];
		__global btBCSOCLSolverBody* pBody1 = pBodies + pCt->m_solverBodyIdA;
		__global btBCSOCLSolverBody* pBody2 = pBodies + pCt->m_solverBodyIdB;
	///*	
		float deltaImpulse = pCt->m_rhs - pCt->m_appliedImpulse.x * pCt->m_cfm;
		float deltaVel1Dotn	=	pCt->m_contactNormal.x * pBody1->m_deltaLinearVelocity.x 
							  + pCt->m_contactNormal.y * pBody1->m_deltaLinearVelocity.y 
							  + pCt->m_contactNormal.z * pBody1->m_deltaLinearVelocity.z 
							  + pCt->m_relpos1CrossNormal.x * pBody1->m_deltaAngularVelocity.x
							  + pCt->m_relpos1CrossNormal.y * pBody1->m_deltaAngularVelocity.y
							  + pCt->m_relpos1CrossNormal.z * pBody1->m_deltaAngularVelocity.z;
		float deltaVel2Dotn	= - pCt->m_contactNormal.x * pBody2->m_deltaLinearVelocity.x 
							  - pCt->m_contactNormal.y * pBody2->m_deltaLinearVelocity.y 
							  - pCt->m_contactNormal.z * pBody2->m_deltaLinearVelocity.z 
							  + pCt->m_relpos2CrossNormal.x * pBody2->m_deltaAngularVelocity.x
							  + pCt->m_relpos2CrossNormal.y * pBody2->m_deltaAngularVelocity.y
							  + pCt->m_relpos2CrossNormal.z * pBody2->m_deltaAngularVelocity.z;

		deltaImpulse	-=	deltaVel1Dotn * pCt->m_jacDiagABInv;
		deltaImpulse	-=	deltaVel2Dotn * pCt->m_jacDiagABInv;
		float sum = pCt->m_appliedImpulse.x + deltaImpulse;
		if (sum < pCt->m_lowerLimit)
		{
			deltaImpulse = pCt->m_lowerLimit - pCt->m_appliedImpulse.x;
			pCt->m_appliedImpulse.x = pCt->m_lowerLimit;
		}
		else
		{
			pCt->m_appliedImpulse.x = sum;
		}
		if (pBody1->m_invMass.x > 0.f)
		{
			pBody1->m_deltaLinearVelocity +=  deltaImpulse * pCt->m_contactNormal * pBody1->m_invMass;
			pBody1->m_deltaAngularVelocity += pCt->m_angularComponentA * (deltaImpulse * pBody1->m_angularFactor);
		}
		if (pBody2->m_invMass.x > 0.f)
		{
			pBody2->m_deltaLinearVelocity += -1.0f * pCt->m_contactNormal * pBody2->m_invMass * deltaImpulse;
			pBody2->m_deltaAngularVelocity +=  pCt->m_angularComponentB * (deltaImpulse * pBody2->m_angularFactor);
		}
	}
//*/	
}




);