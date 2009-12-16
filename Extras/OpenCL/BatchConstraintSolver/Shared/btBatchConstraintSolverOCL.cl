
MSTRINGIFY(

typedef struct 
{
	float4	m_deltaLinVel_invMass;	//  m_invMass in w
	float4	m_deltaAngVel_frict;	//  m_friction in w
} BCSOCLBody;


typedef struct 	// read-only part of solver constraint (96 bytes)
{
	float4	m_relpos1CrossNormal_jacDiagABInv;
	float4	m_contactNormal_friction;
	float4	m_relpos2CrossNormal_rhs;
	float4	m_angularComponentA_cfm;
	float4	m_angularComponentB;
	int		m_numConsecutiveRowsPerKernel;
	int		m_frictionIndex;
	int		m_solverBodyIdA;
	int		m_solverBodyIdB;
} BCSOCLConstrRO;


typedef struct // read-write part of solver constraint (16 bytes)
{
	float	m_appliedImpulse;
	float	m_lowerLimit;
	float	m_upperLimit;
	void*	m_pOrigData;
} BCSOCLConstrRW;

__kernel void kSolveConstraintRow(	int numConstrInBatch, 
									__global BCSOCLBody* pBodies, 
									__global BCSOCLConstrRO* pConstrRO,
									__global BCSOCLConstrRW* pConstrRW,
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
	int constrStartRow = pConstrIdx[constrOffs + index];
	__global BCSOCLConstrRO* pRO = pConstrRO + constrStartRow;
	__global BCSOCLConstrRW* pRW = pConstrRW + constrStartRow;
	int numRows = pRO->m_numConsecutiveRowsPerKernel;
	__global BCSOCLBody* pBody1 = pBodies + pRO->m_solverBodyIdA;
	__global BCSOCLBody* pBody2 = pBodies + pRO->m_solverBodyIdB;
	
	deltaLinearVelocity1[lidx] = pBody1->m_deltaLinVel_invMass;
	float invMass1 = deltaLinearVelocity1[lidx].w;
	deltaLinearVelocity1[lidx].w = 0.f;
	deltaAngularVelocity1[lidx] = pBody1->m_deltaAngVel_frict; 
	deltaAngularVelocity1[lidx].w = 0.f;
	deltaLinearVelocity2[lidx] = pBody2->m_deltaLinVel_invMass;
	float invMass2 = deltaLinearVelocity2[lidx].w;
	deltaLinearVelocity2[lidx].w = 0.f;
	deltaAngularVelocity2[lidx] = pBody2->m_deltaAngVel_frict; 
	deltaAngularVelocity2[lidx].w = 0.f;
	for(int nRow = 0; nRow < numRows; nRow++, pRO++, pRW++)
	{
		float jacDiagABInv = pRO->m_relpos1CrossNormal_jacDiagABInv.w;
		float deltaImpulse = pRO->m_relpos2CrossNormal_rhs.w - pRW->m_appliedImpulse * pRO->m_angularComponentA_cfm.w;
		float deltaVel1Dotn	=  dot(pRO->m_contactNormal_friction, deltaLinearVelocity1[lidx])
							 + dot(pRO->m_relpos1CrossNormal_jacDiagABInv, deltaAngularVelocity1[lidx]);
		float deltaVel2Dotn	= -dot(pRO->m_contactNormal_friction, deltaLinearVelocity2[lidx])
							 + dot(pRO->m_relpos2CrossNormal_rhs, deltaAngularVelocity2[lidx]);
		deltaImpulse	-=	deltaVel1Dotn * jacDiagABInv;
		deltaImpulse	-=	deltaVel2Dotn * jacDiagABInv;
		float sum = pRW->m_appliedImpulse + deltaImpulse;
		if(sum < pRW->m_lowerLimit)
		{
			deltaImpulse = pRW->m_lowerLimit - pRW->m_appliedImpulse;
			pRW->m_appliedImpulse = pRW->m_lowerLimit;
		}
		else if(sum > pRW->m_upperLimit) 
		{
			deltaImpulse = pRW->m_upperLimit - pRW->m_appliedImpulse;
			pRW->m_appliedImpulse = pRW->m_upperLimit;
		}
		else
		{
			pRW->m_appliedImpulse = sum;
		}
		deltaLinearVelocity1[lidx] +=  deltaImpulse * pRO->m_contactNormal_friction * invMass1;
		deltaAngularVelocity1[lidx] += pRO->m_angularComponentA_cfm * deltaImpulse;
		deltaLinearVelocity2[lidx] += -1.0f * pRO->m_contactNormal_friction * invMass2 * deltaImpulse;
		deltaAngularVelocity2[lidx] +=  pRO->m_angularComponentB * deltaImpulse;
	}
	deltaLinearVelocity1[lidx].w = invMass1;
	deltaLinearVelocity2[lidx].w = invMass2;
	pBody1->m_deltaLinVel_invMass = deltaLinearVelocity1[lidx];
	pBody1->m_deltaAngVel_frict = deltaAngularVelocity1[lidx];
	pBody2->m_deltaLinVel_invMass = deltaLinearVelocity2[lidx];
	pBody2->m_deltaAngVel_frict = deltaAngularVelocity2[lidx];
}


__kernel void kSetupFrictionConstraint(	int numFrictionConstr, 
									__global BCSOCLConstrRO* pConstrRO,
									__global BCSOCLConstrRW* pConstrRW,
									int frictionConstrOffs GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numFrictionConstr)
    {
		return;
	}
	__global BCSOCLConstrRO* pFrictCtRO = pConstrRO + frictionConstrOffs + index;
	__global BCSOCLConstrRW* pFrictCtRW = pConstrRW + frictionConstrOffs + index;
	int contIndex = pFrictCtRO->m_frictionIndex;
	__global BCSOCLConstrRW* pContCt = pConstrRW + contIndex;
	float totalImpulse = pContCt->m_appliedImpulse;
	if (totalImpulse > 0.f)
	{
		pFrictCtRW->m_lowerLimit = -pFrictCtRO->m_contactNormal_friction.w * totalImpulse;
		pFrictCtRW->m_upperLimit =  pFrictCtRO->m_contactNormal_friction.w * totalImpulse;
	}
}


);