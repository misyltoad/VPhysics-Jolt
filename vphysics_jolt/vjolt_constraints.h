//=================================================================================================
//
// Constraints
//
//=================================================================================================

#pragma once

#include "vjolt_internal_listeners.h"

enum constraintType_t
{
	CONSTRAINT_UNKNOWN = 0,
	CONSTRAINT_RAGDOLL,
	CONSTRAINT_HINGE,
	CONSTRAINT_FIXED,
	CONSTRAINT_BALLSOCKET,
	CONSTRAINT_SLIDING,
	CONSTRAINT_PULLEY,
	CONSTRAINT_LENGTH,
};

class JoltPhysicsConstraint;
class JoltPhysicsEnvironment;

class JoltPhysicsConstraintGroup final : public IPhysicsConstraintGroup
{
public:
	JoltPhysicsConstraintGroup();
	~JoltPhysicsConstraintGroup() override;

	void Activate() override;
	bool IsInErrorState() override;
	void ClearErrorState() override;
	void GetErrorParams( constraint_groupparams_t *pParams ) override;
	void SetErrorParams( const constraint_groupparams_t &params ) override;
	void SolvePenetration( IPhysicsObject *pObj0, IPhysicsObject *pObj1 ) override;

	void AddConstraint( JoltPhysicsConstraint *pConstraint );
	void RemoveConstraint( JoltPhysicsConstraint *pConstraint );

private:
	std::vector< JoltPhysicsConstraint * >	m_pConstraints;
	constraint_groupparams_t				m_ErrorParams = {};
};

class JoltPhysicsConstraint final : public IPhysicsConstraint, public IJoltObjectDestroyedListener
{
public:
	JoltPhysicsConstraint( JoltPhysicsEnvironment *pPhysicsEnvironment, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, constraintType_t Type = CONSTRAINT_UNKNOWN, JPH::Constraint* pConstraint = nullptr, void *pGameData = nullptr );
	~JoltPhysicsConstraint() override;

	void			Activate() override;
	void			Deactivate() override;

	void			SetGameData( void *gameData ) override;
	void *			GetGameData() const override;

	IPhysicsObject *GetReferenceObject() const override;
	IPhysicsObject *GetAttachedObject() const override;

	void			SetLinearMotor( float speed, float maxLinearImpulse ) override;
	void			SetAngularMotor( float rotSpeed, float maxAngularImpulse ) override;

	void			UpdateRagdollTransforms( const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached ) override;
	bool			GetConstraintTransform( matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached ) const override;
	bool			GetConstraintParams( constraint_breakableparams_t *pParams ) const override;

	void			OutputDebugInfo() override;

	// IJoltObjectDestroyedListener
	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) override;

public:
	bool InitialiseHingeFromRagdoll( IPhysicsConstraintGroup* pGroup, const constraint_ragdollparams_t& ragdoll );
	void InitialiseRagdoll( IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll );
	void InitialiseHinge( IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge );
	void InitialiseSliding( IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding );
	void InitialiseBallsocket( IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket );
	void InitialiseFixed( IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed );
	void InitialiseLength( IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length );

	void SaveConstraintSettings( JPH::StateRecorder &recorder );

private:

	void SetGroup( IPhysicsConstraintGroup *pGroup );

	void DestroyConstraint();

	JoltPhysicsObject			*m_pObjReference = nullptr;
	JoltPhysicsObject			*m_pObjAttached = nullptr;
	JPH::Ref< JPH::Constraint > m_pConstraint;
	constraintType_t			m_ConstraintType = CONSTRAINT_UNKNOWN;

	JoltPhysicsConstraintGroup	*m_pGroup = nullptr;

	void						*m_pGameData = nullptr;
	JoltPhysicsEnvironment		*m_pPhysicsEnvironment = nullptr;
	JPH::PhysicsSystem			*m_pPhysicsSystem = nullptr;
};
