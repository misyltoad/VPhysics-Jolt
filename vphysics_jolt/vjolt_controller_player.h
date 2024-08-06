
#pragma once

#include "vjolt_object.h"
#include "vjolt_environment.h"

class JoltPhysicsPlayerController : public IPhysicsPlayerController, public IJoltObjectDestroyedListener, public IJoltPhysicsController, public JPH::CharacterContactListener
{
public:
	JoltPhysicsPlayerController( JoltPhysicsObject *pObject );
	~JoltPhysicsPlayerController() override;

	void Update( const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground ) override;
	void SetEventHandler( IPhysicsPlayerControllerEvent *handler ) override;
	bool IsInContact( void ) override;
	void MaxSpeed( const Vector &maxVelocity ) override;

	void SetObject( IPhysicsObject *pObject ) override;

	int GetShadowPosition( Vector *position, QAngle *angles ) override;
	void StepUp( float height ) override;
	void Jump() override;
	void GetShadowVelocity( Vector *velocity ) override;
	IPhysicsObject *GetObject() override;
	void GetLastImpulse( Vector *pOut ) override;

	void SetPushMassLimit( float maxPushMass ) override;
	void SetPushSpeedLimit( float maxPushSpeed ) override;

	float GetPushMassLimit() override;
	float GetPushSpeedLimit() override;

	bool WasFrozen() override;

	uint32 GetContactState( uint16 nGameFlags ) override_portal2;

	// IJoltObjectDestroyedListener
	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) override;
	// IJoltPhysicsController
	void OnPreSimulate( float flDeltaTime ) override;
	void OnPostSimulate( float flDeltaTime ) override;

	int TryTeleportObject();

	bool OnContactValidate( const JPH::CharacterVirtual* inCharacter, const JPH::BodyID& inBodyID2, const JPH::SubShapeID& inSubShapeID2 );
	void OnContactAdded( const JPH::CharacterVirtual* inCharacter, const JPH::BodyID& inBodyID2, const JPH::SubShapeID& inSubShapeID2, JPH::RVec3Arg inContactPosition, JPH::Vec3Arg inContactNormal, JPH::CharacterContactSettings& ioSettings );

private:
	void SetObjectInternal( JoltPhysicsObject *pObject );

private:
	
	JPH::Ref<JPH::Character> m_pCharacter;
	JPH::Ref<JPH::Shape> m_pShape;

	JoltPhysicsObject *m_pObject = nullptr;

	Vector m_vOldPosition = vec3_origin;

	IPhysicsPlayerControllerEvent *m_pHandler = nullptr;
	float m_flMaxDeltaPosition = 24.0f;
	float m_flDampFactor = 1.0f;
	float m_flSecondsToArrival = 0.0f;
	float m_flPushableSpeedLimit = 1e4f;
	float m_flPushableMassLimit = VPHYSICS_MAX_MASS;
	Vector m_vTargetPosition = vec3_origin;
	Vector m_vMaxSpeed = vec3_origin;
	Vector m_vCurrentSpeed = vec3_origin;
	Vector m_vLastImpulse = vec3_origin;

	bool m_bEnable = false;
	bool m_bUpdatedSinceLast = false;


};
