
#pragma once

#include "vjolt_object.h"
#include "vjolt_environment.h"

class JoltPhysicsPlayerController : public IPhysicsPlayerController, public IJoltObjectDestroyedListener, public IJoltPhysicsController
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

private:
	void SetObjectInternal( JoltPhysicsObject *pObject );
	void SetGround( JoltPhysicsObject *pObject );

private:
	JoltPhysicsObject *m_pObject = nullptr;
	IPhysicsPlayerControllerEvent *m_pHandler = nullptr;

	JoltPhysicsObject *m_pGround = nullptr;
	JPH::Vec3 m_groundPos = JPH::Vec3::sZero();

	JPH::Vec3 m_targetPosition = JPH::Vec3::sZero();			// Where we want to be
	JPH::Vec3 m_targetVelocity = JPH::Vec3::sZero();			// How we want to be
	float m_secondsToArrival = FLT_EPSILON;						// When we want to be

	float m_maxSpeed = 0.0f;
	float m_maxDampSpeed = 0.0f;
	float m_maxAngular = 0.0f;
	float m_maxDampAngular = 0.0f;
	float m_teleportDistance = 0.0f;
	bool m_isPhysicallyControlled = false;		// If true we're a bone follower on an NPC or something...
	bool m_allowTranslation = false;			// Should we translate?
	bool m_allowRotation = false;				// Should we rotate?

	float m_flPushableMassLimit = 1e4f;
	float m_flPushableSpeedLimit = 1e4f;

	uint16 m_savedMaterialIndex = 0;
};
