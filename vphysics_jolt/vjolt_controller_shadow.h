
#pragma once

#include "vjolt_object.h"
#include "vjolt_environment.h"

class JoltPhysicsShadowController final : public IPhysicsShadowController, public IJoltPhysicsController
{
public:
	JoltPhysicsShadowController( JoltPhysicsObject *pObject, bool allowTranslation, bool allowRotation );
	~JoltPhysicsShadowController() override;

	void Update( const Vector &position, const QAngle &angles, float timeOffset ) override;
	void MaxSpeed( float maxSpeed, float maxAngularSpeed ) override;
	void StepUp( float height ) override;
	
	void SetTeleportDistance( float teleportDistance ) override;
	bool AllowsTranslation() override;
	bool AllowsRotation() override;

	void SetPhysicallyControlled( bool isPhysicallyControlled ) override;
	bool IsPhysicallyControlled() override;
	void GetLastImpulse( Vector *pOut ) override;
	void UseShadowMaterial( bool bUseShadowMaterial ) override;
	void ObjectMaterialChanged( int materialIndex ) override;

	float GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut ) override;
	
	float GetTeleportDistance() override;
	void GetMaxSpeed( float *pMaxSpeedOut, float *pMaxAngularSpeedOut ) override;
	
	// IJoltPhysicsController
	void OnPreSimulate( float flDeltaTime ) override;

private:
	JoltPhysicsObject *m_pObject = nullptr;

	JPH::Vec3 m_targetPosition = JPH::Vec3::sZero();			// Where we want to be
	JPH::Quat m_targetRotation = JPH::Quat::sIdentity();		// How we want to be
	float m_secondsToArrival = 0;						// When we want to be

	float m_maxSpeed = 0.0f;
	float m_maxDampSpeed = 0.0f;
	float m_maxAngular = 0.0f;
	float m_maxDampAngular = 0.0f;
	float m_teleportDistance = 0.0f;
	bool m_isPhysicallyControlled = false;		// If true we're a bone follower on an NPC or something...
	bool m_allowTranslation = false;			// Should we translate?
	bool m_allowRotation = false;				// Should we rotate?

	bool m_enabled = false;

	uint16 m_savedMaterialIndex = 0;
	uint16 m_savedCallbackFlags = 0;
};
