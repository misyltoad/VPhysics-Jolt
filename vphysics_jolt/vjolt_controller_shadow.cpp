
#include "cbase.h"

#include "vjolt_controller_shadow.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

JoltPhysicsShadowController::JoltPhysicsShadowController( JoltPhysicsObject *pObject, bool allowTranslation, bool allowRotation )
	: m_pObject( pObject ), m_allowTranslation( allowTranslation ), m_allowRotation( allowRotation )
{
	// Make our object kinematic
	m_pObject->GetBody()->SetMotionType( JPH::EMotionType::Kinematic );

	m_savedCallbackFlags = m_pObject->GetCallbackFlags();
	m_pObject->SetCallbackFlags( m_savedCallbackFlags | CALLBACK_SHADOW_COLLISION );
}

JoltPhysicsShadowController::~JoltPhysicsShadowController()
{
	if ( !( m_pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
	{
		m_pObject->SetCallbackFlags( m_savedCallbackFlags );
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsShadowController::Update( const Vector &position, const QAngle &angles, float timeOffset )
{
	// timeOffset == secondsToArrival

	JPH::Vec3 targetPosition = SourceToJolt::Distance( position );
	JPH::Quat targetRotation = SourceToJolt::Angle( angles );

	if ( targetPosition.IsClose( m_targetPosition, 1e-8f ) && targetRotation.IsClose( m_targetRotation, 1e-8f ) )
		return;

	m_targetPosition = targetPosition;
	m_targetRotation = targetRotation;
	m_secondsToArrival = Max( timeOffset, 0.0f );
	m_enabled = true;
}

void JoltPhysicsShadowController::MaxSpeed( float maxSpeed, float maxAngularSpeed )
{
	m_maxSpeed = maxSpeed;
	m_maxDampSpeed = maxSpeed;
	m_maxAngular = maxAngularSpeed;
	m_maxDampAngular = maxAngularSpeed;
}

void JoltPhysicsShadowController::StepUp( float height )
{
	if ( height == 0.0f )
		return;

	m_pObject->AddToPosition( JPH::Vec3( 0.0f, 0.0f, SourceToJolt::Distance( height ) ) );
}

//-------------------------------------------------------------------------------------------------
	
void JoltPhysicsShadowController::SetTeleportDistance( float teleportDistance )
{
	m_teleportDistance = SourceToJolt::Distance( teleportDistance );
}

bool JoltPhysicsShadowController::AllowsTranslation()
{
	return m_allowTranslation;
}

bool JoltPhysicsShadowController::AllowsRotation()
{
	return m_allowRotation;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsShadowController::SetPhysicallyControlled( bool isPhysicallyControlled )
{
	m_isPhysicallyControlled = isPhysicallyControlled;
}

bool JoltPhysicsShadowController::IsPhysicallyControlled()
{
	return m_isPhysicallyControlled;
}

void JoltPhysicsShadowController::GetLastImpulse( Vector *pOut )
{
	//*pOut = JoltToSource::Distance( m_lastImpulse );
	VectorClear( *pOut );
}

// HACK HACK HACK WE MIGHT WANT TO CHANGE THIS
// IMPLEMENT ME!
static constexpr int ShadowMaterialIndex = 0xF000;

void JoltPhysicsShadowController::UseShadowMaterial( bool bUseShadowMaterial )
{
	if ( !m_pObject )
		return;

#if 0
	int current = m_pObject->GetMaterialIndex();
	int target = bUseShadowMaterial ? ShadowMaterialIndex : m_savedMaterialIndex;
	if ( target != current )
		m_pObject->SetMaterialIndex( target );
#endif
}

void JoltPhysicsShadowController::ObjectMaterialChanged( int materialIndex )
{
	if ( !m_pObject )
		return;

	m_savedMaterialIndex = materialIndex;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsShadowController::GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut )
{
	if ( pPositionOut )
		*pPositionOut = JoltToSource::Distance( m_targetPosition );
	if ( pAnglesOut )
		*pAnglesOut = JoltToSource::Angle( m_targetRotation );

	return m_secondsToArrival;
}

//-------------------------------------------------------------------------------------------------
	
float JoltPhysicsShadowController::GetTeleportDistance()
{
	return JoltToSource::Distance( m_teleportDistance );
}

void JoltPhysicsShadowController::GetMaxSpeed( float *pMaxSpeedOut, float *pMaxAngularSpeedOut )
{
	if ( pMaxSpeedOut )
		*pMaxSpeedOut = m_maxSpeed;

	if ( pMaxAngularSpeedOut )
		*pMaxAngularSpeedOut = m_maxAngular;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsShadowController::OnPreSimulate( float flDeltaTime )
{
	if (!m_enabled)
		return;

	VJoltAssertMsg( m_pObject->GetBody()->GetMotionType() == JPH::EMotionType::Kinematic, "Shadow controllers must be kinematic!" );

	JPH::BodyInterface &bodyInterface = m_pObject->GetEnvironment()->GetPhysicsSystem()->GetBodyInterfaceNoLock();
	if ( m_secondsToArrival > 0.0f )
		bodyInterface.MoveKinematic( m_pObject->GetBodyID(), m_targetPosition, m_targetRotation, m_secondsToArrival );
	else
	{
		bodyInterface.SetPositionAndRotation( m_pObject->GetBodyID(), m_targetPosition, m_targetRotation, JPH::EActivation::Activate );
		bodyInterface.SetLinearAndAngularVelocity( m_pObject->GetBodyID(), JPH::Vec3::sZero(), JPH::Vec3::sZero() );
		m_enabled = false;
	}

	m_secondsToArrival = Max( m_secondsToArrival - flDeltaTime, 0.0f );
}
