
#include "cbase.h"

#include "vjolt_layers.h"

#include "vjolt_controller_player.h"
#include "vjolt_debugrender.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static ConVar vjolt_player_collision_tolerance( "vjolt_player_collision_tolerance", "0.05" );
static ConVar vjolt_player_character_padding( "vjolt_player_character_padding", "0.02" );

static ConVar vjolt_player_debug( "vjolt_player_debug", "0" );
static ConVar vjolt_player_disable_limit( "vjolt_player_disable_limit", "0.1", 0, "The min speed^2 before we just go where physics wants to take us" );

//-------------------------------------------------------------------------------------------------

// Component-wise Vector clamp
static Vector ClampVector( const Vector &x, const Vector &min, const Vector &max )
{
	return Vector(
		Clamp( x.x, min.x, max.x ),
		Clamp( x.y, min.y, max.y ),
		Clamp( x.z, min.z, max.z ) );
}

static void ComputePlayerController( Vector &vCurrentSpeed, const Vector &vDelta, const Vector &vMaxSpeed, float flScaleDelta, float flDamping, Vector *pOutImpulse )
{
	if ( vCurrentSpeed.LengthSqr() < 1e-6f )
	{
		vCurrentSpeed = vec3_origin;
	}

	Vector vDampAccel = vCurrentSpeed * -flDamping;
	Vector vDeltaAccel = vDelta * flScaleDelta;

	Vector vAcceleration = ClampVector( vDeltaAccel + vDampAccel, -vMaxSpeed, vMaxSpeed );

	vCurrentSpeed += vAcceleration;
	if ( pOutImpulse )
		*pOutImpulse = vAcceleration;
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsPlayerController::JoltPhysicsPlayerController( JoltPhysicsObject *pObject )
{
	SetObjectInternal( pObject );
}

JoltPhysicsPlayerController::~JoltPhysicsPlayerController()
{
	SetObjectInternal( nullptr );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsPlayerController::Update( const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground )
{
	m_bUpdatedSinceLast = true;

	if ( ( velocity - m_vCurrentSpeed ).LengthSqr() < 1e-6f && ( position - m_vTargetPosition ).LengthSqr() < 1e-6f )
		return;

	m_vTargetPosition = position;
	m_flSecondsToArrival = secondsToArrival < 0 ? 0 : secondsToArrival;

	m_vCurrentSpeed = velocity;
	m_pCharacter->Activate();
	m_bEnable = true;

	if ( velocity.LengthSqr() <= vjolt_player_disable_limit.GetFloat() )
	{
		// No input velocity, just go where physics takes you.
		m_bEnable = false;
	}
	else
	{
		MaxSpeed( velocity );
	}

	// We ignore the given ground here, we use the Jolt player controller's ground.
}

void JoltPhysicsPlayerController::SetEventHandler( IPhysicsPlayerControllerEvent *handler )
{
	m_pHandler = handler;
}

bool JoltPhysicsPlayerController::IsInContact()
{
	uint32 nState = GetContactState( 0 );
	return !!( nState & PLAYER_CONTACT_PHYSICS );
}

void JoltPhysicsPlayerController::MaxSpeed( const Vector &velocity )
{
	Vector vCurrentVelocity;
	m_pObject->GetVelocity( &vCurrentVelocity, nullptr );

	Vector vDirection = velocity;
	float flLength = VectorNormalize( vDirection ); // Normalizes in place.

	float flDot = DotProduct( vDirection, vCurrentVelocity );
	if ( flDot > 0 )
	{
		m_vMaxSpeed = Abs( velocity - ( vDirection * flDot * flLength ) );
	}
	else
	{
		m_vMaxSpeed = Abs( velocity );
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsPlayerController::SetObject( IPhysicsObject *pObject )
{
	SetObjectInternal( static_cast<JoltPhysicsObject *>( pObject ) );
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsPlayerController::GetShadowPosition( Vector *position, QAngle *angles )
{
	return m_pObject->GetShadowPosition( position, angles );
}

void JoltPhysicsPlayerController::StepUp( float height )
{
	if ( height == 0.0f )
		return;

	Vector vPos;
	QAngle qAngles;
	m_pObject->GetPosition( &vPos, &qAngles );
	vPos.z += height;
	// Teleport, do not influence implicit velocity.
	m_pObject->SetPosition( vPos, qAngles, true );
}

void JoltPhysicsPlayerController::Jump()
{
	// This does nothing in VPhysics.
}

void JoltPhysicsPlayerController::GetShadowVelocity( Vector *velocity )
{
	if ( !velocity )
		return;

	m_pObject->GetVelocity( velocity, nullptr );

	Vector vBaseVelocity = JoltToSource::Distance( m_pCharacter->GetGroundVelocity() );
	*velocity -= vBaseVelocity;
}

IPhysicsObject *JoltPhysicsPlayerController::GetObject()
{
	return m_pObject;
}

void JoltPhysicsPlayerController::GetLastImpulse( Vector *pOut )
{
	if ( pOut )
		*pOut = m_vLastImpulse;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsPlayerController::SetPushMassLimit( float maxPushMass )
{
	m_flPushableMassLimit = maxPushMass;
}

void JoltPhysicsPlayerController::SetPushSpeedLimit( float maxPushSpeed )
{
	m_flPushableSpeedLimit = maxPushSpeed;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsPlayerController::GetPushMassLimit()
{
	return m_flPushableMassLimit;
}

float JoltPhysicsPlayerController::GetPushSpeedLimit()
{
	return m_flPushableSpeedLimit;
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsPlayerController::WasFrozen()
{
	// I think here the code is referring to IVP freezing objects after inactivity (sleeping),
	// our objects are forced to never sleep, so we don't need to care?
	return false;
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsPlayerController::OnContactValidate( const JPH::CharacterVirtual* inCharacter, const JPH::BodyID& inBodyID2, const JPH::SubShapeID& inSubShapeID2 )
{
	JPH::Body *pOtherBody = m_pObject->GetEnvironment()->GetPhysicsSystem()->GetBodyLockInterfaceNoLock().TryGetBody( inBodyID2 );
	JoltPhysicsObject* pOtherObject = reinterpret_cast< JoltPhysicsObject* >( pOtherBody->GetUserData() );
	JoltPhysicsContactListener *pListener = m_pObject->GetEnvironment()->GetContactListener();
	return pListener->ShouldCollide( m_pObject, pOtherObject );
}

void JoltPhysicsPlayerController::OnContactAdded( const JPH::CharacterVirtual* inCharacter, const JPH::BodyID& inBodyID2, const JPH::SubShapeID& inSubShapeID2, JPH::RVec3Arg inContactPosition, JPH::Vec3Arg inContactNormal, JPH::CharacterContactSettings& ioSettings )
{
	JoltPhysicsContactListener *pListener = m_pObject->GetEnvironment()->GetContactListener();
	( void )pListener;
}

//-------------------------------------------------------------------------------------------------

static void CheckCollision( JoltPhysicsObject *pObject, JPH::CollideShapeCollector &ioCollector, JPH::BodyFilter &ioFilter )
{
	JPH::PhysicsSystem *pSystem = pObject->GetEnvironment()->GetPhysicsSystem();

	if ( !pObject->IsCollisionEnabled() )
		return;

	// Create query broadphase layer filter
	JPH::DefaultBroadPhaseLayerFilter broadphase_layer_filter = pSystem->GetDefaultBroadPhaseLayerFilter( Layers::MOVING );

	// Create query object layer filter
	JPH::DefaultObjectLayerFilter object_layer_filter = pSystem->GetDefaultLayerFilter( Layers::MOVING );

	// Determine position to test
	JPH::Vec3 position;
	JPH::Quat rotation;
	JPH::BodyInterface &bi = pSystem->GetBodyInterfaceNoLock();
	bi.GetPositionAndRotation( pObject->GetBodyID(), position, rotation );
	JPH::Mat44 query_transform = JPH::Mat44::sRotationTranslation( rotation, position + rotation * pObject->GetBody()->GetShape()->GetCenterOfMass() );

	// Settings for collide shape
	JPH::CollideShapeSettings settings;
	settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;
	settings.mActiveEdgeMovementDirection = bi.GetLinearVelocity( pObject->GetBodyID() );
	settings.mBackFaceMode = JPH::EBackFaceMode::IgnoreBackFaces;
	settings.mMaxSeparationDistance = vjolt_player_character_padding.GetFloat();

	pSystem->GetNarrowPhaseQueryNoLock().CollideShape( pObject->GetBody()->GetShape(), JPH::Vec3::sReplicate( 1.0f ), query_transform, settings, JPH::Vec3::sZero(), ioCollector, broadphase_layer_filter, object_layer_filter, ioFilter );
}


template <bool MoveablesOnly>
class SourceHitFilter : public JPH::BodyFilter
{
public:
	SourceHitFilter( JPH::PhysicsSystem* pPhysicsSystem, JoltPhysicsObject* pSelfObject )
		: m_pPhysicsSystem( pPhysicsSystem )
		, m_pSelfObject( pSelfObject )
	{
	}

	bool ShouldCollideLocked( const JPH::Body &inBody ) const override
	{
		JoltPhysicsObject* pObject = reinterpret_cast<JoltPhysicsObject*>( inBody.GetUserData() );

		// Ignore self if specified. This can be nullptr if you don't want this.
		if ( pObject == m_pSelfObject )
			return false;

		if constexpr ( MoveablesOnly )
		{
			if ( pObject->IsTrigger() || !pObject->IsMoveable() )
				return false;
		}

		if ( !pObject->GetEnvironment()->GetContactListener()->ShouldCollide( m_pSelfObject, pObject ) )
			return false;

		return true;
	}

private:
	JPH::PhysicsSystem	*m_pPhysicsSystem;
	JoltPhysicsObject	*m_pSelfObject;
};

uint32 JoltPhysicsPlayerController::GetContactState( uint16 nGameFlags )
{
	// This does not seem to affect much, we should aspire to have our physics be as 1:1 to brush collisions as possible anyway
#ifdef GAME_PORTAL2_OR_NEWER
	if ( !m_pObject->IsCollisionEnabled() )
		return 0;

	// Collector that finds the hit with the normal that is the most 'up'
	class ContactStateCollector : public JPH::CollideShapeCollector
	{
	public:
		ContactStateCollector( JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsObject *pPlayerObject, uint16 nGameFlags )
			: m_pPhysicsSystem( pPhysicsSystem )
			, m_pPlayerObject( pPlayerObject )
			, m_nGameFlags( nGameFlags )
		{
		}

		void Reset() override
		{
			JPH::CollideShapeCollector::Reset();

			m_nFlagsOut = 0;
		}

		void AddHit( const JPH::CollideShapeResult &inResult ) override
		{
			JPH::BodyLockRead lock( m_pPhysicsSystem->GetBodyLockInterfaceNoLock(), inResult.mBodyID2 );
			const JPH::Body &body = lock.GetBody();

			JoltPhysicsObject *pObject = reinterpret_cast<JoltPhysicsObject *>( body.GetUserData() );

			if ( !pObject->IsControlledByGame() )
				m_nFlagsOut |= PLAYER_CONTACT_PHYSICS;

			if ( pObject->GetGameFlags() & m_nGameFlags )
				m_nFlagsOut |= PLAYER_CONTACT_GAMEOBJECT;
		}

		uint32					m_nFlagsOut = 0;

	private:
		JPH::PhysicsSystem		*m_pPhysicsSystem;
		JoltPhysicsObject		*m_pPlayerObject;
		uint16					m_nGameFlags;
	};

	JPH::PhysicsSystem *pSystem = m_pObject->GetEnvironment()->GetPhysicsSystem();
	ContactStateCollector collector( pSystem, m_pObject, nGameFlags );
	SourceHitFilter<true> filter( pSystem, m_pObject );
	CheckCollision( m_pObject, collector, filter );

	return collector.m_nFlagsOut;
#else
	return 0;
#endif
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsPlayerController::TryTeleportObject()
{
	if ( m_pHandler )
	{
		if ( !m_pHandler->ShouldMoveTo( m_pObject, m_vTargetPosition ) )
			return 0;
	}

	QAngle qCurrentAngles;
	m_pObject->GetPosition( nullptr, &qCurrentAngles );
	m_pObject->SetPosition( m_vTargetPosition, qCurrentAngles, true );
	m_pCharacter->SetPosition( SourceToJolt::Distance( m_vTargetPosition ) );
	return 1;
}

void JoltPhysicsPlayerController::OnPreSimulate( float flDeltaTime )
{
	m_pCharacter->SetLayer( m_pObject->IsCollisionEnabled() ? Layers::MOVING : Layers::NO_COLLIDE );

	// Update position from dummy object.
	{
		Vector vObjectPosition;
		QAngle qObjectAngle;
		m_pObject->GetPosition( &vObjectPosition, &qObjectAngle );
		m_pCharacter->SetPositionAndRotation( SourceToJolt::Distance( vObjectPosition ), SourceToJolt::Angle( qObjectAngle ), JPH::EActivation::DontActivate );
	}

	Vector vOldPosition = JoltToSource::Distance( m_pCharacter->GetPosition() );
	Vector vOldVelocity = JoltToSource::Distance( m_pCharacter->GetLinearVelocity() );

	Vector vDeltaPos = m_vTargetPosition - vOldPosition;

	if ( m_bEnable )
	{
		// Totally bogus! Measure error using last known estimate not current position.
		if ( vDeltaPos.LengthSqr() > JPH::Square( m_flMaxDeltaPosition ) )
		{
			if ( TryTeleportObject() )
				return;
		}
	}

	float flFraction = Min( m_flSecondsToArrival > 0.0f ? flDeltaTime / m_flSecondsToArrival : 1.0f, 1.0f );

	// XXX TODO Set Mass
	//m_pCharacter->GetBodyID()->SetMass(m_pObject->GetMass() * vjolt_player_mass_scale.GetFloat());
	m_pCharacter->SetPosition( SourceToJolt::Distance( vOldPosition ) );

	if ( m_bEnable )
	{
		Vector vGroundVelocity = JoltToSource::Distance( m_pCharacter->GetGroundVelocity() );

		Vector vControllerVelocity = vOldVelocity;

		vControllerVelocity -= vGroundVelocity;
		if ( !m_bUpdatedSinceLast )
		{
			float flLen = m_vLastImpulse.Length();
			Vector vTempMaxSpeed = Vector( flLen, flLen, flLen );
			ComputePlayerController( vControllerVelocity, vDeltaPos, vTempMaxSpeed, flFraction / flDeltaTime, m_flDampFactor, nullptr );
		}
		else
		{
			ComputePlayerController( vControllerVelocity, vDeltaPos, m_vMaxSpeed, flFraction / flDeltaTime, m_flDampFactor, &m_vLastImpulse );
		}
		vControllerVelocity += vGroundVelocity;

		m_pCharacter->SetLinearVelocity( SourceToJolt::Distance( vControllerVelocity ) );
	}

	m_vOldPosition = vOldPosition;
}

void JoltPhysicsPlayerController::OnPostSimulate( float flDeltaTime )
{
	m_pCharacter->PostSimulation( vjolt_player_collision_tolerance.GetFloat() );

	// Calculate effective velocity
	Vector vNewPosition = JoltToSource::Distance( m_pCharacter->GetPosition() );
	Vector vNewVelocity = ( vNewPosition - m_vOldPosition ) / flDeltaTime;
	AngularImpulse vAngularImpulse;

	m_pObject->SetPosition( vNewPosition, QAngle(), false );
	m_pObject->SetVelocity( &vNewVelocity, &vAngularImpulse );

	m_vLastImpulse = vNewVelocity;

	if ( vjolt_player_debug.GetBool() )
	{
		JoltPhysicsDebugRenderer& debugRenderer = JoltPhysicsDebugRenderer::GetInstance();

		// Draw last impulse as a blue line.
		debugRenderer.DrawLine(
			SourceToJolt::Distance( vNewPosition ),
			SourceToJolt::Distance( vNewPosition + m_vLastImpulse ),
			JPH::Color( 0, 0, 255, 255 ) );

		// Draw new player velocity as a purple line.
		debugRenderer.DrawLine(
			SourceToJolt::Distance( vNewPosition ),
			SourceToJolt::Distance( vNewPosition + vNewVelocity ),
			JPH::Color( 255, 0, 255, 255 ) );

		Vector vecMins, vecMaxs;
		JPH::Mat44 matComTransform = m_pCharacter->GetWorldTransform().PreTranslated( m_pCharacter->GetShape()->GetCenterOfMass() );
		JoltToSource::AABBBounds( m_pCharacter->GetShape()->GetWorldSpaceBounds( matComTransform, JPH::Vec3{ 1.0f, 1.0f, 1.0f } ), vecMins, vecMaxs );
		debugRenderer.GetDebugOverlay()->AddBoxOverlay( vec3_origin, vecMins, vecMaxs, QAngle(), m_bEnable ? 0 : 255, m_bEnable ? 255 : 0, 0, 100, 0.0f );

#if 0
		Log_Msg( LOG_VJolt,
			"Player State:\n"
			"  vOldPosition: %g %g %g\n"
			"  vOldVelocity: %g %g %g\n"
			"  vNewPosition: %g %g %g\n"
			"  vNewVelocity: %g %g %g\n"
			"  m_vLastImpulse: %g %g %g\n"
			"  vControllerVelocity: %g %g %g\n"
			"  vGroundVelocity: %g %g %g\n",
			vOldPosition.x, vOldPosition.x, vOldPosition.z,
			vOldVelocity.x, vOldVelocity.x, vOldVelocity.z,
			vNewPosition.x, vNewPosition.x, vNewPosition.z,
			vNewVelocity.x, vNewVelocity.x, vNewVelocity.z,
			m_vLastImpulse.x, m_vLastImpulse.x, m_vLastImpulse.z,
			vControllerVelocity.x, vControllerVelocity.x, vControllerVelocity.z,
			vGroundVelocity.x, vGroundVelocity.x, vGroundVelocity.z );
#endif
	}

	if ( m_bEnable )
	{
		m_flSecondsToArrival = Max( m_flSecondsToArrival - flDeltaTime, 0.0f );
	}
}

void JoltPhysicsPlayerController::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	if ( pObject == m_pObject )
	{
		SetObjectInternal( nullptr );
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsPlayerController::SetObjectInternal( JoltPhysicsObject *pObject )
{
	if ( m_pObject == pObject )
		return;

	// Reset the last object
	if ( m_pObject )
	{
		m_pObject->RemoveDestroyedListener( this );
		m_pObject->RemoveCallbackFlags( CALLBACK_IS_PLAYER_CONTROLLER );
		m_pObject->UpdateLayer();

		m_pCharacter->RemoveFromPhysicsSystem();
		m_pCharacter = nullptr;
	}

	// Set our new object
	m_pObject = pObject;

	// Adjust the new object
	if ( m_pObject )
	{
		// Set kinematic
		m_pObject->GetBody()->SetMotionType( JPH::EMotionType::Kinematic );
		m_pObject->AddDestroyedListener( this );
		m_pObject->AddCallbackFlags( CALLBACK_IS_PLAYER_CONTROLLER );
		m_pObject->UpdateLayer();

		static constexpr float k_flNormalSurfaceFriction = 0.8f; // Default surface friction.
		// We can't always get external convars in VPhysics Jolt sadly...
		// At least to my knowledge.
		// Assume a friction of "8" (the default) for now
		//ConVarRef sv_friction( "sv_friction" );
		static constexpr float sv_friction = 8;

		JPH::Ref<JPH::CharacterSettings> settings = new JPH::CharacterSettings();
		settings->mMass                        = m_pObject->GetMass();
		settings->mLayer                       = Layers::MOVING;
		settings->mUp                          = JPH::Vec3::sAxisZ();
		settings->mFriction                    = sv_friction * k_flNormalSurfaceFriction * ( 1.0f / 64.0f ); // Account for Source's friction being tick based.
		settings->mShape                       = m_pObject->GetBody()->GetShape();
		settings->mMaxSlopeAngle               = JPH::DegreesToRadians( 45.573 );
		settings->mEnhancedInternalEdgeRemoval = true;

		m_pCharacter = new JPH::Character( settings, m_pObject->GetBody()->GetPosition(), JPH::Quat::sIdentity(), m_pObject->GetBody()->GetUserData(), m_pObject->GetEnvironment()->GetPhysicsSystem() );
		m_pCharacter->AddToPhysicsSystem();
	}
}
