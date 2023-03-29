
#include "cbase.h"

#include "vjolt_layers.h"

#include "vjolt_controller_player.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

static ConVar vjolt_player_collision_tolerance( "vjolt_player_collision_tolerance", "0.05" );

//-------------------------------------------------------------------------------------------------

JoltPhysicsPlayerController::JoltPhysicsPlayerController( JoltPhysicsObject *pObject )
{
	SetObjectInternal( pObject );
}

JoltPhysicsPlayerController::~JoltPhysicsPlayerController()
{
	SetObjectInternal( nullptr );
	SetGround( nullptr );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsPlayerController::Update( const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground )
{
	// timeOffset == secondsToArrival

	const JPH::Vec3 targetPosition = SourceToJolt::Distance( position );

	if ( targetPosition.IsClose( m_targetPosition ) )
		return;

	const JPH::Vec3 targetVelocity = SourceToJolt::Distance( velocity );

	m_targetPosition = targetPosition;
	m_targetVelocity = targetVelocity;
	m_secondsToArrival = secondsToArrival;

	// Bogus assertion: onground can be true and ground can be null when touching the world. That is okay
	//VJoltAssert( ( onground && ground ) || ( !onground && !ground ) );

	SetGround( static_cast<JoltPhysicsObject *>( ground ) );
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
	// Do we have to care about this? IVP used a rigid body for the player shadow because it didn't
	// have the concept of kinematic objects, our Jolt shadow follows the player exactly and
	// is kinematic so if the game follows this max speed limit we don't need to care.
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

	// Since the player is a kinematic object that slides around the world using velocity, when
	// stepping up onto a platform we need to go there instantly, AddToPosition does that.

	m_pObject->AddToPosition( JPH::Vec3( 0.0f, 0.0f, SourceToJolt::Distance( height ) ) );
}

void JoltPhysicsPlayerController::Jump()
{
	// This does nothing in VPhysics.
}

void JoltPhysicsPlayerController::GetShadowVelocity( Vector *velocity )
{
	if ( !velocity )
		return;

	JPH::Vec3 jphVelocity = m_pObject->GetBody()->GetLinearVelocity();
	if ( m_pGround )
	{
		jphVelocity -= m_pGround->GetBody()->GetPointVelocity( m_groundPos );
	}

	*velocity = JoltToSource::Distance( jphVelocity );
}

IPhysicsObject *JoltPhysicsPlayerController::GetObject()
{
	return m_pObject;
}

void JoltPhysicsPlayerController::GetLastImpulse( Vector *pOut )
{
	VectorClear( *pOut );
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

static void CheckCollision( JoltPhysicsObject *pObject, JPH::CollideShapeCollector &ioCollector, JPH::BodyFilter &ioFilter )
{
	JPH::PhysicsSystem *pSystem = pObject->GetEnvironment()->GetPhysicsSystem();

	// TODO(Josh): Make a PLAYER ONLY layer that will only collide with MOVING ONLY annd
	// NOTHING ELSE tomorrow.

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
	settings.mMaxSeparationDistance = vjolt_player_collision_tolerance.GetFloat();

	pSystem->GetNarrowPhaseQueryNoLock().CollideShape( pObject->GetBody()->GetShape(), JPH::Vec3::sReplicate( 1.0f ), query_transform, settings, JPH::Vec3::sZero(), ioCollector, broadphase_layer_filter, object_layer_filter, ioFilter );
}

// Slart: This is a version of CheckCollision that projects the player by their velocity, to attempt to push objects that we'll walk into soon
#if 0
static void CheckCollision2( JoltPhysicsObject *pObject, JPH::CollideShapeCollector &ioCollector, const JPH::Vec3Arg targetVelocity, float flDeltaTime )
{
	JPH::PhysicsSystem *pSystem = pObject->GetEnvironment()->GetPhysicsSystem();

	// TODO(Josh): Make a PLAYER ONLY layer that will only collide with MOVING ONLY annd
	// NOTHING ELSE tomorrow.

	// Create query broadphase layer filter
	JPH::DefaultBroadPhaseLayerFilter broadphase_layer_filter = pSystem->GetDefaultBroadPhaseLayerFilter( Layers::MOVING );

	// Create query object layer filter
	JPH::DefaultObjectLayerFilter object_layer_filter = pSystem->GetDefaultLayerFilter( Layers::MOVING );

	// Ignore my own body
	JPH::IgnoreSingleBodyFilter body_filter( pObject->GetBodyID() );

	// Determine position to test
	JPH::Vec3 position;
	JPH::Quat rotation;
	JPH::BodyInterface &bi = pSystem->GetBodyInterfaceNoLock();
	bi.GetPositionAndRotation( pObject->GetBodyID(), position, rotation );
	position += targetVelocity * ( flDeltaTime * 2.0f );
	JPH::Mat44 query_transform = JPH::Mat44::sRotationTranslation( rotation, position + rotation * pObject->GetBody()->GetShape()->GetCenterOfMass() );

	// Settings for collide shape
	JPH::CollideShapeSettings settings;
	settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;
	settings.mActiveEdgeMovementDirection = bi.GetLinearVelocity( pObject->GetBodyID() );
	settings.mBackFaceMode = JPH::EBackFaceMode::IgnoreBackFaces;
	settings.mMaxSeparationDistance = vjolt_player_collision_tolerance.GetFloat();

	pSystem->GetNarrowPhaseQueryNoLock().CollideShape( pObject->GetBody()->GetShape(), JPH::Vec3::sReplicate( 1.0f ), query_transform, settings, ioCollector, broadphase_layer_filter, object_layer_filter, body_filter );
}
#endif

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

class NormalWeightedCollector : public JPH::CollideShapeCollector
{
public:
	NormalWeightedCollector( JPH::PhysicsSystem *pPhysicsSystem )
		: m_pPhysicsSystem( pPhysicsSystem )
	{
	}

	void Reset() override
	{
		JPH::CollideShapeCollector::Reset();

		m_bHadHit = false;
		m_flLowestNormalZ = 1.0f;
	}

	void AddHit( const JPH::CollideShapeResult &inResult ) override
	{
		JPH::BodyLockRead lock( m_pPhysicsSystem->GetBodyLockInterfaceNoLock(), inResult.mBodyID2 );
		const JPH::Body &body = lock.GetBody();

		JPH::Vec3 normal = body.GetWorldSpaceSurfaceNormal( inResult.mSubShapeID2, inResult.mContactPointOn2 );
		m_flLowestNormalZ = Min( m_flLowestNormalZ, -normal.GetZ() );

		m_Hit = inResult;
		m_bHadHit = true;
	}

	inline bool HadHit() const
	{
		return m_bHadHit;
	}

	float m_flLowestNormalZ = 1.0f;

	JPH::CollideShapeCollector::ResultType m_Hit;

private:
	JPH::PhysicsSystem		*m_pPhysicsSystem;
	bool					m_bHadHit = false;
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

void JoltPhysicsPlayerController::OnPreSimulate( float flDeltaTime )
{
	VJoltAssertMsg( m_pObject->GetBody()->GetMotionType() == JPH::EMotionType::Kinematic, "Shadow controllers must be kinematic!" );

#if 0
	if ( m_pGround )
	{
		JPH::Mat44 matrix = JPH::Mat44::sRotationTranslation( m_pObject->GetBody()->GetRotation(), m_pObject->GetBody()->GetPosition() ).Transposed3x3();
		m_groundPos = -matrix.Multiply3x3( m_targetPosition );

		matrix3x4_t mat;
		m_pGround->GetPositionMatrix( &mat );
		m_targetPosition = mat.TransformVector( m_GroundPos );

		m_pGround->GetVelocityAtPoint( m_GroundPos, &groundVelocity );
		m_pObject->AddVelocity( -groundVelocity );
	}
#else
	/*if ( m_pGround )
	{
		JPH::Mat44 matrix = JPH::Mat44::sRotationTranslation( m_pObject->GetBody()->GetRotation(), m_pObject->GetBody()->GetPosition() ).Transposed3x3();
		JPH::Vec3 groundPos = -matrix.Multiply3x3( m_targetPosition );
		int g = 5;
		m_targetPosition = matrix * groundPos;
	}*/
#endif

	// Apply downwards force to the ground
	// This code mimics JoltObject::ApplyForceOffset but without Source > Jolt conversions
	/*if ( m_pGround && m_pGround->IsMoveable() )
	{
		JPH::PhysicsSystem *pPhysicsSystem = m_pGround->GetEnvironment()->GetPhysicsSystem();

		JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterfaceNoLock();
		bodyInterface.AddImpulse( m_pGround->GetBodyID(), pPhysicsSystem->GetGravity() * m_pObject->GetMass() * flDeltaTime, m_pObject->GetBody()->GetPosition() );
	}*/

	JPH::PhysicsSystem *pPhysicsSystem = m_pObject->GetEnvironment()->GetPhysicsSystem();
	JPH::BodyInterface &bodyInterface = pPhysicsSystem->GetBodyInterfaceNoLock();

	// Project ourselves towards our velocity
	NormalWeightedCollector collector( pPhysicsSystem );
	SourceHitFilter<true> filter( pPhysicsSystem, m_pObject );
	CheckCollision( m_pObject, collector, filter );
	
	// Source typically uses -0.7 for ground.
	if ( collector.HadHit() && collector.m_flLowestNormalZ < -0.7f )
	{
		JPH::BodyID otherID = collector.m_Hit.mBodyID2;

		//bodyInterface.AddImpulse( otherID, m_pObject->GetMass() * m_targetVelocity * flDeltaTime, m_pObject->GetBody()->GetPosition() );
		bodyInterface.AddImpulse( otherID, m_pObject->GetMass() * pPhysicsSystem->GetGravity() * flDeltaTime, m_pObject->GetBody()->GetPosition());
	}

	if ( m_secondsToArrival > 0.0f )
		bodyInterface.MoveKinematic( m_pObject->GetBodyID(), m_targetPosition, JPH::Quat::sIdentity(), m_secondsToArrival );
	else
	{
		bodyInterface.SetPositionAndRotation( m_pObject->GetBodyID(), m_targetPosition, JPH::Quat::sIdentity(), JPH::EActivation::Activate );
		bodyInterface.SetLinearAndAngularVelocity( m_pObject->GetBodyID(), JPH::Vec3::sZero(), JPH::Vec3::sZero() );
	}

	m_secondsToArrival = Max( m_secondsToArrival - flDeltaTime, 0.0f );
}

void JoltPhysicsPlayerController::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	if ( pObject == m_pObject )
	{
		SetObjectInternal( nullptr );
	}
	if ( pObject == m_pGround )
	{
		SetGround( nullptr );
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
		// Don't bother resetting kinematic or sleep state, it does not matter because
		// any object tied to a player controller was created to be a player object
		m_pObject->RemoveDestroyedListener( this );
		m_pObject->RemoveCallbackFlags( CALLBACK_IS_PLAYER_CONTROLLER );
	}

	// Set our new object
	m_pObject = pObject;

	// Adjust the new object
	if ( m_pObject )
	{
		// Set kinematic
		m_pObject->GetBody()->SetMotionType( JPH::EMotionType::Kinematic );
		m_pObject->GetBody()->SetAllowSleeping( false );

		m_pObject->AddDestroyedListener( this );
		m_pObject->AddCallbackFlags( CALLBACK_IS_PLAYER_CONTROLLER );
	}
}

void JoltPhysicsPlayerController::SetGround( JoltPhysicsObject *pGround )
{
	if ( m_pGround == pGround )
		return;

	if ( m_pGround )
	{
		m_pGround->RemoveDestroyedListener( this );
	}

	// Set our new ground
	m_pGround = pGround;

	if ( m_pGround )
	{
		m_pGround->AddDestroyedListener( this );
	}
}
