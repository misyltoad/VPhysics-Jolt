//=================================================================================================
//
// Interface to a physics scene
// Physics environments are implemented as discrete JPH::PhysicsSystems.
// Portal and Portal 2 have two of these.
//
// Notes:
//  Josh: We always use BodyInterfaceNoLock and deal with unlocked bodies now.
//  Jolt starts to get very angry and asserts at us for doing that but we are never simulating
//  or having it do things while the bodies are technically unlocked -- but it doesn't matter.
//  We need to do this so we can assume the lock during the callbacks for collisions and stuff.
//
//=================================================================================================

#include "cbase.h"

#include "vjolt_callstack.h"
#include "vjolt_collide.h"
#include "vjolt_constraints.h"
#include "vjolt_controller_fluid.h"
#include "vjolt_controller_motion.h"
#include "vjolt_controller_player.h"
#include "vjolt_controller_shadow.h"
#include "vjolt_controller_vehicle.h"
#include "vjolt_debugrender.h"
#include "vjolt_layers.h"
#include "vjolt_object.h"
#include "vjolt_state_recorder_file.h"

#include "vjolt_environment.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

// This is the max amount of rigid bodies that you can add to the physics system. If you try to add more you'll get an error.
static constexpr uint kMaxBodies = 16384;

// This determines how many mutexes to allocate to protect rigid bodies from concurrent access. Set it to 0 for the default settings.
static constexpr uint kNumBodyMutexes = 0;

// This is the max amount of body pairs that can be queued at any time (the broad phase will detect overlapping
// body pairs based on their bounding boxes and will insert them into a queue for the narrowphase). If you make this buffer
// too small the queue will fill up and the broad phase jobs will start to do narrow phase work. This is slightly less efficient.
static constexpr uint kMaxBodyPairs = kMaxBodies;

// This is the maximum size of the contact constraint buffer. If more contacts (collisions between bodies) are detected than this
// number then these contacts will be ignored and bodies will start interpenetrating / fall through the world.
static constexpr uint kMaxContactConstraints = kMaxBodies;

static ConVar vjolt_linearcast( "vjolt_linearcast", "1", FCVAR_NONE, "Whether bodies will be created with linear cast motion quality (only takes effect after map restart)." );
static ConVar vjolt_initial_simulation( "vjolt_initial_simulation", "0", FCVAR_NONE, "Whether to pre-settle physics objects on map load." );

static ConVar vjolt_substeps_collision( "vjolt_substeps_collision", "1", FCVAR_NONE, "Number of collision steps to perform.", true, 0.0f, true, 4.0f );
static ConVar vjolt_substeps_integration( "vjolt_substeps_integration", "1", FCVAR_NONE, "Number of integration substeps to perform.", true, 0.0f, true, 4.0f );

static ConVar vjolt_baumgarte_factor( "vjolt_baumgarte_factor", "0.2", FCVAR_NONE, "Baumgarte stabilization factor (how much of the position error to 'fix' in 1 update). Changing this may help with constraint stability. Requires a map restart to change.", true, 0.0f, true, 1.0f );

//-------------------------------------------------------------------------------------------------

class JoltObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter
{
public:
	// Function that determines if two object layers can collide
	bool ShouldCollide( JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2 ) const override
	{
		switch ( inObject1 )
		{
		// NO_COLLIDE collides with nothing.
		case Layers::NO_COLLIDE:
			return	false;
		// NON_MOVING collides with moving objects and debris.
		case Layers::NON_MOVING_WORLD:
		case Layers::NON_MOVING_OBJECT:
			return	inObject2 == Layers::MOVING ||
					inObject2 == Layers::DEBRIS;
		// MOVING collides with moving and non-moving objects.
		case Layers::MOVING:
			return	inObject2 == Layers::MOVING ||
					inObject2 == Layers::NON_MOVING_WORLD ||
					inObject2 == Layers::NON_MOVING_OBJECT;
	
		// DEBRIS only collides with non-moving objects.
		case Layers::DEBRIS:
			return	inObject2 == Layers::NON_MOVING_WORLD || inObject2 == Layers::NON_MOVING_OBJECT;
		default:
			VJoltAssert( false );
			return false;
	}
};

private:
};

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class JoltBroadPhaseLayerInterface final : public JPH::BroadPhaseLayerInterface
{
public:
	JoltBroadPhaseLayerInterface()
	{
		// Create a mapping table from object to broad phase layer
		mObjectToBroadPhase[Layers::NON_MOVING_WORLD] = BroadPhaseLayers::NON_MOVING_WORLD;
		mObjectToBroadPhase[Layers::NON_MOVING_OBJECT] = BroadPhaseLayers::NON_MOVING_OBJECT;
		mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
		mObjectToBroadPhase[Layers::NO_COLLIDE] = BroadPhaseLayers::NO_COLLIDE;
		mObjectToBroadPhase[Layers::DEBRIS] = BroadPhaseLayers::DEBRIS;
	}

	uint GetNumBroadPhaseLayers() const override
	{
		return Layers::NUM_LAYERS;
	}

	JPH::BroadPhaseLayer GetBroadPhaseLayer( JPH::ObjectLayer inLayer ) const override
	{
		VJoltAssert( inLayer < Layers::NUM_LAYERS );
		return mObjectToBroadPhase[inLayer];
	}

#if defined( JPH_EXTERNAL_PROFILE ) || defined( JPH_PROFILE_ENABLED )
	const char *GetBroadPhaseLayerName( JPH::BroadPhaseLayer inLayer ) const override
	{
		switch ( (JPH::BroadPhaseLayer::Type)inLayer )
		{
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING_WORLD:	return "NON_MOVING_WORLD";
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING_OBJECT:	return "NON_MOVING_OBJECT";
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NO_COLLIDE:			return "NO_COLLIDE";
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::DEBRIS:				return "DEBRIS";
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:				return "MOVING";
		default:																VJoltAssert( false ); return "INVALID";
		}
	}
#endif

private:
	JPH::BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

class JoltObjectVsBroadPhaseLayerFilter final : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
	// Function that determines if two broadphase layers can collide
	bool ShouldCollide( JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2 ) const override
	{
		switch (inLayer1)
		{
		// NO_COLLIDE collides with nothing.
		case Layers::NO_COLLIDE:
			return false;

		// NON_MOVING collides with moving objects and debris.
		case Layers::NON_MOVING_WORLD:
		case Layers::NON_MOVING_OBJECT:
			return inLayer2 == BroadPhaseLayers::MOVING ||
				   inLayer2 == BroadPhaseLayers::DEBRIS;

		// MOVING collides with moving and non-moving objects.
		case Layers::MOVING:
			return inLayer2 == BroadPhaseLayers::MOVING ||
				   inLayer2 == BroadPhaseLayers::NON_MOVING_WORLD ||
				   inLayer2 == BroadPhaseLayers::NON_MOVING_OBJECT;
	
		// DEBRIS only collides with non-moving objects.
		case Layers::DEBRIS:
			return inLayer2 == BroadPhaseLayers::NON_MOVING_WORLD || inLayer2 == BroadPhaseLayers::NON_MOVING_OBJECT;

		default:
			VJoltAssert( false );
			return false;
		}
	}
private:
};

//-------------------------------------------------------------------------------------------------

static char s_szNextEnvironmentDumpPath[ MAX_PATH ];
static bool s_bShouldDumpEnvironmentClient = false;
static bool s_bShouldDumpEnvironmentServer = false;

CON_COMMAND( vjolt_environment_dump_client, "Dumps the next simulated environment to a .bin file" )
{
	s_bShouldDumpEnvironmentClient = true;
	V_strncpy( s_szNextEnvironmentDumpPath, args.Arg( 1 ), MAX_PATH );
}

CON_COMMAND( vjolt_environment_dump_server, "Dumps the next simulated environment to a .bin file" )
{
	s_bShouldDumpEnvironmentServer = true;
	V_strncpy( s_szNextEnvironmentDumpPath, args.Arg( 1 ), MAX_PATH );
}

//-------------------------------------------------------------------------------------------------

JoltBroadPhaseLayerInterface JoltPhysicsEnvironment::s_BroadPhaseLayerInterface;
JoltObjectVsBroadPhaseLayerFilter JoltPhysicsEnvironment::s_BroadPhaseFilter;
JoltObjectLayerPairFilter JoltPhysicsEnvironment::s_LayerPairFilter;

JoltPhysicsEnvironment::JoltPhysicsEnvironment()
	: m_ContactListener( m_PhysicsSystem )
{
	m_PerformanceParams.Defaults();

	m_PhysicsSystem.Init(
		kMaxBodies, kNumBodyMutexes, kMaxBodyPairs, kMaxContactConstraints,
		s_BroadPhaseLayerInterface, s_BroadPhaseFilter, s_LayerPairFilter);

	{
		JPH::PhysicsSettings settings = m_PhysicsSystem.GetPhysicsSettings();
		settings.mBaumgarte = vjolt_baumgarte_factor.GetFloat();
		m_PhysicsSystem.SetPhysicsSettings( settings );
	}

	// A body activation listener gets notified when bodies activate and go to sleep
	// Note that this is called from a job so whatever you do here needs to be thread safe.
	// Registering one is entirely optional.
	//m_PhysicsSystem.SetBodyActivationListener( &vars.bodyActivationListener );

	// A contact listener gets notified when bodies (are about to) collide, and when they separate again.
	// Note that this is called from a job so whatever you do here needs to be thread safe.
	// Registering one is entirely optional.
	m_PhysicsSystem.SetContactListener( &m_ContactListener );

	// Source clamps friction from 0 -> 1, so lets do that.
	m_PhysicsSystem.SetCombineFriction( []( const JPH::Body &inBody1, const JPH::SubShapeID &inSubShapeID1, const JPH::Body &inBody2, const JPH::SubShapeID &inSubShapeID2 ) -> float
	{
		return Clamp( inBody1.GetFriction() * inBody2.GetFriction(), 0.0f, 1.0f );
	} );

	// Jolt normally does max( x, y ) for resitution, but
	// Source's values expect them to be multiplied and clamped.
	m_PhysicsSystem.SetCombineRestitution( []( const JPH::Body &inBody1, const JPH::SubShapeID& inSubShapeID1, const JPH::Body &inBody2, const JPH::SubShapeID& inSubShapeID2 ) -> float
	{
		return Clamp( inBody1.GetRestitution() * inBody2.GetRestitution(), 0.0f, 1.0f );
	} );

	// Set our linear cast member
	m_bUseLinearCast = vjolt_linearcast.GetBool();
}

JoltPhysicsEnvironment::~JoltPhysicsEnvironment()
{
	// Clear any pending dead bodies.
	DeleteDeadObjects();

	// Clear out all our bodies.
	m_PhysicsSystem.GetBodies( m_CachedBodies );

	const int nCount = int ( m_CachedBodies.size() );
	for ( int i = 0; i < nCount; i++ )
	{
		JPH::Body *pBody = m_PhysicsSystem.GetBodyLockInterfaceNoLock().TryGetBody( m_CachedBodies[ i ] );
		JoltPhysicsObject *pObject = reinterpret_cast< JoltPhysicsObject * >( pBody->GetUserData() );
		RemoveBodyAndDeleteObject( pObject );
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetDebugOverlay( CreateInterfaceFn debugOverlayFactory )
{
	m_pDebugOverlay = nullptr;
	if ( debugOverlayFactory )
	{
		m_pDebugOverlay = (IVJoltDebugOverlay *)debugOverlayFactory( VJOLT_DEBUG_OVERLAY_VERSION, nullptr );

		JoltPhysicsInterface::GetInstance().SetDebugOverlay( m_pDebugOverlay );
	}
}

IVPhysicsDebugOverlay *JoltPhysicsEnvironment::GetDebugOverlay()
{
	// Slart: For some reason this is part of the vphysics interface, nothing ever uses it
	// outside of vphysics and we shouldn't either, we want to be able to pick between the
	// full debugoverlay or the vphysics one based on a compile-time parameter
	return nullptr;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetGravity( const Vector &gravityVector )
{
	JPH::Vec3 gravity = SourceToJolt::Distance( gravityVector );
	m_PhysicsSystem.SetGravity( gravity );
}

void JoltPhysicsEnvironment::GetGravity( Vector *pGravityVector ) const
{
	VJoltAssert( pGravityVector );
	*pGravityVector = JoltToSource::Distance( m_PhysicsSystem.GetGravity() );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetAirDensity( float density )
{
	// Josh: This is linear damping there is also angular damping...
	// Slart: Maybe we should set both to this value
	m_flAirDensity = density;
	Log_Stub( LOG_VJolt );
}

float JoltPhysicsEnvironment::GetAirDensity() const
{
	Log_Stub( LOG_VJolt );
	return m_flAirDensity;
}

//-------------------------------------------------------------------------------------------------

static objectparams_t NormalizeObjectParams( objectparams_t* pParams )
{
	objectparams_t params = *pParams;
	params.mass = Clamp( pParams->mass, VPHYSICS_MIN_MASS, VPHYSICS_MAX_MASS );

	return params;
}

//-------------------------------------------------------------------------------------------------

IPhysicsObject *JoltPhysicsEnvironment::CreatePolyObject( const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams )
{
	objectparams_t params = NormalizeObjectParams( pParams );

	const JPH::Shape* pShape = pCollisionModel->ToShape();
	if ( params.massCenterOverride )
	{
		JPH::Vec3 massCenterOverride = SourceToJolt::Distance( *params.massCenterOverride );
		pShape = CreateCOMOverrideShape( pShape, massCenterOverride );
	}

	JPH::BodyCreationSettings settings( pShape, SourceToJolt::Distance( position ), SourceToJolt::Angle( angles ), JPH::EMotionType::Dynamic, Layers::MOVING );
	settings.mMassPropertiesOverride.mMass = params.mass;
	//settings.mMassPropertiesOverride.mInertia = JPH::Mat44::sIdentity() * params.inertia;
	settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia; // JPH::EOverrideMassProperties::MassAndInertiaProvided;

	if ( m_bUseLinearCast )
		settings.mMotionQuality = JPH::EMotionQuality::LinearCast;

	JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	JPH::Body *pBody = bodyInterface.CreateBody( settings );
	bodyInterface.AddBody( pBody->GetID(), JPH::EActivation::DontActivate );

	return new JoltPhysicsObject( pBody, this, false, materialIndex, &params );
}

IPhysicsObject *JoltPhysicsEnvironment::CreatePolyObjectStatic( const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams )
{
	objectparams_t params = NormalizeObjectParams( pParams );

	JPH::BodyCreationSettings settings( pCollisionModel->ToShape(), SourceToJolt::Distance( position ), SourceToJolt::Angle( angles ), JPH::EMotionType::Static, Layers::NON_MOVING_WORLD );

	JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	JPH::Body *pBody = bodyInterface.CreateBody( settings );
	bodyInterface.AddBody( pBody->GetID(), JPH::EActivation::DontActivate );

	return new JoltPhysicsObject( pBody, this, true, materialIndex, &params );
}

IPhysicsObject *JoltPhysicsEnvironment::CreateSphereObject( float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic )
{
	objectparams_t params = NormalizeObjectParams( pParams );

	const JPH::Shape *pShape = new JPH::SphereShape( SourceToJolt::Distance( radius ) );
	if ( params.massCenterOverride )
	{
		JPH::Vec3 massCenterOverride = SourceToJolt::Distance( *params.massCenterOverride );
		pShape = CreateCOMOverrideShape( pShape, massCenterOverride );
	}

	JPH::EMotionType motionType = isStatic ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic;
	JPH::ObjectLayer objectLayer = isStatic ? Layers::NON_MOVING_WORLD : Layers::MOVING;

	JPH::BodyCreationSettings settings( pShape, SourceToJolt::Distance( position ), SourceToJolt::Angle( angles ), motionType, objectLayer );

	if ( !isStatic )
	{
		if ( m_bUseLinearCast )
			settings.mMotionQuality = JPH::EMotionQuality::LinearCast;

		settings.mMassPropertiesOverride.mMass = params.mass;
		//settings.mMassPropertiesOverride.mInertia = JPH::Mat44::sIdentity() * params.inertia;
		settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;//JPH::EOverrideMassProperties::MassAndInertiaProvided;
	}

	JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	JPH::Body *pBody = bodyInterface.CreateBody( settings );
	bodyInterface.AddBody( pBody->GetID(), JPH::EActivation::DontActivate );

	return new JoltPhysicsObject( pBody, this, isStatic, materialIndex, &params );
}

void JoltPhysicsEnvironment::DestroyObject( IPhysicsObject *pObject )
{
	if ( !pObject )
		return;

	JoltPhysicsObject *pJoltObject = static_cast<JoltPhysicsObject *>( pObject );

	if ( pJoltObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE )
	{
		// Object deleted twice.
		VJoltAssertMsg( 0, "Object deleted twice.\n" );
		return;
	}

	pJoltObject->AddCallbackFlags( CALLBACK_MARKED_FOR_DELETE );

	// If we are simulating or the delete queue is enabled, add it to the delete queue.
	// Otherwise, just delete it now.
	if ( m_bSimulating || m_bEnableDeleteQueue )
		m_pDeadObjects.push_back( pJoltObject );
	else
		RemoveBodyAndDeleteObject( pJoltObject );
}

//-------------------------------------------------------------------------------------------------

IPhysicsFluidController *JoltPhysicsEnvironment::CreateFluidController( IPhysicsObject *pFluidObject, fluidparams_t *pParams )
{
	JoltPhysicsObject *pJoltObject = static_cast< JoltPhysicsObject * >( pFluidObject );
	JoltPhysicsFluidController *pFluidController = new JoltPhysicsFluidController( &m_PhysicsSystem, pJoltObject, pParams );
	m_pPhysicsControllers.push_back( pFluidController );
	return pFluidController;
}

void JoltPhysicsEnvironment::DestroyFluidController( IPhysicsFluidController *pFluidController )
{
	JoltPhysicsFluidController *pInternalFluidController = static_cast<JoltPhysicsFluidController *>( pFluidController );
	Erase( m_pPhysicsControllers, pInternalFluidController );
	delete pInternalFluidController;
}

//-------------------------------------------------------------------------------------------------

class JoltPhysicsSpring final : public IPhysicsSpring, public IJoltObjectDestroyedListener
{
public:
	JoltPhysicsSpring( JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsObject *pObjectStart, JoltPhysicsObject *pObjectEnd, springparams_t *pParams );
	~JoltPhysicsSpring() override;

	void GetEndpoints( Vector *worldPositionStart, Vector *worldPositionEnd ) override;
	void SetSpringConstant( float flSpringConstant ) override;
	void SetSpringDamping( float flSpringDamping ) override;
	void SetSpringLength( float flSpringLength ) override;

	IPhysicsObject *GetStartObject() override;
	IPhysicsObject *GetEndObject() override;

	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject );

private:
	JPH::PhysicsSystem *m_pPhysicsSystem = nullptr;

	JoltPhysicsObject *m_pObjectStart = nullptr;
	JoltPhysicsObject *m_pObjectEnd = nullptr;

	JPH::DistanceConstraint *m_pConstraint = nullptr;
	bool m_OnlyStretch = false;
};

//-------------------------------------------------------------------------------------------------

JoltPhysicsSpring::JoltPhysicsSpring( JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsObject *pObjectStart, JoltPhysicsObject *pObjectEnd, springparams_t *pParams )
	: m_pPhysicsSystem( pPhysicsSystem )
	, m_pObjectStart( pObjectStart )
	, m_pObjectEnd( pObjectEnd )
	, m_OnlyStretch( pParams->onlyStretch )
{
	JPH::Body *refBody = m_pObjectStart->GetBody();
	JPH::Body *attBody = m_pObjectEnd->GetBody();

	JPH::DistanceConstraintSettings settings;
	settings.mSpace = pParams->useLocalPositions ? JPH::EConstraintSpace::LocalToBodyCOM : JPH::EConstraintSpace::WorldSpace;
	settings.mPoint1 = SourceToJolt::Distance( pParams->startPosition );
	settings.mPoint2 = SourceToJolt::Distance( pParams->endPosition );
	settings.mMinDistance = m_OnlyStretch ? 0.0f : SourceToJolt::Distance( pParams->naturalLength );
	settings.mMaxDistance = SourceToJolt::Distance( pParams->naturalLength );

	settings.mFrequency = GetSpringFrequency( pParams->constant, m_pObjectStart, m_pObjectEnd );
	// TODO(Josh): The damping values are normally fucking crazy like 5500 from Source... wtf is going on here.
	settings.mDamping = 0.0f;

	m_pConstraint = static_cast< JPH::DistanceConstraint * >( settings.Create( *refBody, *attBody ) );
	m_pConstraint->SetEnabled( true );

	m_pPhysicsSystem->AddConstraint( m_pConstraint );

	m_pObjectStart->AddDestroyedListener( this );
	m_pObjectEnd->AddDestroyedListener( this );
}

JoltPhysicsSpring::~JoltPhysicsSpring()
{
	if ( m_pObjectStart )
		m_pObjectStart->RemoveDestroyedListener( this );

	if ( m_pObjectEnd )
		m_pObjectEnd->RemoveDestroyedListener( this );

	m_pPhysicsSystem->RemoveConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsSpring::GetEndpoints( Vector *worldPositionStart, Vector *worldPositionEnd )
{
	// TODO(Josh): Implement this.
	Log_Stub( LOG_VJolt );

	if ( worldPositionStart )
		*worldPositionStart = vec3_origin;

	if ( worldPositionEnd )
		*worldPositionEnd = vec3_origin;
}

void JoltPhysicsSpring::SetSpringConstant( float flSpringConstant )
{
	m_pObjectStart->Wake();
	m_pObjectEnd->Wake();

	m_pConstraint->SetFrequency( GetSpringFrequency( flSpringConstant, m_pObjectStart, m_pObjectEnd ) );
}

void JoltPhysicsSpring::SetSpringDamping( float flSpringDamping )
{
	m_pObjectStart->Wake();
	m_pObjectEnd->Wake();

	//m_pConstraint->SetDamping( flSpringDamping );
}

void JoltPhysicsSpring::SetSpringLength( float flSpringLength )
{
	m_pObjectStart->Wake();
	m_pObjectEnd->Wake();

	float flLength = SourceToJolt::Distance( flSpringLength );
	m_pConstraint->SetDistance( m_OnlyStretch ? 0.0f : flLength, flLength );
}

//-------------------------------------------------------------------------------------------------

IPhysicsObject *JoltPhysicsSpring::GetStartObject()
{
	return m_pObjectStart;
}

IPhysicsObject *JoltPhysicsSpring::GetEndObject()
{
	return m_pObjectEnd;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsSpring::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	if ( pObject == m_pObjectStart )
		m_pObjectStart = nullptr;

	if ( pObject == m_pObjectEnd )
		m_pObjectEnd = nullptr;
}

//-------------------------------------------------------------------------------------------------

IPhysicsSpring *JoltPhysicsEnvironment::CreateSpring( IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd, springparams_t *pParams )
{
	JoltPhysicsObject *pJoltObjectStart = static_cast< JoltPhysicsObject *>( pObjectStart );
	JoltPhysicsObject *pJoltObjectEnd = static_cast< JoltPhysicsObject *>( pObjectEnd );

	return new JoltPhysicsSpring( &m_PhysicsSystem, pJoltObjectStart, pJoltObjectEnd, pParams );
}

void JoltPhysicsEnvironment::DestroySpring( IPhysicsSpring *pSpring )
{
	JoltPhysicsSpring *pJoltSpring = static_cast< JoltPhysicsSpring * >( pSpring );
	delete pJoltSpring;
}

//-------------------------------------------------------------------------------------------------

IPhysicsConstraint *JoltPhysicsEnvironment::CreateRagdollConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll )
{
	JoltPhysicsConstraint *pConstraint = new JoltPhysicsConstraint( this, pReferenceObject, pAttachedObject );
	pConstraint->InitialiseRagdoll( pGroup, ragdoll );
	return pConstraint;
}

IPhysicsConstraint *JoltPhysicsEnvironment::CreateHingeConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge )
{
	JoltPhysicsConstraint *pConstraint = new JoltPhysicsConstraint( this, pReferenceObject, pAttachedObject );
	pConstraint->InitialiseHinge( pGroup, hinge );
	return pConstraint;
}

IPhysicsConstraint *JoltPhysicsEnvironment::CreateFixedConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed )
{
	JoltPhysicsConstraint *pConstraint = new JoltPhysicsConstraint( this, pReferenceObject, pAttachedObject );
	pConstraint->InitialiseFixed( pGroup, fixed );
	return pConstraint;
}

IPhysicsConstraint *JoltPhysicsEnvironment::CreateSlidingConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding )
{
	JoltPhysicsConstraint *pConstraint = new JoltPhysicsConstraint( this, pReferenceObject, pAttachedObject );
	pConstraint->InitialiseSliding( pGroup, sliding );
	return pConstraint;
}

IPhysicsConstraint *JoltPhysicsEnvironment::CreateBallsocketConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket )
{
	JoltPhysicsConstraint *pConstraint = new JoltPhysicsConstraint( this, pReferenceObject, pAttachedObject );
	pConstraint->InitialiseBallsocket( pGroup, ballsocket );
	return pConstraint;
}

IPhysicsConstraint *JoltPhysicsEnvironment::CreatePulleyConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley )
{
	Log_Stub( LOG_VJolt );
	return nullptr;
}

IPhysicsConstraint *JoltPhysicsEnvironment::CreateLengthConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length )
{
	JoltPhysicsConstraint *pConstraint = new JoltPhysicsConstraint( this, pReferenceObject, pAttachedObject );
	pConstraint->InitialiseLength( pGroup, length );
	return pConstraint;
}

void JoltPhysicsEnvironment::DestroyConstraint( IPhysicsConstraint *pConstraint )
{
	if ( !pConstraint )
		return;

	JoltPhysicsConstraint *pJoltConstraint = static_cast< JoltPhysicsConstraint * >( pConstraint );
	if ( m_bWakeObjectsOnConstraintDeletion )
	{
		IPhysicsObject *pObjectRef = pJoltConstraint->GetReferenceObject();
		if ( pObjectRef )
			pObjectRef->Wake();

		IPhysicsObject *pObjectAtt = pJoltConstraint->GetAttachedObject();
		if ( pObjectAtt )
			pObjectAtt->Wake();
	}

	// Suprisingly, the quick delete thing only affects whether we wake
	// constraints or not.
	// It does not affect whether it goes in the delete queue.
	if ( m_bSimulating )
	{
		pJoltConstraint->Deactivate();
		m_pDeadConstraints.push_back( pJoltConstraint );
	}
	else
	{
		delete pJoltConstraint;
	}
}

//-------------------------------------------------------------------------------------------------

IPhysicsConstraintGroup *JoltPhysicsEnvironment::CreateConstraintGroup( const constraint_groupparams_t &groupParams )
{
	return new JoltPhysicsConstraintGroup;
}

void JoltPhysicsEnvironment::DestroyConstraintGroup( IPhysicsConstraintGroup *pGroup )
{
	delete static_cast<JoltPhysicsConstraintGroup *>( pGroup );
}

//-------------------------------------------------------------------------------------------------

IPhysicsShadowController *JoltPhysicsEnvironment::CreateShadowController( IPhysicsObject *pObject, bool allowTranslation, bool allowRotation )
{
	JoltPhysicsShadowController *pController = new JoltPhysicsShadowController( static_cast<JoltPhysicsObject *>( pObject ), allowTranslation, allowRotation );
	m_pPhysicsControllers.push_back( pController );
	return pController;
}

void JoltPhysicsEnvironment::DestroyShadowController( IPhysicsShadowController *pShadowController )
{
	JoltPhysicsShadowController *pController = static_cast< JoltPhysicsShadowController * >( pShadowController );
	Erase( m_pPhysicsControllers, pController );
	delete pController;
}

//-------------------------------------------------------------------------------------------------

IPhysicsPlayerController *JoltPhysicsEnvironment::CreatePlayerController( IPhysicsObject *pObject )
{
	JoltPhysicsPlayerController *pController = new JoltPhysicsPlayerController( static_cast<JoltPhysicsObject *>( pObject ) );
	m_pPhysicsControllers.push_back( pController );
	return pController;
}

void JoltPhysicsEnvironment::DestroyPlayerController( IPhysicsPlayerController *pPlayerController )
{
	JoltPhysicsPlayerController *pController = static_cast< JoltPhysicsPlayerController * >( pPlayerController );
	Erase( m_pPhysicsControllers, pController );
	delete pController;
}

//-------------------------------------------------------------------------------------------------

IPhysicsMotionController *JoltPhysicsEnvironment::CreateMotionController( IMotionEvent *pHandler )
{
	JoltPhysicsMotionController *pController = new JoltPhysicsMotionController( pHandler );
	m_pPhysicsControllers.push_back( pController );
	return pController;
}

void JoltPhysicsEnvironment::DestroyMotionController( IPhysicsMotionController *pController )
{
	JoltPhysicsMotionController *pJoltController = static_cast< JoltPhysicsMotionController * >( pController );
	Erase( m_pPhysicsControllers, pJoltController );
	delete pJoltController;
}

//-------------------------------------------------------------------------------------------------

IPhysicsVehicleController *JoltPhysicsEnvironment::CreateVehicleController( IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace )
{
	JoltPhysicsObject *pJoltCarBodyObject = static_cast< JoltPhysicsObject * >( pVehicleBodyObject );

	JoltPhysicsVehicleController *pController = new JoltPhysicsVehicleController( this, &m_PhysicsSystem, pJoltCarBodyObject, params, nVehicleType, pGameTrace );
	m_pPhysicsControllers.push_back( pController );
	return pController;
}

void JoltPhysicsEnvironment::DestroyVehicleController( IPhysicsVehicleController *pVehicleController )
{
	JoltPhysicsVehicleController *pJoltController = static_cast<JoltPhysicsVehicleController *>( pVehicleController );
	Erase( m_pPhysicsControllers, pJoltController );
	delete pJoltController;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetCollisionSolver( IPhysicsCollisionSolver *pSolver )
{
	m_ContactListener.SetGameSolver( pSolver );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::Simulate( float deltaTime )
{
	// Handle pausing the game...
	if ( deltaTime == 0.0f )
		return;

	// Grab our shared assets from the interface
	JPH::TempAllocator *tempAllocator = JoltPhysicsInterface::GetInstance().GetTempAllocator();
	JPH::JobSystem *jobSystem = JoltPhysicsInterface::GetInstance().GetJobSystem();

	// Clear any dead objects before running the simulation.
	DeleteDeadObjects();

	HandleDebugDumpingEnvironment( VJOLT_RETURN_ADDRESS() );

	m_bSimulating = true;

	// Funnily enough, VPhysics calls this BEFORE
	// doing the simulation...
	m_ContactListener.PostSimulationFrame();

	// Run pre-simulation controllers
	for ( IJoltPhysicsController *pController : m_pPhysicsControllers )
		pController->OnPreSimulate( deltaTime );

	const int nIntegrationSubSteps = vjolt_substeps_integration.GetInt();
	const int nCollisionSubSteps = vjolt_substeps_collision.GetInt();

	// If we haven't already, optimize the broadphase, currently this can only happen once per-environment
	if ( !m_bOptimizedBroadPhase )
	{
		m_PhysicsSystem.OptimizeBroadPhase();
		m_bOptimizedBroadPhase = true;

		if ( vjolt_initial_simulation.GetBool() )
		{
			// Do an initial simulation to settle objects down.
			static constexpr float InitialIterationTimescale = 1.0f / 60.0f;
			static constexpr int MaxInitialIterations = 1024;
			static constexpr int InitialSubSteps = 4;

			int nIterCount = 0;
			while ( m_PhysicsSystem.GetNumActiveBodies() && nIterCount < MaxInitialIterations )
			{
				m_PhysicsSystem.Update( InitialIterationTimescale, 1, InitialSubSteps, tempAllocator, jobSystem );
				nIterCount++;
			}
		}
		else
		{
			// Move things around!
			m_PhysicsSystem.Update( deltaTime, nCollisionSubSteps, nIntegrationSubSteps, tempAllocator, jobSystem );
		}
	}
	else
	{
		// Move things around!
		m_PhysicsSystem.Update( deltaTime, nCollisionSubSteps, nIntegrationSubSteps, tempAllocator, jobSystem );
	}
	m_ContactListener.FlushCallbacks();

	// Run post-simulation controllers
	for ( IJoltPhysicsController *pController : m_pPhysicsControllers )
		pController->OnPostSimulate( deltaTime );

	m_bSimulating = false;

	// If the delete queue is disabled, we only added to it during the simulation
	// ie. callbacks etc. So flush that now.
	if ( !m_bEnableDeleteQueue )
		DeleteDeadObjects();

#ifdef JPH_DEBUG_RENDERER
	JoltPhysicsDebugRenderer::GetInstance().RenderPhysicsSystem( m_PhysicsSystem );
#endif
}

bool JoltPhysicsEnvironment::IsInSimulation() const
{
	return m_bSimulating;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsEnvironment::GetSimulationTimestep() const
{
	return m_flStepTime;
}

void JoltPhysicsEnvironment::SetSimulationTimestep( float timestep )
{
	m_flStepTime = timestep;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsEnvironment::GetSimulationTime() const
{
	Log_Stub( LOG_VJolt );
	return 0.0f;
}

void JoltPhysicsEnvironment::ResetSimulationClock()
{
	Log_Stub( LOG_VJolt );
}

float JoltPhysicsEnvironment::GetNextFrameTime() const
{
	Log_Stub( LOG_VJolt );
	return 0.0f;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetCollisionEventHandler( IPhysicsCollisionEvent *pCollisionEvents )
{
	m_ContactListener.SetGameListener( pCollisionEvents );
}

void JoltPhysicsEnvironment::SetObjectEventHandler( IPhysicsObjectEvent *pObjectEvents )
{
	Log_Stub( LOG_VJolt );
}

void JoltPhysicsEnvironment::SetConstraintEventHandler( IPhysicsConstraintEvent *pConstraintEvents )
{
	m_pConstraintListener = pConstraintEvents;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetQuickDelete( bool bQuick )
{
	// Josh:
	// What a weird function, all this does is determine whether
	// to wake objects when the constraint gets deleted or not.
	m_bWakeObjectsOnConstraintDeletion = !bQuick;
}

//-------------------------------------------------------------------------------------------------

// GetActiveObjectCount and GetActiveObjects are always called in pairs
// and outside of the simulation.
//
// If there is a weird case where this is different (GMod lua on collision callbacks maybe?).
// Uncomment the locking code below.

int JoltPhysicsEnvironment::GetActiveObjectCount() const
{
	if ( !m_bActiveObjectCountFirst )
	{
		m_PhysicsSystem.GetActiveBodies( m_CachedActiveBodies );
		// Append any dirty static bodies we need the game side transforms
		// to be updated for.
		m_CachedActiveBodies.insert( m_CachedActiveBodies.end(), m_DirtyStaticBodies.begin(), m_DirtyStaticBodies.end() );
	}
	else
	{
		// If this is the first call, then some objects may have become
		// asleep from the initial simulation have their visuals not match where they are.
		m_PhysicsSystem.GetBodies( m_CachedActiveBodies );
		m_bActiveObjectCountFirst = false;
	}

	m_DirtyStaticBodies.clear();

	return int( m_CachedActiveBodies.size() );
}

void JoltPhysicsEnvironment::GetActiveObjects( IPhysicsObject **pOutputObjectList ) const
{
	const int nCount = int ( m_CachedActiveBodies.size() );
	for ( int i = 0; i < nCount; i++ )
	{
		JPH::Body *pBody = m_PhysicsSystem.GetBodyLockInterfaceNoLock().TryGetBody( m_CachedActiveBodies[ i ] );
		pOutputObjectList[ i ] = reinterpret_cast<IPhysicsObject *>( pBody->GetUserData() );
	}
}

const IPhysicsObject **JoltPhysicsEnvironment::GetObjectList( int *pOutputObjectCount ) const
{
	m_PhysicsSystem.GetBodies( m_CachedBodies );

	const int nCount = int ( m_CachedBodies.size() );

	if ( pOutputObjectCount )
		*pOutputObjectCount = nCount;

	m_CachedObjects.resize( nCount );
	for ( int i = 0; i < nCount; i++ )
	{
		JPH::Body *pBody = m_PhysicsSystem.GetBodyLockInterfaceNoLock().TryGetBody( m_CachedBodies[ i ] );
		m_CachedObjects[ i ] = reinterpret_cast< IPhysicsObject * >( pBody->GetUserData() );
	}

	return m_CachedObjects.data();
}

bool JoltPhysicsEnvironment::TransferObject( IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment )
{
	JoltPhysicsObject *pJoltObject = static_cast< JoltPhysicsObject * >( pObject );
	JoltPhysicsEnvironment *pJoltEnv = static_cast< JoltPhysicsEnvironment * >( pDestinationEnvironment );

	JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	bodyInterface.RemoveBody( pJoltObject->GetBodyID() );

	pJoltEnv->ObjectTransferHandOver( pJoltObject );
	pJoltObject->UpdateEnvironment( pJoltEnv );

	return true;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::CleanupDeleteList()
{
	DeleteDeadObjects();
}

void JoltPhysicsEnvironment::EnableDeleteQueue( bool enable )
{
	m_bEnableDeleteQueue = enable;
}

//-------------------------------------------------------------------------------------------------

class JoltStateRecorderSave final : public JPH::StateRecorder
{
public:
	JoltStateRecorderSave( ISave *pSave )
		: m_pSave( pSave )
	{
	}

	JoltStateRecorderSave( IRestore *pRestore )
		: m_pRestore( pRestore )
	{
	}

	void WriteBytes( const void* inData, size_t inNumBytes ) override
	{
		m_pSave->WriteData( reinterpret_cast< const char * >( inData ), int( inNumBytes ) );
	}

	void ReadBytes( void* outData, size_t inNumBytes ) override
	{
		m_pRestore->ReadData( reinterpret_cast< char * >( outData ), int( inNumBytes ), 0 );
	}

	bool IsEOF()	const override	{ return false; }
	bool IsFailed()	const override	{ return false; }

private:
	union
	{
		ISave		*m_pSave;
		IRestore	*m_pRestore;
	};
};

bool JoltPhysicsEnvironment::Save( const physsaveparams_t &params )
{
	JoltStateRecorderSave recorder( params.pSave );

	switch ( params.type )
	{
		default:
		case PIID_UNKNOWN:
			Log_Warning( LOG_VJolt, "Saving PIID_UNKNOWN is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSOBJECT:
		{
			JoltPhysicsObject *pObject = reinterpret_cast< JoltPhysicsObject * >( params.pObject );
			JPH::BodyCreationSettings bodyCreationSettings = pObject->GetBody()->GetBodyCreationSettings();

			recorder.Write( reinterpret_cast<uintptr_t>( pObject ) );
			bodyCreationSettings.SaveBinaryState( recorder );
			pObject->SaveObjectState( recorder );
			return true;
		}
		case PIID_IPHYSICSFLUIDCONTROLLER:
			// This just returns false in regular VPhysics.
			return false;
		case PIID_IPHYSICSSPRING:
			Log_Warning( LOG_VJolt, "Saving PIID_IPHYSICSSPRING is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSCONSTRAINTGROUP:
			Log_Warning( LOG_VJolt, "Saving PIID_IPHYSICSCONSTRAINTGROUP is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSCONSTRAINT:
		{
			JoltPhysicsConstraint *pConstraint = reinterpret_cast< JoltPhysicsConstraint * >( params.pObject );
			JoltPhysicsObject *pRefObject = reinterpret_cast< JoltPhysicsObject * >( pConstraint->GetReferenceObject() );
			JoltPhysicsObject *pAttObject = reinterpret_cast< JoltPhysicsObject * >( pConstraint->GetAttachedObject() );

			recorder.Write( reinterpret_cast<uintptr_t>( pConstraint ) );
			recorder.Write( reinterpret_cast<uintptr_t>( pRefObject ) );
			recorder.Write( reinterpret_cast<uintptr_t>( pAttObject ) );
			pConstraint->SaveConstraintSettings( recorder );
			return true;
		}
		case PIID_IPHYSICSSHADOWCONTROLLER:
			Log_Warning( LOG_VJolt, "Saving PIID_IPHYSICSSHADOWCONTROLLER is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSPLAYERCONTROLLER:
			// This just returns false in regular VPhysics.
			return false;
		case PIID_IPHYSICSMOTIONCONTROLLER:
			Log_Warning( LOG_VJolt, "Saving PIID_IPHYSICSMOTIONCONTROLLER is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSVEHICLECONTROLLER:
			Log_Warning( LOG_VJolt, "Saving PIID_IPHYSICSVEHICLECONTROLLER is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSGAMETRACE:
			Log_Warning( LOG_VJolt, "Saving PIID_IPHYSICSGAMETRACE is unsupported right now.\n" );
			return false;
	}
}

void JoltPhysicsEnvironment::PreRestore( const physprerestoreparams_t &params )
{
	m_SaveRestorePointerMap.clear();

	for ( int i = 0; i < params.recreatedObjectCount; i++ )
		AddPhysicsSaveRestorePointer(
			reinterpret_cast< uintp >( params.recreatedObjectList[ i ].pOldObject ),
			params.recreatedObjectList[ i ].pNewObject );
}

bool JoltPhysicsEnvironment::Restore( const physrestoreparams_t &params )
{
	JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	JoltStateRecorderSave recorder( params.pRestore );

	switch ( params.type )
	{
		default:
		case PIID_UNKNOWN:
			Log_Warning( LOG_VJolt, "Restoring PIID_UNKNOWN is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSOBJECT:
		{
			const JPH::Shape* pShape = params.pCollisionModel->ToShape();
			JPH::BodyCreationSettings bodyCreationSettings;
			uintp originalPtr;

			recorder.Read( originalPtr );
			bodyCreationSettings.RestoreBinaryState( recorder );
			bodyCreationSettings.SetShape( pShape );
			JPH::Body *pBody = bodyInterface.CreateBody( bodyCreationSettings );
			bodyInterface.AddBody( pBody->GetID(), JPH::EActivation::DontActivate );
			JoltPhysicsObject *pJoltObject = new JoltPhysicsObject( pBody, this, params.pGameData, recorder );

			*params.ppObject = reinterpret_cast< void * >( pJoltObject );
			AddPhysicsSaveRestorePointer( originalPtr, pJoltObject );
			return true;
		}
		case PIID_IPHYSICSFLUIDCONTROLLER:
			// Given saving this just returns false, this should never happen.
			Log_Warning( LOG_VJolt, "Restoring PIID_IPHYSICSFLUIDCONTROLLER is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSSPRING:
			Log_Warning( LOG_VJolt, "Restoring PIID_IPHYSICSSPRING is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSCONSTRAINTGROUP:
			Log_Warning( LOG_VJolt, "Restoring PIID_IPHYSICSCONSTRAINTGROUP is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSCONSTRAINT:
		{
			uintp constraintPtr, refObjectPtr, attObjectPtr;
			constraintType_t type;

			recorder.Read( constraintPtr );
			recorder.Read( refObjectPtr );
			recorder.Read( attObjectPtr );
			
			recorder.Read( type );
			JPH::ConstraintSettings::ConstraintResult result = JPH::ConstraintSettings::sRestoreFromBinaryState( recorder );

			if ( result.HasError() )
			{
				Log_Warning( LOG_VJolt, "Error restoring constraint: %s.\n", result.GetError().c_str() );
				return false;
			}

			JoltPhysicsObject* pRefObject = LookupPhysicsSaveRestorePointer< JoltPhysicsObject >( refObjectPtr );
			JoltPhysicsObject* pAttObject = LookupPhysicsSaveRestorePointer< JoltPhysicsObject >( attObjectPtr );

			const JPH::TwoBodyConstraintSettings *pSettings = static_cast< const JPH::TwoBodyConstraintSettings * >( result.Get().GetPtr() );
			JPH::Constraint *pConstraint = bodyInterface.CreateConstraint( pSettings, pRefObject->GetBodyID(), pAttObject->GetBodyID() );
			pConstraint->RestoreState( recorder );
			m_PhysicsSystem.AddConstraint( pConstraint );
			JoltPhysicsConstraint* pJoltConstraint = new JoltPhysicsConstraint( this, pRefObject, pAttObject, type, pConstraint, params.pGameData );
			*params.ppObject = reinterpret_cast<void*>( pJoltConstraint );
			AddPhysicsSaveRestorePointer( constraintPtr, pJoltConstraint );
			return true;
		}
		case PIID_IPHYSICSSHADOWCONTROLLER:
			Log_Warning( LOG_VJolt, "Restoring PIID_IPHYSICSSHADOWCONTROLLER is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSPLAYERCONTROLLER:
			// Given saving this just returns false, this should never happen.
			Log_Warning( LOG_VJolt, "Restoring PIID_IPHYSICSPLAYERCONTROLLER is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSMOTIONCONTROLLER:
			Log_Warning( LOG_VJolt, "Restoring PIID_IPHYSICSMOTIONCONTROLLER is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSVEHICLECONTROLLER:
			Log_Warning( LOG_VJolt, "Restoring PIID_IPHYSICSVEHICLECONTROLLER is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSGAMETRACE:
			Log_Warning( LOG_VJolt, "Restoring PIID_IPHYSICSGAMETRACE is unsupported right now.\n" );
			return false;
	}
}

void JoltPhysicsEnvironment::PostRestore()
{
	Log_Stub( LOG_VJolt );
	m_SaveRestorePointerMap.clear();
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsEnvironment::IsCollisionModelUsed( CPhysCollide *pCollide ) const
{
	// Josh: This is only used in debug code.
	return false;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::TraceRay( const Ray_t &ray, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace )
{
	// Josh: This does nothing in regular vphysics.
}

void JoltPhysicsEnvironment::SweepCollideable( const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd,
	const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace )
{
	// Josh: This does nothing in regular vphysics.
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::GetPerformanceSettings( physics_performanceparams_t *pOutput ) const
{
	if ( pOutput )
		*pOutput = m_PerformanceParams;
}

void JoltPhysicsEnvironment::SetPerformanceSettings( const physics_performanceparams_t *pSettings )
{
	if ( pSettings )
	{
		m_PerformanceParams = *pSettings;

		// Normalize these values to match VPhysics behaviour.
		m_PerformanceParams.minFrictionMass = Clamp( m_PerformanceParams.minFrictionMass, 1.0f, VPHYSICS_MAX_MASS );
		m_PerformanceParams.maxFrictionMass = Clamp( m_PerformanceParams.maxFrictionMass, 1.0f, VPHYSICS_MAX_MASS );
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::ReadStats( physics_stats_t *pOutput )
{
	Log_Stub( LOG_VJolt );
}

void JoltPhysicsEnvironment::ClearStats()
{
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

// StateRecorder implementation that gathers the size required to save a body's state
class VJoltStateSizeRecorder final : public JPH::StateRecorder
{
public:
	// StreamIn
	void ReadBytes( void *outData, size_t inNumBytes ) override {}
	bool IsEOF() const override { return false; }

	// StreamOut
	void WriteBytes( const void *inData, size_t inNumBytes )
	{
		saveSize += inNumBytes;
	}

	// Both
	bool IsFailed() const override { return false; }

public:
	size_t saveSize = 0;
};

unsigned int JoltPhysicsEnvironment::GetObjectSerializeSize( IPhysicsObject *pObject ) const
{
	JoltPhysicsObject *pJoltObject = static_cast<JoltPhysicsObject *>( pObject );
	const JPH::Body *pBody = pJoltObject->GetBody();

	VJoltStateSizeRecorder recorder;

	pBody->SaveState( recorder );

	pJoltObject->SaveObjectState( recorder );

	// Larger than we actually need, but it doesn't matter
	return static_cast<unsigned int>( sizeof( void * ) + recorder.saveSize + sizeof( JPH::BodyCreationSettings ) );
}

void JoltPhysicsEnvironment::SerializeObjectToBuffer( IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize )
{
	JoltPhysicsObject *pJoltObject = static_cast<JoltPhysicsObject *>( pObject );
	const JPH::Body *pBody = pJoltObject->GetBody();

	VJoltStateRecorder recorder( pBuffer, bufferSize );

	// Write shape
	recorder.Write( const_cast<void *>( reinterpret_cast<const void *>( pBody->GetShape() ) ) );

	// Write body creation settings
	JPH::BodyCreationSettings bodyCreationSettings = pBody->GetBodyCreationSettings();
	bodyCreationSettings.SaveBinaryState( recorder );

	pJoltObject->SaveObjectState( recorder );
}

IPhysicsObject *JoltPhysicsEnvironment::UnserializeObjectFromBuffer( void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions )
{
	VJoltStateRecorder recorder( pBuffer, bufferSize, CUtlBuffer::READ_ONLY );

	// Read shape
	JPH::Shape *pShape;
	recorder.Read< JPH::Shape* >( pShape );

	// Read body creation settings
	JPH::BodyCreationSettings bodyCreationSettings;
	bodyCreationSettings.RestoreBinaryState( recorder );

	bodyCreationSettings.SetShape( pShape );

	JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	JPH::Body *pBody = bodyInterface.CreateBody( bodyCreationSettings );
	bodyInterface.AddBody( pBody->GetID(), JPH::EActivation::Activate );

	JoltPhysicsObject *pJoltObject = new JoltPhysicsObject( pBody, this, pGameData, recorder );
	pJoltObject->EnableCollisions( enableCollisions );
	return pJoltObject;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::EnableConstraintNotify( bool bEnable )
{
	m_EnableConstraintNotify = bEnable;
}

void JoltPhysicsEnvironment::DebugCheckContacts()
{
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetAlternateGravity( const Vector &gravityVector )
{
	Log_Stub( LOG_VJolt );
}

void JoltPhysicsEnvironment::GetAlternateGravity( Vector *pGravityVector ) const
{
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsEnvironment::GetDeltaFrameTime( int maxTicks ) const
{
	Log_Stub( LOG_VJolt );
	// TODO(Josh): wtf to do here?
	return m_flStepTime * maxTicks;
}

void JoltPhysicsEnvironment::ForceObjectsToSleep( IPhysicsObject **pList, int listCount )
{
	for ( int i = 0; i < listCount; i++ )
		pList[ i ]->Sleep();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetPredicted( bool bPredicted )
{
	Log_Stub( LOG_VJolt );
}

bool JoltPhysicsEnvironment::IsPredicted()
{
	Log_Stub( LOG_VJolt );
	return false;
}

void JoltPhysicsEnvironment::SetPredictionCommandNum( int iCommandNum )
{
	Log_Stub( LOG_VJolt );
}

int JoltPhysicsEnvironment::GetPredictionCommandNum()
{
	Log_Stub( LOG_VJolt );
	return 0;
}

void JoltPhysicsEnvironment::DoneReferencingPreviousCommands( int iCommandNum )
{
	Log_Stub( LOG_VJolt );
}

void JoltPhysicsEnvironment::RestorePredictedSimulation()
{
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::DestroyCollideOnDeadObjectFlush( CPhysCollide *pCollide )
{
	// If this collide is part of a dead object, add it to a queue to delete.
	for ( JoltPhysicsObject *pObject : m_pDeadObjects )
	{
		if ( pObject->GetCollide() == pCollide )
		{
			if ( !VectorContains( m_pDeadObjectCollides, pCollide ) )
				m_pDeadObjectCollides.push_back( pCollide );
			return;
		}
	}

	// Otherwise, just delete it now.
	JoltPhysicsCollision::GetInstance().DestroyCollide( pCollide );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::ObjectTransferHandOver( JoltPhysicsObject *pObject )
{
	JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	bodyInterface.AddBody( pObject->GetBodyID(), JPH::EActivation::Activate );
}

void JoltPhysicsEnvironment::NotifyConstraintDisabled( JoltPhysicsConstraint* pConstraint )
{
	if ( m_pConstraintListener && m_EnableConstraintNotify )
		m_pConstraintListener->ConstraintBroken( pConstraint );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::AddDirtyStaticBody( const JPH::BodyID &id )
{
	m_DirtyStaticBodies.push_back( id );
}

void JoltPhysicsEnvironment::RemoveDirtyStaticBody( const JPH::BodyID &id )
{
	Erase( m_DirtyStaticBodies, id );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::RemoveBodyAndDeleteObject( JoltPhysicsObject *pObject )
{
	JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	bodyInterface.RemoveBody( pObject->GetBodyID() );
	delete pObject;
}

void JoltPhysicsEnvironment::DeleteDeadObjects()
{
	for ( JoltPhysicsObject *pObject : m_pDeadObjects )
		RemoveBodyAndDeleteObject( pObject );
	m_pDeadObjects.clear();

	for ( JoltPhysicsConstraint *pConstraint : m_pDeadConstraints )
		delete pConstraint;
	m_pDeadConstraints.clear();

	for ( CPhysCollide *pCollide : m_pDeadObjectCollides )
		JoltPhysicsCollision::GetInstance().DestroyCollide( pCollide );
	m_pDeadObjectCollides.clear();
}

//-------------------------------------------------------------------------------------------------

template < typename T >
void JoltPhysicsEnvironment::AddPhysicsSaveRestorePointer( uintp oldPtr, T* newPtr )
{
	VJoltAssert( oldPtr != 0 );
	VJoltAssert( newPtr != 0 );

	m_SaveRestorePointerMap[ oldPtr ] = reinterpret_cast< void * >( newPtr );
}

template < typename T >
T *JoltPhysicsEnvironment::LookupPhysicsSaveRestorePointer( uintp oldPtr )
{
	if ( !oldPtr )
		return nullptr;

	auto iter = m_SaveRestorePointerMap.find( oldPtr );
	if ( iter == m_SaveRestorePointerMap.end() )
		return nullptr;

	return reinterpret_cast< T * >( iter->second );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::HandleDebugDumpingEnvironment( void *pReturnAddress )
{
	if ( !s_bShouldDumpEnvironmentClient && !s_bShouldDumpEnvironmentServer )
		return;

	// Josh:
	// Check if we were called by either server.dll or client.dll
	// so we get the right environment.
	// (Client will only have ragdolls etc)
	char szModulePath[MAX_PATH];
	GetCallingFunctionModulePath( pReturnAddress, szModulePath, MAX_PATH );
	const char* pszModuleFileName = V_UnqualifiedFileName( szModulePath );

	bool bIsServerDumping = StringHasPrefix( pszModuleFileName, "server" ) && s_bShouldDumpEnvironmentServer;
	bool bIsClientDumping = StringHasPrefix( pszModuleFileName, "client" ) && s_bShouldDumpEnvironmentClient;

	VJoltAssertMsg( !StringHasPrefix( pszModuleFileName, "vphysics" ), "Should never get vphysics as the calling module, this only looks up 1 call in the stack." );

	if ( !bIsServerDumping && !bIsClientDumping )
		return;

	JoltStateRecorderFile recorder( s_szNextEnvironmentDumpPath, false );
	if ( recorder.IsValid() )
	{
		JPH::PhysicsScene scene;
		scene.FromPhysicsSystem( &m_PhysicsSystem );
		scene.SaveBinaryState( recorder, true, true );
	}
	else
		Log_Warning( LOG_VJolt, "Failed to open stream to dump environment to file %s!\n", s_szNextEnvironmentDumpPath );

	s_bShouldDumpEnvironmentClient = false;
	s_bShouldDumpEnvironmentServer = false;
}
