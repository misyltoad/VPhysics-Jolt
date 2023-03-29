
#include "cbase.h"

#include "vjolt_layers.h"
#include "vjolt_collide.h"
#include "vjolt_object.h"
#include "vjolt_environment.h"
#include "vjolt_surfaceprops.h"

#include "vjolt_controller_fluid.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

// Josh: The surfacePlane in fluidparams_t is specified in world-space, but we need to translate
// that to the pFluidObject's local-space so we can handle the fluid object moving
// for eg. func_water_analog.
static cplane_t PlaneToLocalSpace( JoltPhysicsObject* pFluidObject, Vector4D worldSurfacePlane )
{
	const cplane_t worldPlane = { worldSurfacePlane.AsVector3D(), worldSurfacePlane[3] };

	matrix3x4_t objectToWorld;
	pFluidObject->GetPositionMatrix( &objectToWorld );

	cplane_t localPlane;
	MatrixITransformPlane( objectToWorld, worldPlane, localPlane );
	return localPlane;
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsFluidController::JoltPhysicsFluidController( JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsObject *pFluidObject, const fluidparams_t *pParams )
	: m_pPhysicsSystem( pPhysicsSystem )
	, m_pFluidObject( pFluidObject )
	, m_Params( *pParams )
	, m_LocalPlane( PlaneToLocalSpace( pFluidObject, pParams->surfacePlane ) )
{
	m_pFluidObject->BecomeTrigger();
	m_pFluidObject->SetFluidController( this );
	m_pFluidObject->AddDestroyedListener( this );
}

JoltPhysicsFluidController::~JoltPhysicsFluidController()
{
	ClearCachedObjectsInShape();

	if ( m_pFluidObject )
	{
		m_pFluidObject->RemoveDestroyedListener( this );
		m_pFluidObject->SetFluidController( nullptr );
		m_pFluidObject->RemoveTrigger();
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFluidController::SetGameData( void *pGameData )
{
	m_Params.pGameData = pGameData;
}

void *JoltPhysicsFluidController::GetGameData() const
{
	return m_Params.pGameData;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFluidController::GetSurfacePlane( Vector *pNormal, float *pDist ) const
{
	const cplane_t worldPlane = GetSurfacePlane();

	if ( pNormal )
		*pNormal = worldPlane.normal;

	if ( pDist )
		*pDist = worldPlane.dist;
}

float JoltPhysicsFluidController::GetDensity() const
{
	// TODO(Josh): We know the material density, but what units should this be in?
	return 1.0f;
}

void JoltPhysicsFluidController::WakeAllSleepingObjects()
{
	for ( JoltPhysicsObject *pObject : m_ObjectsInShape )
		pObject->Wake();
}

int JoltPhysicsFluidController::GetContents() const
{
	return m_Params.contents;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFluidController::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	if ( pObject == m_pFluidObject )
		m_pFluidObject = nullptr;

	Erase( m_ObjectsInShape, pObject );
}

//-------------------------------------------------------------------------------------------------

static JPH::Vec3 ProjectPoint( const JPH::Plane &plane, JPH::Vec3Arg point )
{
	return point - plane.SignedDistance(point) * plane.GetNormal();
}

// Applies buoyancy to any body that intersects with the water shape
class SourceFluidCollector : public JPH::CollideShapeCollector
{
public:
	SourceFluidCollector( JPH::PhysicsSystem *inSystem, std::vector<JoltPhysicsObject *> &objectsInShape, const JPH::Plane &inSurface, float inDeltaTime, const fluidparams_t &params, float flDensity )
		: m_pPhysicsSystem	( inSystem )
		, m_ObjectsInShape	( objectsInShape )
		, m_Surface			( inSurface )
		, m_DeltaTime		( inDeltaTime )
		, m_Params			( params )
		, m_flDensity		( flDensity ) {}

	void AddHit( const ResultType &inResult ) override
	{
		const JPH::BodyID inBodyID = inResult.mBodyID2;

		JPH::BodyLockWrite lock( m_pPhysicsSystem->GetBodyLockInterface(), inBodyID );
		JPH::Body &body = lock.GetBody();
		JoltPhysicsObject *pObject = reinterpret_cast<JoltPhysicsObject *>( body.GetUserData() );

		// Don't do fluid simulation for these.
		if ( pObject->GetShadowController() || !( pObject->GetCallbackFlags() & CALLBACK_DO_FLUID_SIMULATION ) )
			return;

		m_ObjectsInShape.push_back( pObject );

		// Josh:
		// The buoyancy ratio in Source works like this:
		//   fluid_density = m_flDensity * pObject->GetBuoyancyRatio()
		// but in Jolt, it gets the fluid density like this (you pass in inBuoyancy)
		//   fluid_density = inBuoyancy / (total_volume * inverse_mass);
		// so with some rearranging...
		//   inBuoyancy = fluid_density * (total_volume * inverse_mass);
		const float flFluidDensity = m_flDensity * pObject->GetBuoyancyRatio();
		float inBuoyancy = flFluidDensity * pObject->GetBody()->GetShape()->GetVolume() * pObject->GetInvMass();
		if ( body.IsActive() )
		{
			// Project (0, 0, 0) onto plane to get a point on it.
			JPH::Vec3 point = ProjectPoint( m_Surface, JPH::Vec3::sZero() );
			JPH::Vec3 normal = m_Surface.GetNormal();
			body.ApplyBuoyancyImpulse( point, normal, inBuoyancy, m_Params.damping, 0.1f, SourceToJolt::Distance( m_Params.currentVelocity ), m_pPhysicsSystem->GetGravity(), m_DeltaTime );
		}
	}

private:
	JPH::PhysicsSystem *				m_pPhysicsSystem;
	std::vector<JoltPhysicsObject *>	&m_ObjectsInShape;
	JPH::Plane							m_Surface;
	float								m_DeltaTime;
	const fluidparams_t					&m_Params;
	float								m_flDensity;
};

void JoltPhysicsFluidController::OnPreSimulate( float deltaTime )
{
	// Clear out our last list of items
	ClearCachedObjectsInShape();

	if ( !m_pFluidObject )
		return;

	const cplane_t surfacePlane = GetSurfacePlane();

	JPH::Plane surface = JPH::Plane(
		SourceToJolt::Unitless( surfacePlane.normal ),
		SourceToJolt::Distance( -surfacePlane.dist ) );

	const float flDensity = m_pFluidObject->GetMaterialDensity();

	SourceFluidCollector collector( m_pPhysicsSystem, m_ObjectsInShape, surface, deltaTime, m_Params, flDensity );

	// Apply buoyancy to all bodies that intersect with the water
	JPH::CollideShapeSettings collideSettings;
	collideSettings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;

	// Ignore my own body
	// TODO(Josh): Should we use the SourceHitFilter from the player controller here?
	// May make more sense and respect ShouldCollide. Not sure if regular VPhysics does here.
	JPH::IgnoreSingleBodyFilter body_filter( m_pFluidObject->GetBodyID() );

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	JPH::Mat44 queryTransform = bodyInterface.GetCenterOfMassTransform( m_pFluidObject->GetBodyID() );

	const JPH::Shape *pShape = m_pFluidObject->GetCollide()->ToShape();

	m_pPhysicsSystem->GetNarrowPhaseQueryNoLock().CollideShape(
		pShape, JPH::Vec3::sReplicate( 1.0f ), queryTransform, collideSettings, JPH::Vec3::sZero(), collector,
		JPH::SpecifiedBroadPhaseLayerFilter( BroadPhaseLayers::MOVING ), JPH::SpecifiedObjectLayerFilter( Layers::MOVING ), body_filter );

	for ( JoltPhysicsObject *pObject : m_ObjectsInShape )
		pObject->AddDestroyedListener( this );
}

//-------------------------------------------------------------------------------------------------

cplane_t JoltPhysicsFluidController::GetSurfacePlane() const
{
	if ( !m_pFluidObject )
		return cplane_t{};

	matrix3x4_t objectToWorld;
	m_pFluidObject->GetPositionMatrix( &objectToWorld );

	cplane_t worldPlane;
	MatrixTransformPlane( objectToWorld, m_LocalPlane, worldPlane );

	return worldPlane;
}

void JoltPhysicsFluidController::ClearCachedObjectsInShape()
{
	// TODO(Josh):  This could maybe be made more efficient by having two vectors
	// and only updating the listeners on the ones we need to, then std::move-ing.
	for ( JoltPhysicsObject *pObject : m_ObjectsInShape )
		pObject->RemoveDestroyedListener( this );

	m_ObjectsInShape.clear();
}
