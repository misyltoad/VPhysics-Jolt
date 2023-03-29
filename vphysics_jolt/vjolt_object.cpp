//=================================================================================================
//
// A physics object, implemented as a wrapper over JPH::Body
// Every tangible object in the game has one of these
//
//=================================================================================================

#include "cbase.h"

#include "vjolt_collide.h"
#include "vjolt_surfaceprops.h"
#include "vjolt_friction.h"
#include "vjolt_environment.h"
#include "vjolt_layers.h"
#include "vjolt_controller_shadow.h"

#include "vjolt_object.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

JoltPhysicsObject::JoltPhysicsObject( JPH::Body *pBody, JoltPhysicsEnvironment *pEnvironment, bool bStatic, int nMaterialIndex, const objectparams_t *pParams )
	: m_pBody( pBody )
	, m_pEnvironment( pEnvironment )
	, m_pPhysicsSystem( pEnvironment->GetPhysicsSystem() )
	, m_bStatic( bStatic )
	, m_flCachedMass( pParams->mass )
	, m_flCachedInvMass( m_flCachedMass ? 1.0f / m_flCachedMass : 0.0f )
	, m_pGameData( pParams->pGameData )
	, m_materialIndex( Max( nMaterialIndex, 0 ) ) // Sometimes we get passed -1.
	, m_flVolume( pParams->volume )
{
	// Josh:
	// Assert that m_pGameData is the first element, some games
	// decide to just read this directly by offsetting by the vtable size
	// instead of calling GetGameData().
	static_assert( offsetof( JoltPhysicsObject, m_pGameData ) == sizeof( void * ) );

	// Set the body's userdata as ourselves
	pBody->SetUserData( reinterpret_cast<uint64>( this ) );
	if ( !m_pBody->IsStatic() )
	{
		JPH::MotionProperties* pMotionProperties = m_pBody->GetMotionProperties();
		pMotionProperties->SetLinearDamping( pParams->damping );
		pMotionProperties->SetAngularDamping( pParams->rotdamping );
	}

	UpdateMaterialProperties();
}

JoltPhysicsObject::JoltPhysicsObject( JPH::Body *pBody, JoltPhysicsEnvironment *pEnvironment, void *pGameData, JPH::StateRecorder &recorder )
	: m_pBody( pBody )
	, m_pEnvironment( pEnvironment )
	, m_pPhysicsSystem( pEnvironment->GetPhysicsSystem() )
	, m_pGameData( pGameData )
{
	RestoreObjectState( recorder );
}

JoltPhysicsObject::~JoltPhysicsObject()
{
	RemoveShadowController();

	// Josh:
	// Iterate over this in reverse as we could remove a listener from inside this callback
	for ( int i = m_destroyedListeners.Count() - 1; i >= 0; i-- )
		m_destroyedListeners[ i ]->OnJoltPhysicsObjectDestroyed( this );

	m_pEnvironment->RemoveDirtyStaticBody( GetBodyID() );

	JPH::BodyInterface& bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.DestroyBody( GetBodyID() );
}

bool JoltPhysicsObject::IsStatic() const
{
	// Whether this a static body to VPhysics and not
	// to Jolt (static or motion disabled) itself.
	return m_bStatic;
}

bool JoltPhysicsObject::IsAsleep() const
{
	return !m_pBody->IsActive();
}

bool JoltPhysicsObject::IsTrigger() const
{
	return m_pBody->IsSensor();
}

bool JoltPhysicsObject::IsFluid() const
{
	return m_pFluidController != nullptr;
}

bool JoltPhysicsObject::IsHinged() const
{
	Log_Stub( LOG_VJolt );
	return false;
}

bool JoltPhysicsObject::IsCollisionEnabled() const
{
	return m_bCachedCollisionEnabled;
}

bool JoltPhysicsObject::IsGravityEnabled() const
{
	if ( !m_pBody->IsStatic() )
	{
		JPH::MotionProperties* pMotionProperties = m_pBody->GetMotionProperties();
		return pMotionProperties->GetGravityFactor() != 0.0f;
	}

	return false;
}

bool JoltPhysicsObject::IsDragEnabled() const
{
	Log_Stub( LOG_VJolt );
	return false;
}

bool JoltPhysicsObject::IsMotionEnabled() const
{
	return !m_bPinned;
}

bool JoltPhysicsObject::IsMoveable() const
{
	return IsMotionEnabled() && !IsStatic();
}

bool JoltPhysicsObject::IsAttachedToConstraint( bool bExternalOnly ) const
{
	Log_Stub( LOG_VJolt );
	return false;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::EnableCollisions( bool enable )
{
	// Josh:
	// When collisions are disabled, they disable for EVERYTHING, including against the world.
	m_bCachedCollisionEnabled = enable;
	UpdateLayer();
}

void JoltPhysicsObject::EnableGravity( bool enable )
{
	if ( !m_pBody->IsStatic() )
	{
		JPH::MotionProperties* pMotionProperties = m_pBody->GetMotionProperties();
		pMotionProperties->SetGravityFactor( enable ? 1.0f : 0.0f );
	}
}

void JoltPhysicsObject::EnableDrag( bool enable )
{
	Log_Stub( LOG_VJolt );
}

void JoltPhysicsObject::EnableMotion( bool enable )
{
	if ( IsStatic() )
		return;

	const bool bPinned = !enable;

	if ( m_bPinned == bPinned )
		return;

	m_bPinned = bPinned;
	UpdateLayer();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetGameData( void *pGameData )
{
	m_pGameData = pGameData;
}

void *JoltPhysicsObject::GetGameData() const
{
	return m_pGameData;
}

void JoltPhysicsObject::SetGameFlags( unsigned short userFlags )
{
	m_gameFlags = userFlags;
}

unsigned short JoltPhysicsObject::GetGameFlags() const 
{
	return m_gameFlags;
}

void JoltPhysicsObject::SetGameIndex( unsigned short gameIndex )
{
	m_gameIndex = gameIndex;
}

unsigned short JoltPhysicsObject::GetGameIndex() const
{
	return m_gameIndex;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetCallbackFlags( unsigned short callbackflags )
{
	m_callbackFlags = callbackflags;
}

unsigned short JoltPhysicsObject::GetCallbackFlags() const
{
	return m_callbackFlags;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::Wake()
{
	if ( !m_pBody->IsStatic() )
	{
		JPH::BodyInterface& bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
		bodyInterface.ActivateBody( m_pBody->GetID() );
	}
	else
	{
		// See other comments in UpdateLayer.
		m_pEnvironment->AddDirtyStaticBody( m_pBody->GetID() );
	}
}

void JoltPhysicsObject::Sleep()
{
	if ( !m_pBody->IsStatic() )
	{
		JPH::BodyInterface& bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
		bodyInterface.DeactivateBody( m_pBody->GetID() );
	}
}

void JoltPhysicsObject::RecheckCollisionFilter()
{
	RecheckContactPoints();
}

void JoltPhysicsObject::RecheckContactPoints( bool bSearchForNewContacts /*= false*/ )
{
	JPH::BodyInterface& bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.InvalidateContactCache( GetBodyID() );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetMass( float mass )
{
	// To match regular VPhysics, allow 0 here but not at init.
	mass = Clamp( mass, 0.0f, VPHYSICS_MAX_MASS );

	m_flCachedMass = mass;
	m_flCachedInvMass = mass ? 1.0f / mass : 0.0f;

	if ( !IsStatic() )
	{
		JPH::MotionProperties* pMotionProperties = m_pBody->GetMotionProperties();
		// Mass is already in KG. IVP is weird.

		// Josh: This is what we used to do and it was giving VERY whacky results
		// when moving objects around after calling SetMass.
		// pMotionProperties->SetInverseMass( m_flCachedInvMass );
		// This method below seems to work properly because it deals with all of the inertia crap.
		JPH::MassProperties massProperties = m_pBody->GetShape()->GetMassProperties();
		massProperties.ScaleToMass( mass );
		massProperties.mInertia( 3, 3 ) = 1.0f;
		pMotionProperties->SetMassProperties( massProperties );

		CalculateBuoyancy();
	}
}

float JoltPhysicsObject::GetMass() const
{
	return m_flCachedMass;
}

float JoltPhysicsObject::GetInvMass() const
{
	return m_flCachedInvMass;
}

Vector JoltPhysicsObject::GetInertia() const
{
	Vector inv = GetInvInertia();
	return Vector( 1.0f / inv.x, 1.0f / inv.y, 1.0f / inv.z );
}

Vector JoltPhysicsObject::GetInvInertia() const
{
	if ( IsStatic() )
		return Vector( 1.0f, 1.0f, 1.0f );

	//const JPH::Vec3 inertia = m_pBody->GetMotionProperties()->GetInverseInertiaDiagonal();
	const JPH::Vec3 inertia = m_pBody->GetInverseInertia() * JPH::Vec3::sReplicate( 1.0f );
	return Abs( JoltToSource::Unitless( inertia ) );
}

void JoltPhysicsObject::SetInertia( const Vector &inertia )
{
	// TODO(Josh): Does anything use this?
	// and if so, does this specify the local diagonal or rotated diagonal?
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetDamping( const float *speed, const float *rot )
{
	if ( IsStatic() )
		return;

	JPH::MotionProperties* pMotionProperties = m_pBody->GetMotionProperties();
	if ( speed )
		pMotionProperties->SetLinearDamping( *speed );
	if ( rot )
		pMotionProperties->SetAngularDamping( *rot );
}

void JoltPhysicsObject::GetDamping( float *speed, float *rot ) const
{
	if ( IsStatic() )
		return;

	JPH::MotionProperties* pMotionProperties = m_pBody->GetMotionProperties();
	if ( speed )
		*speed = pMotionProperties->GetLinearDamping();
	if ( rot )
		*rot = pMotionProperties->GetAngularDamping();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetDragCoefficient( float *pDrag, float *pAngularDrag )
{
	Log_Stub( LOG_VJolt );
}

void JoltPhysicsObject::SetBuoyancyRatio( float ratio )
{
	m_flBuoyancyRatio = ratio;
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsObject::GetMaterialIndex() const
{
	return m_materialIndex;
}

void JoltPhysicsObject::SetMaterialIndex( int materialIndex )
{
	// Gotta clamp it, because we get -1 sometimes
	materialIndex = Max( 0, materialIndex );

	if ( m_materialIndex != materialIndex )
	{
		m_materialIndex = materialIndex;
		UpdateMaterialProperties();
	}
}

//-------------------------------------------------------------------------------------------------

unsigned int JoltPhysicsObject::GetContents() const
{
	return m_contents;
}

void JoltPhysicsObject::SetContents( unsigned int contents )
{
	m_contents = contents;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsObject::GetSphereRadius() const
{
	if ( m_pBody->GetShape()->GetSubType() != JPH::EShapeSubType::Sphere )
		return 0.0f;

	const JPH::SphereShape *pSphereShape = static_cast< const JPH::SphereShape * >( m_pBody->GetShape() );
	return JoltToSource::Distance( pSphereShape->GetRadius() );
}

void JoltPhysicsObject::SetSphereRadius( float radius )
{
	if ( m_pBody->GetShape()->GetSubType() != JPH::EShapeSubType::Sphere )
		return;

	// Can't get this shape non-const... urg...
	Log_Stub( LOG_VJolt );
}

float JoltPhysicsObject::GetEnergy() const
{
	// 1/2 * mv^2
	const float flKineticEnergy = 0.5f * m_flCachedMass * m_pBody->GetLinearVelocity().LengthSq();
	// TODO(Josh): We need to factor in inertia or something here to get this right.
	// as this AngularVelocity is in rads/s...
	// I guess it's a good enough approximation for now.
	// Right now the equation is 1/2ww where we probably want 1/2wIw.
	// Intertia in Jolt is weird... Not sure what's going on with its matrix thingy.
	const float flAngularEnergy = 0.5f * m_flCachedMass * m_pBody->GetAngularVelocity().LengthSq();

	return JoltToSource::Energy( flKineticEnergy + flAngularEnergy );
}

Vector JoltPhysicsObject::GetMassCenterLocalSpace() const
{
	return JoltToSource::Distance( m_pBody->GetShape()->GetCenterOfMass() );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetPosition( const Vector &worldPosition, const QAngle &angles, bool isTeleport )
{
	JPH::Vec3 joltPosition = SourceToJolt::Distance( worldPosition );
	JPH::Quat joltRotation = SourceToJolt::Angle( angles );

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();

	bodyInterface.SetPositionAndRotation( m_pBody->GetID(), joltPosition, joltRotation, JPH::EActivation::DontActivate );
}

void JoltPhysicsObject::SetPositionMatrix( const matrix3x4_t &matrix, bool isTeleport )
{
	SetPosition( GetColumn( matrix, MatrixAxis::Origin ), ToQAngle( matrix ), isTeleport );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::GetPosition( Vector *worldPosition, QAngle *angles ) const
{
	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();

	JPH::Vec3 joltPosition;
	JPH::Quat joltRotation;
	bodyInterface.GetPositionAndRotation( m_pBody->GetID(), joltPosition, joltRotation );

	if ( worldPosition )
		*worldPosition = JoltToSource::Distance( joltPosition );

	if ( angles )
		*angles = JoltToSource::Angle( joltRotation );
}

void JoltPhysicsObject::GetPositionMatrix( matrix3x4_t *positionMatrix ) const
{
	matrix3x4_t matrix;
	SetIdentityMatrix( matrix );
	AngleMatrix( JoltToSource::Angle( m_pBody->GetRotation() ), JoltToSource::Distance( m_pBody->GetPosition() ), matrix );
	*positionMatrix = matrix;
}

void JoltPhysicsObject::SetVelocity( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	JPH::Vec3 joltLinearVelocity = velocity ? SourceToJolt::Distance( *velocity ) : JPH::Vec3{};
	JPH::Vec3 joltAngularVelocity = angularVelocity ? SourceToJolt::AngularImpulse( *angularVelocity ) : JPH::Vec3{};

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();

	if ( velocity && angularVelocity )
		bodyInterface.SetLinearAndAngularVelocity( m_pBody->GetID(), joltLinearVelocity, joltAngularVelocity );
	else if ( velocity )
		bodyInterface.SetLinearVelocity( m_pBody->GetID(), joltLinearVelocity );
	else if ( angularVelocity )
		bodyInterface.SetAngularVelocity( m_pBody->GetID(), joltAngularVelocity );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetVelocityInstantaneous( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	SetVelocity( velocity, angularVelocity );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::GetVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const
{
	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();

	JPH::Vec3 joltLinearVelocity;
	JPH::Vec3 joltAngularVelocity;
	bodyInterface.GetLinearAndAngularVelocity( m_pBody->GetID(), joltLinearVelocity, joltAngularVelocity );

	if ( velocity )
		*velocity = JoltToSource::Distance( joltLinearVelocity );

	if ( angularVelocity )
		*angularVelocity = JoltToSource::AngularImpulse( joltAngularVelocity );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::AddVelocity( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	if ( !IsMoveable() )
		return;

	// Do this longer method do set velocity and angular velocity
	// in the same lock.
	const JPH::BodyLockInterfaceNoLock &bodyLockInterface = m_pPhysicsSystem->GetBodyLockInterfaceNoLock();

	JPH::BodyLockWrite lock( bodyLockInterface, m_pBody->GetID() );
	if ( lock.Succeeded() )
	{
		JPH::Body &body = lock.GetBody();

		if ( velocity )
			body.SetLinearVelocityClamped( body.GetLinearVelocity() + SourceToJolt::Distance( *velocity ) );

		if ( angularVelocity )
			body.SetAngularVelocityClamped( body.GetAngularVelocity() + SourceToJolt::AngularImpulse( *angularVelocity ) );

		if ( !body.IsActive() && ( !body.GetLinearVelocity().IsNearZero() || !body.GetAngularVelocity().IsNearZero() ) )
			m_pPhysicsSystem->GetBodyInterfaceNoLock().ActivateBodies( &m_pBody->GetID(), 1 );
	}
}

void JoltPhysicsObject::GetVelocityAtPoint( const Vector &worldPosition, Vector *pVelocity ) const
{
	VJoltAssert( pVelocity );

	*pVelocity = JoltToSource::Distance( m_pPhysicsSystem->GetBodyInterfaceNoLock().GetPointVelocity( m_pBody->GetID(), SourceToJolt::Distance( worldPosition ) ) );
}

void JoltPhysicsObject::GetImplicitVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const
{
	Log_Stub( LOG_VJolt );
	if ( velocity )
		*velocity = vec3_origin;

	if ( angularVelocity )
		*angularVelocity = vec3_origin;
}

void JoltPhysicsObject::LocalToWorld( Vector *worldPosition, const Vector &localPosition ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	// Copy in case src == dest
	VectorTransform( Vector( localPosition ), matrix, *worldPosition );
}

void JoltPhysicsObject::WorldToLocal( Vector *localPosition, const Vector &worldPosition ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	// Copy in case src == dest
	VectorITransform( Vector( worldPosition ), matrix, *localPosition );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::LocalToWorldVector( Vector *worldVector, const Vector &localVector ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	// Copy in case src == dest
	VectorRotate( Vector( localVector ), matrix, *worldVector );
}

void JoltPhysicsObject::WorldToLocalVector( Vector *localVector, const Vector &worldVector ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	// Copy in case src == dest
	VectorIRotate( Vector( worldVector ), matrix, *localVector );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::ApplyForceCenter( const Vector &forceVector )
{
	if ( !IsMoveable() )
		return;

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.AddImpulse( m_pBody->GetID(), SourceToJolt::Distance( forceVector ) );
}

void JoltPhysicsObject::ApplyForceOffset( const Vector &forceVector, const Vector &worldPosition )
{
	if ( !IsMoveable() )
		return;

	JPH::Vec3 impulse = SourceToJolt::Distance( forceVector );
	JPH::Vec3 point = SourceToJolt::Distance( worldPosition );

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.AddImpulse( m_pBody->GetID(), impulse, point );
}

void JoltPhysicsObject::ApplyTorqueCenter( const AngularImpulse &torque )
{
	if ( !IsMoveable() )
		return;

	// Do this longer method do set velocity and angular velocity
	// in the same lock.
	const JPH::BodyLockInterfaceNoLock &bodyLockInterface = m_pPhysicsSystem->GetBodyLockInterfaceNoLock();

	JPH::BodyLockWrite lock( bodyLockInterface, m_pBody->GetID() );
	if ( lock.Succeeded() )
	{
		JPH::Body &body = lock.GetBody();

		body.AddAngularImpulse( SourceToJolt::AngularImpulse( torque ) );

		if ( !body.IsActive() && ( !body.GetAngularVelocity().IsNearZero() ) )
			m_pPhysicsSystem->GetBodyInterfaceNoLock().ActivateBodies( &m_pBody->GetID(), 1 );
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::CalculateForceOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerForce, AngularImpulse *centerTorque ) const
{
	JPH::Vec3 pos = SourceToJolt::Distance( worldPosition );
	JPH::Vec3 force = SourceToJolt::Distance( forceVector );

	JPH::Vec3 com = pos - m_pBody->GetCenterOfMassPosition();
	JPH::Vec3 cross = com.Cross( force );

	if ( centerForce )
		*centerForce = JoltToSource::Distance( force );

	if ( centerTorque )
		*centerTorque = JoltToSource::AngularImpulse( cross );
}

void JoltPhysicsObject::CalculateVelocityOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerVelocity, AngularImpulse *centerAngularVelocity ) const
{
	// Convert force to SI units to multiply by mass for impulse.
	JPH::Vec3 siForce = SourceToJolt::Distance( forceVector );

	if ( centerVelocity )
		*centerVelocity = JoltToSource::Distance( siForce * m_flCachedInvMass );

	if ( centerAngularVelocity )
	{
		JPH::Vec3 siPosition = SourceToJolt::Distance( worldPosition );

		// TODO(Josh): Check this math.
		JPH::Vec3 siRelativePosition = siPosition - m_pBody->GetCenterOfMassPosition();
		JPH::Vec3 cross = siRelativePosition.Cross( siForce );
		cross = m_pBody->GetWorldTransform().Transposed3x3() * cross;

		*centerAngularVelocity = JoltToSource::AngularImpulse( cross );
	}
}

float JoltPhysicsObject::CalculateLinearDrag( const Vector &unitDirection ) const
{
	Log_Stub( LOG_VJolt );
	return 0.0f;
}

float JoltPhysicsObject::CalculateAngularDrag( const Vector &objectSpaceRotationAxis ) const
{
	Log_Stub( LOG_VJolt );
	return 0.0f;
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsObject::GetContactPoint( Vector *contactPoint, IPhysicsObject **contactObject ) const
{
	Log_Stub( LOG_VJolt );
	return false;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetShadow( float maxSpeed, float maxAngularSpeed, bool allowPhysicsMovement, bool allowPhysicsRotation )
{
	if ( m_pShadowController )
	{
		m_pShadowController->MaxSpeed( maxSpeed, maxAngularSpeed );
	}
	else
	{
#if 1
		m_bShadowTemporarilyDisableGravity = false;
#endif

		m_pShadowController = static_cast<JoltPhysicsShadowController *>( m_pEnvironment->CreateShadowController( this, allowPhysicsMovement, allowPhysicsRotation ) );
		m_pShadowController->MaxSpeed( maxSpeed, maxAngularSpeed );
	}
}

void JoltPhysicsObject::UpdateShadow( const Vector &targetPosition, const QAngle &targetAngles, bool tempDisableGravity, float timeOffset )
{
	if ( m_pShadowController )
	{
#if 1
		if ( tempDisableGravity != m_bShadowTemporarilyDisableGravity )
		{
			m_bShadowTemporarilyDisableGravity = tempDisableGravity;
			if ( !m_pShadowController || m_pShadowController->AllowsTranslation() )
				EnableGravity( !m_bShadowTemporarilyDisableGravity );
		}
#endif
		m_pShadowController->Update( targetPosition, targetAngles, timeOffset );
	}
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsObject::GetShadowPosition( Vector *position, QAngle *angles ) const
{
	// Josh:
	// If func_door_rotating, func_tracktrains are moving slowly,
	// check this function out...
	//
	// Interpolates to the next timestep
	float flTimeStep = m_pEnvironment->GetSimulationTimestep();

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();

	JPH::Vec3 joltPosition, joltLinearVelocity, joltAngularVelocity;
	JPH::Quat joltRotation;

	bodyInterface.GetPositionAndRotation( m_pBody->GetID(), joltPosition, joltRotation );
	bodyInterface.GetLinearAndAngularVelocity( m_pBody->GetID(), joltLinearVelocity, joltAngularVelocity );

	if ( position )
	{
		*position = JoltToSource::Distance( joltPosition + ( joltLinearVelocity * flTimeStep ) );
	}
	if ( angles )
	{
		// From Jolt's AddRotationStep.

		JPH::Vec3 joltAngularVelocityTimesDT = joltAngularVelocity * flTimeStep;
		float len = joltAngularVelocityTimesDT.Length();

		JPH::Quat newQuat = joltRotation;
		if ( len > 1.0e-6f ) 
			newQuat = ( JPH::Quat::sRotation( joltAngularVelocityTimesDT / len, len ) * joltRotation ).Normalized();
		*angles = JoltToSource::Angle( newQuat );
	}

	return 1;
}

IPhysicsShadowController *JoltPhysicsObject::GetShadowController() const
{
	return m_pShadowController;
}

void JoltPhysicsObject::RemoveShadowController()
{
	if ( m_pShadowController )
	{
		m_pEnvironment->DestroyShadowController(m_pShadowController);
		m_pShadowController = nullptr;
	}
}

static void ComputeController( JPH::Vec3 &vecCurrentVelocity, const JPH::Vec3 &vecDeltaPos, float flMaxSpeed, float flMaxDampSpeed, float flScaleDelta, float flDamping, JPH::Vec3 *pOutImpulse = nullptr )
{
	float flCurrentSpeedSq = vecCurrentVelocity.LengthSq();
	if ( flCurrentSpeedSq < 1e-6f )
	{
		vecCurrentVelocity = JPH::Vec3::sZero();
	}
	else if ( flMaxDampSpeed > 0 )
	{
		JPH::Vec3 vecAccelDampening = vecCurrentVelocity * -flDamping;
		float flSpeed = sqrtf( flCurrentSpeedSq ) * fabsf( flDamping );
		if ( flSpeed > flMaxDampSpeed )
		{
			flSpeed = flMaxDampSpeed / flSpeed;
			vecAccelDampening *= flSpeed;
		}
		vecCurrentVelocity += vecAccelDampening;
	}

	JPH::Vec3 vecAcceleration = JPH::Vec3::sZero();
	if ( flMaxSpeed > 0.0f )
	{
		vecAcceleration = vecDeltaPos * flScaleDelta;
		float flSpeed = vecDeltaPos.Length() * flScaleDelta;
		if ( flSpeed > flMaxSpeed )
		{
			flSpeed = flMaxSpeed / flSpeed;
			vecAcceleration *= flSpeed;
		}
		vecCurrentVelocity += vecAcceleration;
	}

	if ( pOutImpulse )
		*pOutImpulse = vecAcceleration;
}

// hlshadowcontrol_params_t but in Jolt space.
struct JoltShadowControlParams
{
	JPH::Vec3			TargetPosition;
	JPH::Quat			TargetRotation;
	JPH::Vec3			LastPosition;
	JPH::Vec3			LastImpulse;
	float				MaxAngular;
	float				MaxDampAngular;
	float				MaxSpeed;
	float				MaxDampSpeed;
	float				DampFactor;
	float				TeleportDistance;
};

static float ComputeShadowController( JoltShadowControlParams &params, JPH::Vec3 &position, JPH::Quat &rotation, JPH::Vec3 &linearVelocity, JPH::Vec3& angularVelocity, float flSecondsToArrival, float flDeltaTime )
{
	const float flFraction = flSecondsToArrival > 0.0f
		? Min( flDeltaTime / flSecondsToArrival, 1.0f )
		: 1.0f;

	flSecondsToArrival = Max( flSecondsToArrival - flDeltaTime, 0.0f );

	if ( flFraction <= 0.0f )
		return flSecondsToArrival;

	JPH::Vec3 deltaPosition = params.TargetPosition - position;

	if ( params.TeleportDistance > 0.0f && deltaPosition.LengthSq() > Square( params.TeleportDistance ) )
	{
		position = params.TargetPosition;
		rotation = params.TargetRotation;
		deltaPosition = JPH::Vec3::sZero();
	}

	const float flInvDeltaTime = 1.0f / flDeltaTime;
	const float flFractionTime = flFraction * flInvDeltaTime;

	ComputeController( linearVelocity, deltaPosition, params.MaxSpeed, params.MaxDampSpeed, flFractionTime, params.DampFactor, &params.LastImpulse);

	params.LastPosition = position + linearVelocity * flDeltaTime;

	JPH::Quat deltaRotation = params.TargetRotation * rotation.Inversed();
	
	JPH::Vec3 axis;
	float angle;
	deltaRotation.GetAxisAngle( axis, angle );
	
	JPH::Vec3 deltaAngles = axis * angle;
	ComputeController( angularVelocity, deltaAngles, params.MaxAngular, params.MaxDampAngular, flFractionTime, params.DampFactor );

	return flSecondsToArrival;
}


float JoltPhysicsObject::ComputeShadowControl( const hlshadowcontrol_params_t &params, float flSecondsToArrival, float flDeltaTime )
{
	JoltShadowControlParams joltParams =
	{
		.TargetPosition		= SourceToJolt::Distance( params.targetPosition ),
		.TargetRotation		= SourceToJolt::Angle( params.targetRotation ),
		.MaxAngular			= SourceToJolt::Angle( params.maxAngular ),
		.MaxDampAngular		= SourceToJolt::Angle( params.maxDampAngular ),
		.MaxSpeed			= SourceToJolt::Distance( params.maxSpeed ),
		.MaxDampSpeed		= SourceToJolt::Distance( params.maxDampSpeed ),
		.DampFactor			= params.dampFactor,
		.TeleportDistance	= SourceToJolt::Distance( params.teleportDistance ),
	};

	JPH::BodyInterface& bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();

	JPH::Vec3 position;
	JPH::Quat rotation;
	bodyInterface.GetPositionAndRotation( m_pBody->GetID(), position, rotation );
	JPH::Vec3 linearVelocity;
	JPH::Vec3 angularVelocity;
	bodyInterface.GetLinearAndAngularVelocity( m_pBody->GetID(), linearVelocity, angularVelocity );

	JPH::Vec3 scratchPosition = position;
	JPH::Quat scratchRotation = rotation;
	JPH::Vec3 scratchLinearVelocity = linearVelocity;
	JPH::Vec3 scratchAngularVelocity = angularVelocity;
	float flNewSecondsToArrival =
		ComputeShadowController( joltParams, scratchPosition, scratchRotation, scratchLinearVelocity, scratchAngularVelocity, flSecondsToArrival, flDeltaTime );
	
	if ( scratchPosition != position || scratchRotation != rotation )
		bodyInterface.SetPositionAndRotation( m_pBody->GetID(), scratchPosition, scratchRotation, JPH::EActivation::Activate );

	if ( scratchLinearVelocity != linearVelocity || scratchAngularVelocity != angularVelocity )
		bodyInterface.SetLinearAndAngularVelocity( m_pBody->GetID(), scratchLinearVelocity, scratchAngularVelocity );

	return flNewSecondsToArrival;
}

//-------------------------------------------------------------------------------------------------

const CPhysCollide *JoltPhysicsObject::GetCollide() const
{
	const CPhysCollide *pCollide = CPhysCollide::FromShape( m_pBody->GetShape() );
	return pCollide;
}

const char *JoltPhysicsObject::GetName() const
{
	// Slart: Jolt used to store debug names in JPH::Body, but it was removed. So now everybody's NoName.
	return "NoName";
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::BecomeTrigger()
{
	m_pBody->SetIsSensor( true );
}

void JoltPhysicsObject::RemoveTrigger()
{
	if ( !IsTrigger() )
		return;

	// Josh:
	// All this logic below is to trigger ObjectLeaveTrigger
	// when the trigger is deleted.
	IPhysicsCollisionEvent *pEventListener = m_pEnvironment->GetContactListener()->GetGameListener();
	if ( pEventListener && IsTrigger() )
	{
		class SourceTriggerCollector : public JPH::CollideShapeCollector
		{
		public:
			SourceTriggerCollector( JPH::PhysicsSystem *pPhysicsSystem, IPhysicsCollisionEvent *pGameListener, JoltPhysicsObject *pTrigger )
				: m_pPhysicsSystem( pPhysicsSystem )
				, m_pGameListener ( pGameListener )
				, m_pTrigger      ( pTrigger ) {}

			void AddHit( const ResultType &inResult ) override
			{
				const JPH::BodyID inBodyID = inResult.mBodyID2;

				JPH::BodyLockWrite lock( m_pPhysicsSystem->GetBodyLockInterface(), inBodyID );
				JPH::Body &body = lock.GetBody();
				JoltPhysicsObject *pObject = reinterpret_cast<JoltPhysicsObject *>( body.GetUserData() );

				if ( !pObject )
					return;

				m_pGameListener->ObjectLeaveTrigger( m_pTrigger, pObject );
			}

		private:
			JPH::PhysicsSystem     *m_pPhysicsSystem;
			IPhysicsCollisionEvent *m_pGameListener;
			JoltPhysicsObject      *m_pTrigger;
		};

		SourceTriggerCollector collector( m_pPhysicsSystem, pEventListener, this );

		JPH::IgnoreSingleBodyFilter body_filter( GetBodyID() );

		JPH::CollideShapeSettings collideSettings;
		collideSettings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideWithAll;

		JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
		JPH::Mat44 queryTransform = bodyInterface.GetCenterOfMassTransform( GetBodyID() );

		const JPH::Shape *pShape = GetCollide()->ToShape();

		m_pPhysicsSystem->GetNarrowPhaseQueryNoLock().CollideShape(
			pShape, JPH::Vec3::sReplicate( 1.0f ), queryTransform, collideSettings, JPH::Vec3::sZero(), collector,
			JPH::SpecifiedBroadPhaseLayerFilter( BroadPhaseLayers::MOVING ), JPH::SpecifiedObjectLayerFilter( Layers::MOVING ), body_filter );
	}

	m_pBody->SetIsSensor( false );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::BecomeHinged( int localAxis )
{
	Log_Stub( LOG_VJolt );
}

void JoltPhysicsObject::RemoveHinged()
{
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

IPhysicsFrictionSnapshot *JoltPhysicsObject::CreateFrictionSnapshot()
{
	return new JoltPhysicsFrictionSnapshot;
}

void JoltPhysicsObject::DestroyFrictionSnapshot( IPhysicsFrictionSnapshot *pSnapshot )
{
	delete pSnapshot;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::OutputDebugInfo() const
{
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetUseAlternateGravity( bool bSet )
{
	Log_Stub( LOG_VJolt );
}

void JoltPhysicsObject::SetCollisionHints( uint32 collisionHints )
{
	m_collisionHints = collisionHints;
	UpdateLayer();
}

uint32 JoltPhysicsObject::GetCollisionHints() const
{
	return m_collisionHints;
}

//-------------------------------------------------------------------------------------------------

IPredictedPhysicsObject *JoltPhysicsObject::GetPredictedInterface() const
{
	Log_Stub( LOG_VJolt );
	return nullptr;
}

void JoltPhysicsObject::SyncWith( IPhysicsObject *pOther )
{
	if ( this->IsCollisionEnabled() != pOther->IsCollisionEnabled() )
		EnableCollisions( pOther->IsCollisionEnabled() );

	if ( this->IsGravityEnabled() != pOther->IsGravityEnabled() )
		EnableGravity( pOther->IsGravityEnabled() );

	if ( this->IsDragEnabled() != pOther->IsDragEnabled() )
		EnableDrag(pOther->IsDragEnabled() );

	if ( this->IsMotionEnabled() != pOther->IsMotionEnabled() )
		EnableMotion(pOther->IsMotionEnabled() );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::UpdateEnvironment( JoltPhysicsEnvironment *pEnvironment )
{
	m_pEnvironment = pEnvironment;
	m_pPhysicsSystem = pEnvironment->GetPhysicsSystem();
}

void JoltPhysicsObject::AddDestroyedListener( IJoltObjectDestroyedListener *pListener )
{
	m_destroyedListeners.AddToTail( pListener );
}

void JoltPhysicsObject::RemoveDestroyedListener( IJoltObjectDestroyedListener *pListener )
{
	m_destroyedListeners.FindAndRemove( pListener );
}

void JoltPhysicsObject::AddToPosition( JPH::Vec3Arg addPos )
{
	const JPH::BodyLockInterfaceNoLock &bodyLockInterface = m_pPhysicsSystem->GetBodyLockInterfaceNoLock();

	JPH::BodyLockWrite lock( bodyLockInterface, m_pBody->GetID() );
	if ( lock.Succeeded() )
	{
		JPH::Body &body = lock.GetBody();

		JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
		bodyInterface.SetPosition( m_pBody->GetID(), body.GetPosition() + addPos, JPH::EActivation::DontActivate );
	}
}

void JoltPhysicsObject::SetPosition( const Vector &worldPosition )
{
	JPH::Vec3 joltPosition = SourceToJolt::Distance( worldPosition );

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();

	bodyInterface.SetPosition( m_pBody->GetID(), joltPosition, JPH::EActivation::DontActivate );
}

void JoltPhysicsObject::AddVelocity( const Vector &worldPosition )
{
	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.AddLinearVelocity( m_pBody->GetID(), SourceToJolt::Distance( worldPosition ) );
}

Vector JoltPhysicsObject::GetVelocity()
{
	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	return JoltToSource::Distance( bodyInterface.GetLinearVelocity( m_pBody->GetID() ) );
}

void JoltPhysicsObject::CalculateBuoyancy()
{
	if ( m_flVolume != 0.0f )
	{
		float flVolume = SourceToJolt::Volume( Max( m_flVolume, 5.0f ) );
		float flDensity = m_flCachedMass / flVolume;
		m_flBuoyancyRatio = flDensity / m_flMaterialDensity;
	}
	else
	{
		m_flBuoyancyRatio = 1.0f;
	}
}

float JoltPhysicsObject::GetMaterialDensity() const
{
	return m_flMaterialDensity;
}

float JoltPhysicsObject::GetBuoyancyRatio() const
{
	return m_flBuoyancyRatio; 
}

bool JoltPhysicsObject::IsControlledByGame() const
{
	if ( m_pShadowController && !m_pShadowController->IsPhysicallyControlled() )
		return true;

	if ( m_callbackFlags & CALLBACK_IS_PLAYER_CONTROLLER )
		return true;

	return false;
}

void JoltPhysicsObject::SaveObjectState( JPH::StateRecorder &recorder )
{
	m_pBody->SaveState( recorder );

	// Josh: Do not write m_pGameData, as this is passed in, in UnserializeObjectFromBuffer.
	//recorder.Write( m_pGameData );
	recorder.Write( m_gameFlags );
	recorder.Write( m_gameIndex );
	recorder.Write( m_callbackFlags );
	recorder.Write( m_bStatic );
	recorder.Write( m_bPinned );
	recorder.Write( m_materialIndex );
	recorder.Write( m_contents );
	recorder.Write( m_flCachedMass );
	recorder.Write( m_flCachedInvMass );
	recorder.Write( m_bCachedCollisionEnabled );
	recorder.Write( m_flMaterialDensity );
	recorder.Write( m_flBuoyancyRatio );
	recorder.Write( m_flVolume );
	recorder.Write( m_GameMaterial );

	// Josh:
	// In regular VPhysics, shadows are serialized but then forced to never be read.
	// Lets just not bother serializing these.
}

void JoltPhysicsObject::RestoreObjectState( JPH::StateRecorder &recorder )
{
	// Restore the body's state.
	m_pBody->RestoreState( recorder );

	// Set the body's userdata as ourselves
	m_pBody->SetUserData( reinterpret_cast<uint64>( this ) );

	// Josh: Do not read m_pGameData, as this is not serialized.
	//recorder.Read( m_pGameData );
	recorder.Read( m_gameFlags );
	recorder.Read( m_gameIndex );
	recorder.Read( m_callbackFlags );
	recorder.Read( m_bStatic );
	recorder.Read( m_bPinned );
	recorder.Read( m_materialIndex );
	recorder.Read( m_contents );
	recorder.Read( m_flCachedMass );
	recorder.Read( m_flCachedInvMass );
	recorder.Read( m_bCachedCollisionEnabled );
	recorder.Read( m_flMaterialDensity );
	recorder.Read( m_flBuoyancyRatio );
	recorder.Read( m_flVolume );
	recorder.Read( m_GameMaterial );

	// Recompute states.
	UpdateMaterialProperties();
	UpdateLayer();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::UpdateMaterialProperties()
{
	const surfacedata_t *pSurface = JoltPhysicsSurfaceProps::GetInstance().GetSurfaceData( m_materialIndex );

	m_pBody->SetRestitution( pSurface->physics.elasticity );
	m_pBody->SetFriction( pSurface->physics.friction );
	m_flMaterialDensity = pSurface->physics.density;
	m_GameMaterial = pSurface->game.material;
	CalculateBuoyancy();
}

void JoltPhysicsObject::UpdateLayer()
{
	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();

	const bool bCollisionsEnabled = m_bCachedCollisionEnabled;
	const bool bStatic = IsStatic();
	const bool bPinned = m_bPinned;
	const bool bDebris = m_collisionHints & COLLISION_HINT_DEBRIS;
	const bool bStaticSolid = m_collisionHints & COLLISION_HINT_STATICSOLID;

	// Update motion type if not made as a complete solid.
	if ( !bStatic && !IsControlledByGame() )
	{
		bool bStaticMotionType = bStaticSolid || bPinned;

		// If we are transfering to being static, and we were active
		// add us to a list of bodies on the environment so we can be included in
		// GetActiveObjects for the next step.
		// This way the game can correctly update the transforms on the game side
		// when move -> wake -> become pinned happens.
		if ( bStaticMotionType && m_pBody->IsActive() )
			m_pEnvironment->AddDirtyStaticBody( m_pBody->GetID() );
		else if ( !bStaticMotionType )
			m_pEnvironment->RemoveDirtyStaticBody( m_pBody->GetID() );

		bodyInterface.SetMotionType( m_pBody->GetID(), bStaticMotionType ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic, JPH::EActivation::Activate );
	}

	// Update layer
	uint8 layer = Layers::MOVING;

	if ( bDebris )
		layer = Layers::DEBRIS;

	if ( bStatic || bStaticSolid )
		layer = Layers::NON_MOVING_WORLD;
	else if ( bPinned )
		layer = Layers::NON_MOVING_OBJECT;

	if ( !bCollisionsEnabled )
		layer = Layers::NO_COLLIDE;

	bodyInterface.SetObjectLayer( m_pBody->GetID(), layer );
}
