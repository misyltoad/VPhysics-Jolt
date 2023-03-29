//=================================================================================================
//
// Constraints
//
//=================================================================================================

#include "cbase.h"

#include "vjolt_environment.h"
#include "vjolt_layers.h"
#include "vjolt_object.h"

#include "vjolt_constraints.h"

#include "vjolt_layers.h"

//-------------------------------------------------------------------------------------------------

static ConVar vjolt_ragdoll_hinge_optimization( "vjolt_ragdoll_hinge_optimization", "1", FCVAR_REPLICATED,
	"Optimizes ragdolls to use hinge constraints for joints with 1 degree of freedom. Additionally fixes legs going back on themselves. Currently breaks ragdolls of NPCs killed in a pose (they inherit the pose).");

//-------------------------------------------------------------------------------------------------

JoltPhysicsConstraintGroup::JoltPhysicsConstraintGroup()
{
}

JoltPhysicsConstraintGroup::~JoltPhysicsConstraintGroup()
{
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraintGroup::Activate()
{
	for ( JoltPhysicsConstraint *pConstraint : m_pConstraints )
		pConstraint->Activate();
}

bool JoltPhysicsConstraintGroup::IsInErrorState()
{
	return false;
}

void JoltPhysicsConstraintGroup::ClearErrorState()
{
}

void JoltPhysicsConstraintGroup::GetErrorParams( constraint_groupparams_t *pParams )
{
	if ( pParams )
		*pParams = m_ErrorParams;
}

void JoltPhysicsConstraintGroup::SetErrorParams( const constraint_groupparams_t &params )
{
	m_ErrorParams = params;
}

void JoltPhysicsConstraintGroup::SolvePenetration( IPhysicsObject *pObj0, IPhysicsObject *pObj1 )
{
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraintGroup::AddConstraint( JoltPhysicsConstraint *pConstraint )
{
	m_pConstraints.push_back( pConstraint );
}

void JoltPhysicsConstraintGroup::RemoveConstraint( JoltPhysicsConstraint *pConstraint )
{
	Erase( m_pConstraints, pConstraint );
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsConstraint::JoltPhysicsConstraint( JoltPhysicsEnvironment *pPhysicsEnvironment, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, constraintType_t Type, JPH::Constraint* pConstraint, void *pGameData )
	: m_pPhysicsEnvironment( pPhysicsEnvironment )
	, m_pPhysicsSystem( pPhysicsEnvironment->GetPhysicsSystem() )
	, m_pObjReference( static_cast<JoltPhysicsObject*>( pReferenceObject ) )
	, m_pObjAttached( static_cast<JoltPhysicsObject*>( pAttachedObject ) )
	, m_ConstraintType( Type )
	, m_pConstraint( pConstraint )
	, m_pGameData( pGameData )
{
	m_pObjReference->AddDestroyedListener( this );
	m_pObjAttached->AddDestroyedListener( this );
}

JoltPhysicsConstraint::~JoltPhysicsConstraint()
{
	if ( m_pGroup )
	{
		m_pGroup->RemoveConstraint( this );
		m_pGroup = nullptr;
	}

	DestroyConstraint();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::Activate()
{
	if ( m_pConstraint )
		m_pConstraint->SetEnabled( true );
}

void JoltPhysicsConstraint::Deactivate()
{
	if ( m_pConstraint )
		m_pConstraint->SetEnabled( false );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SetGameData( void *gameData )
{
	m_pGameData = gameData;
}

void *JoltPhysicsConstraint::GetGameData() const
{
	return m_pGameData;
}

//-------------------------------------------------------------------------------------------------

IPhysicsObject *JoltPhysicsConstraint::GetReferenceObject() const
{
	return m_pObjReference;
}

IPhysicsObject *JoltPhysicsConstraint::GetAttachedObject() const
{
	return m_pObjAttached;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SetLinearMotor( float speed, float maxLinearImpulse )
{
	if ( !m_pConstraint )
		return;

	speed = SourceToJolt::Distance( speed );
	maxLinearImpulse = SourceToJolt::Distance( maxLinearImpulse );

	switch ( m_ConstraintType )
	{
		case CONSTRAINT_SLIDING:
		{
			JPH::SliderConstraint *pConstraint = static_cast<JPH::SliderConstraint *>( m_pConstraint.GetPtr() );
			pConstraint->SetMotorState( speed ? JPH::EMotorState::Velocity : JPH::EMotorState::Off );
			pConstraint->SetTargetVelocity( speed );

			JPH::MotorSettings &motorSettings = pConstraint->GetMotorSettings();
			motorSettings.SetForceLimits( -maxLinearImpulse, maxLinearImpulse );

			break;
		}
	}
}

void JoltPhysicsConstraint::SetAngularMotor( float rotSpeed, float maxAngularImpulse )
{
	if ( !m_pConstraint )
		return;

	rotSpeed = DEG2RAD( rotSpeed );
	maxAngularImpulse = DEG2RAD( maxAngularImpulse );

	switch ( m_ConstraintType )
	{
		case CONSTRAINT_RAGDOLL:
		{
			// Josh:
			// If you change the hinge optimization stuff, remember to
			// check this! m_ConstraintType is CONSTRAINT_HINGE for that! (same with normal vphysics)
			//
			// Something else to note is... does the below code for friction vs angular impulse work on
			// ragdolls -> hinges correctly? This happens in Source, but this may not necessarily be correct.
			// :/
			VJoltAssert( m_pConstraint->GetSubType() == JPH::EConstraintSubType::SixDOF );

			JPH::SixDOFConstraint *pConstraint = static_cast<JPH::SixDOFConstraint *>( m_pConstraint.GetPtr() );
			pConstraint->SetTargetAngularVelocityCS( JPH::Vec3( rotSpeed, rotSpeed, rotSpeed ) );
			pConstraint->SetMaxFriction( JPH::SixDOFConstraint::EAxis::RotationX, maxAngularImpulse );
			pConstraint->SetMaxFriction( JPH::SixDOFConstraint::EAxis::RotationY, maxAngularImpulse );
			pConstraint->SetMaxFriction( JPH::SixDOFConstraint::EAxis::RotationZ, maxAngularImpulse );
			break;
		}

		case CONSTRAINT_HINGE:
		{
			JPH::HingeConstraint *pConstraint = static_cast<JPH::HingeConstraint *>( m_pConstraint.GetPtr() );
			pConstraint->SetMotorState( rotSpeed ? JPH::EMotorState::Velocity : JPH::EMotorState::Off );
			pConstraint->SetTargetAngularVelocity( rotSpeed );

			JPH::MotorSettings &motorSettings = pConstraint->GetMotorSettings();
			motorSettings.SetForceLimits( -fabsf( maxAngularImpulse ), fabsf( maxAngularImpulse ) );

			break;
		}
	}
}

//-------------------------------------------------------------------------------------------------

// Slart: This is never called anywhere in our codebase
void JoltPhysicsConstraint::UpdateRagdollTransforms( const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached )
{
}

// Slart: This is only used for visual debugging, which we don't *really* need since we have Jolt's debugger
bool JoltPhysicsConstraint::GetConstraintTransform( matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached ) const
{
	if ( m_pObjReference && pConstraintToReference )
		m_pObjReference->GetPositionMatrix( pConstraintToReference );
	if ( m_pObjAttached && pConstraintToAttached )
		m_pObjAttached->GetPositionMatrix( pConstraintToAttached );
	return true;
}

// Slart: Yet another debugging thing
bool JoltPhysicsConstraint::GetConstraintParams( constraint_breakableparams_t *pParams ) const
{
	return false;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::OutputDebugInfo()
{

}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	DestroyConstraint();

	// Normal VPhysics calls ConstraintBroken when an object being killed destroys the constraint.
	m_pPhysicsEnvironment->NotifyConstraintDisabled( this );
}

//-------------------------------------------------------------------------------------------------
// Ragdoll
//-------------------------------------------------------------------------------------------------

static Vector DOFToAxis( uint32 uDOF )
{
	return Vector(
		uDOF == 0 ? 1.0f : 0.0f,
		uDOF == 1 ? 1.0f : 0.0f,
		uDOF == 2 ? 1.0f : 0.0f );
}

static uint32 NextDOF( uint32 uDOF )
{
	return ( uDOF + 1 ) % 3;
}

static bool IsFixedAxis( const constraint_axislimit_t &axis )
{
	return axis.minRotation == axis.maxRotation;
}

// Returns a bitmask of degrees of freedom for a ragdoll.
static uint32 GetDegreesOfFreedom( const constraint_ragdollparams_t &ragdoll )
{
	uint32 uDOFMask = 0;

	for ( int i = 0; i < 3; i++ )
	{
		if ( !IsFixedAxis( ragdoll.axes[i] ) )
			uDOFMask |= 1u << i;
	}

	return uDOFMask;
}

bool JoltPhysicsConstraint::InitialiseHingeFromRagdoll( IPhysicsConstraintGroup* pGroup, const constraint_ragdollparams_t& ragdoll )
{
	const uint32 uDOFMask = GetDegreesOfFreedom( ragdoll );
	const uint32 uDOFCount = JPH::CountBits( uDOFMask );

	if ( uDOFCount != 1 )
		return false;

	const uint32 uDOF = JPH::CountTrailingZeros( uDOFMask );
	const Vector vecNextDOFAxis = DOFToAxis( NextDOF( uDOF ) );

	matrix3x4_t refObjToWorld;
	m_pObjReference->GetPositionMatrix( &refObjToWorld );

	matrix3x4_t constraintToWorld;
	ConcatTransforms( refObjToWorld, ragdoll.constraintToReference, constraintToWorld );

	Vector perpAxisDir;
	VectorIRotate( vecNextDOFAxis, ragdoll.constraintToReference, perpAxisDir );

	constraint_limitedhingeparams_t hinge;
	hinge.Defaults();
	hinge.constraint					= ragdoll.constraint;
	hinge.worldPosition					= GetColumn( constraintToWorld, MatrixAxis::Origin );
	hinge.worldAxisDirection			= GetColumn( constraintToWorld, static_cast< JoltMatrixAxes >( uDOF ) );
	hinge.referencePerpAxisDirection	= vecNextDOFAxis;
	hinge.attachedPerpAxisDirection		= Rotate( perpAxisDir, ragdoll.constraintToAttached );
	hinge.hingeAxis						= ragdoll.axes[ uDOF ];

	if ( !ragdoll.useClockwiseRotations )
	{
		const float minLimit = hinge.hingeAxis.minRotation;
		const float maxLimit = hinge.hingeAxis.maxRotation;

		hinge.hingeAxis.minRotation = -maxLimit;
		hinge.hingeAxis.maxRotation = -minLimit;
	}

	InitialiseHinge( pGroup, hinge );

	return true;
}

void JoltPhysicsConstraint::InitialiseRagdoll( IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll )
{
	// Josh:
	// Optimize to a hinge constraint if we can -- avoids a bunch of useless computation
	// and additionally fixes the fact that we can only specify disjoint min/max rotations
	// on the X axis with Jolt 6DOF
	// Currently breaks killing NPCs that are in a pose for some reason -- they stay in the pose. Needs investigation.
	if ( vjolt_ragdoll_hinge_optimization.GetBool() && InitialiseHingeFromRagdoll( pGroup, ragdoll ) )
		return;

	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_RAGDOLL;

	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::Mat44 constraintToReference = SourceToJolt::Matrix( ragdoll.constraintToReference );
	JPH::Mat44 constraintToAttached = SourceToJolt::Matrix( ragdoll.constraintToAttached );

	JPH::SixDOFConstraintSettings settings;
	settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;

	settings.mPosition1 = constraintToReference.GetTranslation() - refBody->GetShape()->GetCenterOfMass();
	settings.mAxisX1 = constraintToReference.GetAxisX();
	settings.mAxisY1 = constraintToReference.GetAxisY();

	settings.mPosition2 = constraintToAttached.GetTranslation() - attBody->GetShape()->GetCenterOfMass();
	settings.mAxisX2 = constraintToAttached.GetAxisX();
	settings.mAxisY2 = constraintToAttached.GetAxisY();


	for ( int i = 0; i < 3; i++ )
	{
		JPH::SixDOFConstraintSettings::EAxis positionalAxis = static_cast< JPH::SixDOFConstraintSettings::EAxis >(
			JPH::SixDOFConstraintSettings::EAxis::TranslationX + i );

		JPH::SixDOFConstraintSettings::EAxis rotationalAxis = static_cast< JPH::SixDOFConstraintSettings::EAxis >(
			JPH::SixDOFConstraintSettings::EAxis::RotationX + i );

		// Make positional axes fixed, unless otherwise stated; the airboat needs them unlocked for a funny hack.
		if ( !ragdoll.onlyAngularLimits )
			settings.MakeFixedAxis( positionalAxis );

		if ( ragdoll.axes[i].minRotation == ragdoll.axes[i].maxRotation )
		{
			//Log_Msg( LOG_VJolt, "Creating ragdoll-fixed constraint\n" );
			settings.MakeFixedAxis( rotationalAxis );
		}
		else
		{
			if ( i == 0 )
			{
				//Log_Msg( LOG_VJolt, "Creating X ragdoll constraint with %g min and %g max\n", -ragdoll.axes[i].maxRotation, -ragdoll.axes[i].minRotation );
				settings.SetLimitedAxis( rotationalAxis, DEG2RAD( ragdoll.axes[i].minRotation ), DEG2RAD( ragdoll.axes[i].maxRotation ) );
			}
			else
			{
				// Josh:
				// Jolt uses a Swing Twist for part of this, which means we have to find the max movement allowed
				// to contrain it by, I think. This results in legs that kind of flop both ways... From the Jolt code
				// "The swing twist constraint part requires symmetrical rotations around Y and Z"
				//
				// This is kind of 'worked around' by the code above that converts 1DOF -> hinges.

				const float maxMovement = Max( fabsf( ragdoll.axes[i].maxRotation ), fabsf( ragdoll.axes[i].minRotation ) );
				//Log_Msg( LOG_VJolt, "Creating Y/Z ragdoll constraint with %g min and %g max\n", -maxMovement, maxMovement );
				settings.SetLimitedAxis( rotationalAxis, -DEG2RAD( maxMovement ), DEG2RAD( maxMovement ) );
			}
		}

		// Swap the limits if we are using clockwise rotations,
		// this is only not true if we are saving/loading.
		if ( ragdoll.useClockwiseRotations )
		{
			const float minLimit = settings.mLimitMin[rotationalAxis];
			const float maxLimit = settings.mLimitMax[rotationalAxis];

			settings.mLimitMin[rotationalAxis] = -maxLimit;
			settings.mLimitMax[rotationalAxis] = -minLimit;
		}

		// TODO(Josh): What is .torque on a ragdoll in Source? I want to understand what it is
		// before setting random values on the Jolt side.
		//
		//if ( ragdoll.axes[ i ].torque != 0.0f )
		//	settings.mMotorSettings[ rotationalAxis ].SetTorqueLimit( ragdoll.axes[ i ].torque );
	}

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetEnabled( !pGroup && ragdoll.constraint.isActive );

	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------
// Hinge
//-------------------------------------------------------------------------------------------------

static JPH::Vec3 HingePerpendicularVector( JPH::Vec3Arg dir )
{
	return fabsf( dir.GetX() ) < 0.57f
		? JPH::Vec3::sAxisX().Cross( dir ).Normalized()
		: JPH::Vec3::sAxisY().Cross( dir ).Normalized();
}

void JoltPhysicsConstraint::InitialiseHinge( IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_HINGE;

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::HingeConstraintSettings settings;
	settings.mPoint1 = SourceToJolt::Distance( hinge.worldPosition );
	settings.mPoint2 = SourceToJolt::Distance( hinge.worldPosition );

	settings.mHingeAxis1 = JPH::Vec3( hinge.worldAxisDirection.x, hinge.worldAxisDirection.y, hinge.worldAxisDirection.z );
	settings.mHingeAxis2 = JPH::Vec3( hinge.worldAxisDirection.x, hinge.worldAxisDirection.y, hinge.worldAxisDirection.z );

	settings.mNormalAxis1 = HingePerpendicularVector( settings.mHingeAxis1 );
	settings.mNormalAxis2 = HingePerpendicularVector( settings.mHingeAxis2 );

	if ( hinge.hingeAxis.minRotation != hinge.hingeAxis.maxRotation )
	{
		settings.mLimitsMin = DEG2RAD( -hinge.hingeAxis.maxRotation );
		settings.mLimitsMax = DEG2RAD( -hinge.hingeAxis.minRotation );
	}

	// TODO(Josh): Fix this... I have no idea what this should be.
	//settings.mMaxFrictionTorque = hinge.hingeAxis.torque;

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetEnabled( !pGroup && hinge.constraint.isActive );

	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------
// Sliding
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialiseSliding( IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_SLIDING;

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::SliderConstraintSettings settings;
	settings.mAutoDetectPoint = true;
	settings.SetSliderAxis( JPH::Vec3( sliding.slideAxisRef.x, sliding.slideAxisRef.y, sliding.slideAxisRef.z ) );

	if ( sliding.limitMin != sliding.limitMax )
	{
		settings.mLimitsMin = SourceToJolt::Distance( sliding.limitMin );
		settings.mLimitsMax = SourceToJolt::Distance( sliding.limitMax );
	}

	settings.mMaxFrictionForce = sliding.friction;

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetEnabled( !pGroup && sliding.constraint.isActive );

	if ( sliding.velocity )
	{
		JPH::SliderConstraint *pConstraint = static_cast<JPH::SliderConstraint *>( m_pConstraint.GetPtr() );
		pConstraint->SetMotorState( JPH::EMotorState::Velocity );
		pConstraint->SetTargetVelocity( SourceToJolt::Distance( sliding.velocity ) );
	}

	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------
// Ballsocket
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialiseBallsocket( IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_BALLSOCKET;

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::PointConstraintSettings settings;
	settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	settings.mPoint1 = SourceToJolt::Distance( ballsocket.constraintPosition[0] ) - refBody->GetShape()->GetCenterOfMass();
	settings.mPoint2 = SourceToJolt::Distance( ballsocket.constraintPosition[1] ) - attBody->GetShape()->GetCenterOfMass();

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetEnabled( !pGroup && ballsocket.constraint.isActive );

	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------
// Fixed
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialiseFixed( IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_FIXED;

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::FixedConstraintSettings settings;
	settings.mAutoDetectPoint = true;

	m_pConstraint = settings.Create( *refBody, *attBody );

	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------
// Length
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialiseLength( IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_LENGTH;

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::DistanceConstraintSettings settings;
	settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	settings.mPoint1 = SourceToJolt::Distance( length.objectPosition[0] ) - refBody->GetShape()->GetCenterOfMass();
	settings.mPoint2 = SourceToJolt::Distance( length.objectPosition[1] ) - attBody->GetShape()->GetCenterOfMass();

	settings.mMinDistance = SourceToJolt::Distance( length.minLength );
	settings.mMaxDistance = SourceToJolt::Distance( length.totalLength );

	// Josh: UNDONE! Nothing seems to use strength on length ever
	// after analysing the codebase.
	// 
	//settings.mFrequency = 1.0f - length.constraint.strength;
	//if ( settings.mFrequency )
	//	settings.mDamping = 1.0f;

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetEnabled( !pGroup && length.constraint.isActive );

	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SaveConstraintSettings( JPH::StateRecorder &recorder )
{
	recorder.Write( m_ConstraintType );
	auto settings = m_pConstraint->GetConstraintSettings();
	settings->SaveBinaryState( recorder );
	m_pConstraint->SaveState( recorder );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SetGroup( IPhysicsConstraintGroup *pGroup )
{
	if ( m_pGroup )
		m_pGroup->RemoveConstraint( this );
	m_pGroup = static_cast< JoltPhysicsConstraintGroup * >( pGroup );
	if ( m_pGroup )
		m_pGroup->AddConstraint( this );
}

void JoltPhysicsConstraint::DestroyConstraint()
{
	if ( m_pObjAttached )
	{
		m_pObjAttached->RemoveDestroyedListener( this );
		m_pObjAttached = nullptr;
	}
	if ( m_pObjReference )
	{
		m_pObjReference->RemoveDestroyedListener( this );
		m_pObjReference = nullptr;
	}

	if ( m_pConstraint )
	{
		m_pPhysicsSystem->RemoveConstraint( m_pConstraint );
		m_pConstraint = nullptr;
	}
}
