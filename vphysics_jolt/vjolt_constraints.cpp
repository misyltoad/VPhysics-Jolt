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

static ConVar vjolt_constraint_velocity_substeps( "vjolt_constraint_velocity_substeps", "0" );
static ConVar vjolt_constraint_position_substeps( "vjolt_constraint_position_substeps", "0" );

static ConVar vjolt_ragdoll_min_torque_friction( "vjolt_ragdoll_min_torque_friction", "0.05" );

//-------------------------------------------------------------------------------------------------

static JPH::Vec3 HingePerpendicularVector( JPH::Vec3Arg dir )
{
	return fabsf( dir.GetX() ) < 0.57f
		? JPH::Vec3::sAxisX().Cross( dir ).Normalized()
		: JPH::Vec3::sAxisY().Cross( dir ).Normalized();
}

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

static std::optional<JoltMatrixAxes> DOFBitToAxis( uint32 uDOFMask )
{
	if ( uDOFMask & 0b001 )
		return MatrixAxis::X;
	else if ( uDOFMask & 0b010 )
		return MatrixAxis::Y;
	else if ( uDOFMask & 0b100 )
		return MatrixAxis::Z;
	else
		return std::nullopt;
}

struct RagdollLimits_t
{
	struct Limit_t
	{
		float Min;
		float Max;

		float GetRange() const
		{
			return Max - Min;
		}
	};

	RagdollLimits_t( const constraint_ragdollparams_t &ragdoll )
	{
		for ( int i = 0; i < 3; i++ )
		{
			if ( ragdoll.useClockwiseRotations )
			{
				lAxisLimitsRad[i].Min = DEG2RAD( -ragdoll.axes[i].maxRotation );
				lAxisLimitsRad[i].Max = DEG2RAD( -ragdoll.axes[i].minRotation );
			}
			else
			{
				lAxisLimitsRad[i].Min = DEG2RAD( ragdoll.axes[i].minRotation );
				lAxisLimitsRad[i].Max = DEG2RAD( ragdoll.axes[i].maxRotation );
			}
		}
	}

	uint32 GetDegreesOfFreedomMask() const
	{
		uint32 uDOFMask = 0;

		for ( int i = 0; i < 3; i++ )
		{
			if ( lAxisLimitsRad[i].GetRange() > DEG2RAD( 5.0f ) )
				uDOFMask |= 1u << i;
		}

		return uDOFMask;
	}

	Limit_t lAxisLimitsRad[3]{};
};

void JoltPhysicsConstraint::InitialiseRagdoll( IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_RAGDOLL;

	JPH::Mat44 constraintToReference = SourceToJolt::Matrix( ragdoll.constraintToReference );
	JPH::Mat44 constraintToAttached = SourceToJolt::Matrix( ragdoll.constraintToAttached );

	RagdollLimits_t limits = RagdollLimits_t( ragdoll );
	
	const uint32 uDOFMask = limits.GetDegreesOfFreedomMask();
	const uint32 uDOFCount = JPH::CountBits( uDOFMask );

	JPH::Body *pRefBody = m_pObjReference->GetBody();
	JPH::Body *pAttBody = m_pObjAttached->GetBody();

	matrix3x4_t refObjToWorld;
	m_pObjReference->GetPositionMatrix( &refObjToWorld );

	matrix3x4_t constraintToWorld;
	ConcatTransforms( refObjToWorld, ragdoll.constraintToReference, constraintToWorld );

	const float flMinTorqueFriction = vjolt_ragdoll_min_torque_friction.GetFloat();

	JPH::Constraint *pConstraint = nullptr;

	if ( uDOFCount == 0 )
	{
		JPH::FixedConstraintSettings settings;
		settings.mAutoDetectPoint = true;

		pConstraint = settings.Create( *pRefBody, *pAttBody );
	}
	else if ( uDOFCount == 1 )
	{
		JoltMatrixAxes eAxis = *DOFBitToAxis( uDOFMask );

		JPH::HingeConstraintSettings settings;
		settings.mPoint1 = SourceToJolt::Distance( GetColumn( constraintToWorld, MatrixAxis::Origin ) );
		settings.mPoint2 = SourceToJolt::Distance( GetColumn( constraintToWorld, MatrixAxis::Origin ) );
		settings.mHingeAxis1 = SourceToJolt::Unitless( GetColumn( constraintToWorld, eAxis ) );
		settings.mHingeAxis2 = SourceToJolt::Unitless( GetColumn( constraintToWorld, eAxis ) );
		settings.mNormalAxis1 = HingePerpendicularVector( settings.mHingeAxis1 );
		settings.mNormalAxis2 = HingePerpendicularVector( settings.mHingeAxis2 );
		settings.mLimitsMin = limits.lAxisLimitsRad[ eAxis ].Min;
		settings.mLimitsMax = limits.lAxisLimitsRad[ eAxis ].Max;
		settings.mMaxFrictionTorque = Max( flMinTorqueFriction, ragdoll.axes[ eAxis ].torque );
		
		pConstraint = settings.Create( *pRefBody, *pAttBody );
	}
	else
	{
		JPH::SwingTwistConstraintSettings settings;
		// Allow ~1deg either side to avoid joints glitching out.
		settings.mTwistMinAngle = Min( limits.lAxisLimitsRad[0].Min, DEG2RAD( -1.0f ) );
		settings.mTwistMaxAngle = Max( limits.lAxisLimitsRad[0].Max, DEG2RAD(  1.0f ) );
		settings.mNormalHalfConeAngle = Max( 0.5f * ( limits.lAxisLimitsRad[ MatrixAxis::X ].GetRange() ), DEG2RAD( 1.0f ) );
		settings.mPlaneHalfConeAngle = Max( 0.5f * ( limits.lAxisLimitsRad[ MatrixAxis::Y ].GetRange() ), DEG2RAD( 1.0f ) );

		settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;

		settings.mPosition1 = constraintToReference.GetTranslation() - pRefBody->GetShape()->GetCenterOfMass();
		settings.mTwistAxis1 = constraintToReference.GetAxisX();
		settings.mPlaneAxis1 = constraintToReference.GetAxisY();

		settings.mPosition2 = constraintToAttached.GetTranslation() - pAttBody->GetShape()->GetCenterOfMass();
		settings.mTwistAxis2 = constraintToAttached.GetAxisX();
		settings.mPlaneAxis2 = constraintToAttached.GetAxisY();

		settings.mMaxFrictionTorque = Max( flMinTorqueFriction, ( ragdoll.axes[0].torque + ragdoll.axes[1].torque + ragdoll.axes[2].torque ) / 3.0f );

		pConstraint = settings.Create( *pRefBody, *pAttBody );
	}

	if ( ragdoll.onlyAngularLimits )
	{
		Log_Warning( LOG_VJolt, "Only angular limits. Need a way to disable the linear part of the constraint.\n" );
	}

	const bool bActive = !m_pGroup && ragdoll.constraint.isActive;

	m_pConstraint = pConstraint;
	m_pConstraint->SetEnabled( bActive );
	m_pPhysicsSystem->AddConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------
// Hinge
//-------------------------------------------------------------------------------------------------

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
	settings.mNumVelocityStepsOverride = vjolt_constraint_velocity_substeps.GetInt();
	settings.mNumPositionStepsOverride = vjolt_constraint_position_substeps.GetInt();
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
	settings.mNumVelocityStepsOverride = vjolt_constraint_velocity_substeps.GetInt();
	settings.mNumPositionStepsOverride = vjolt_constraint_position_substeps.GetInt();
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
	settings.mNumVelocityStepsOverride = vjolt_constraint_velocity_substeps.GetInt();
	settings.mNumPositionStepsOverride = vjolt_constraint_position_substeps.GetInt();
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
// Pulley
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialisePulley( IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_PULLEY;

	// Get our bodies
	JPH::Body* refBody = m_pObjReference->GetBody();
	JPH::Body* attBody = m_pObjAttached->GetBody();

	JPH::PulleyConstraintSettings settings;
	settings.mNumVelocityStepsOverride = vjolt_constraint_velocity_substeps.GetInt();
	settings.mNumPositionStepsOverride = vjolt_constraint_position_substeps.GetInt();
	settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	settings.mBodyPoint1 = SourceToJolt::Distance( pulley.objectPosition[0] ) - refBody->GetShape()->GetCenterOfMass();
	settings.mBodyPoint2 = SourceToJolt::Distance( pulley.objectPosition[1] ) - attBody->GetShape()->GetCenterOfMass();

	settings.mFixedPoint1 = SourceToJolt::Distance( pulley.pulleyPosition[0] );
	settings.mFixedPoint2 = SourceToJolt::Distance( pulley.pulleyPosition[1] );

	settings.mRatio = pulley.gearRatio;

	settings.mMaxLength = SourceToJolt::Distance( pulley.totalLength ); // PiMoN: from my testing, it is the same value as Jolt would calculate automatically

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetEnabled( !pGroup && pulley.constraint.isActive );

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
