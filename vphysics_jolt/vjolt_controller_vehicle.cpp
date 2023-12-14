
#include "cbase.h"

#include "vjolt_layers.h"

#include "vjolt_controller_vehicle.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//------------------------------------------------------------------------------------------------

static ConVar vjolt_vehicle_wheel_debug( "vjolt_vehicle_wheel_debug", "0", FCVAR_CHEAT );

static ConVar vjolt_vehicle_throttle_opposition_limit( "vjolt_vehicle_throttle_opposition_limit", "5", FCVAR_NONE,
	"Below what speed should we be attempting to drive/climb with handbrake on to avoid falling down." );

//------------------------------------------------------------------------------------------------

static const JPH::Vec3 VehicleUpVector		= JPH::Vec3( 0, 0, 1 );
static const JPH::Vec3 VehicleForwardVector	= JPH::Vec3( 0, 1, 0 );

static const char *VehicleTypeToName( unsigned int VehicleType )
{
	switch ( VehicleType )
	{
		case VEHICLE_TYPE_CAR_WHEELS:		return "Car Wheels";
		case VEHICLE_TYPE_CAR_RAYCAST:		return "Car Raycast";
		case VEHICLE_TYPE_JETSKI_RAYCAST:	return "Jetski Raycast";
		case VEHICLE_TYPE_AIRBOAT_RAYCAST:	return "Airboat Raycast";
		default:							return "Unknown";
	}
}

JPH::Ref< JPH::VehicleCollisionTester > CreateVehicleCollisionTester( unsigned int VehicleType, float LargestWheelRadius )
{
	switch ( VehicleType )
	{
	default:
		Log_Warning( LOG_VJolt, "Don't know how to make vehicle type: %s (%u).\n", VehicleTypeToName( VehicleType ), VehicleType );
		[[ fallthrough ]];
	case VEHICLE_TYPE_CAR_WHEELS:
		return new JPH::VehicleCollisionTesterCastSphere( Layers::MOVING, LargestWheelRadius, VehicleUpVector );
	}
}

//------------------------------------------------------------------------------------------------

JoltPhysicsVehicleController::JoltPhysicsVehicleController( JoltPhysicsEnvironment* pEnvironment, JPH::PhysicsSystem* pPhysicsSystem, JoltPhysicsObject* pVehicleBodyObject, const vehicleparams_t& params, unsigned int nVehicleType, IPhysicsGameTrace* pGameTrace )
	: m_pEnvironment( pEnvironment )
	, m_pPhysicsSystem( pPhysicsSystem )
	, m_pCarBodyObject( pVehicleBodyObject )
	, m_VehicleType( nVehicleType )
	, m_VehicleParams( params )
{
	JPH::VehicleConstraintSettings vehicle;
	vehicle.mUp					= VehicleUpVector;
	vehicle.mForward			= VehicleForwardVector;
	vehicle.mDrawConstraintSize = 0.1f;
	CreateWheels( vehicle );
	vehicle.mController			= CreateVehicleController();

	m_Tester = CreateVehicleCollisionTester( nVehicleType, m_InternalState.LargestWheelRadius );

	m_pCarBodyObject->AddDestroyedListener( this );
	m_VehicleConstraint = new JPH::VehicleConstraint( *m_pCarBodyObject->GetBody(), vehicle );
	m_pPhysicsSystem->AddConstraint( m_VehicleConstraint );
	m_pPhysicsSystem->AddStepListener( m_VehicleConstraint );
}

JoltPhysicsVehicleController::~JoltPhysicsVehicleController()
{
	DetachObject();

	for ( auto &wheel : m_Wheels )
		m_pEnvironment->DestroyObject( wheel.pObject );
	m_Wheels.clear();
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::Update( float dt, vehicle_controlparams_t &controls )
{
	m_ControlParams = controls;

	UpdateBooster( dt );

	HandleBoostKey();
}

const vehicle_operatingparams_t &JoltPhysicsVehicleController::GetOperatingParams()
{
	return m_OperatingParams;
}

const vehicleparams_t &JoltPhysicsVehicleController::GetVehicleParams()
{
	return m_VehicleParams;
}

vehicleparams_t &JoltPhysicsVehicleController::GetVehicleParamsForChange()
{
	return m_VehicleParams;
}

float JoltPhysicsVehicleController::UpdateBooster( float dt )
{
	m_InternalState.BoostDelay				= Max( m_InternalState.BoostDelay - dt, 0.0f );
	m_InternalState.BoosterRemainingTime	= Max( m_InternalState.BoosterRemainingTime - dt, 0.0f );

	return m_InternalState.BoostDelay;
}

int JoltPhysicsVehicleController::GetWheelCount()
{
	return int( m_Wheels.size() );
}

IPhysicsObject *JoltPhysicsVehicleController::GetWheel( int index )
{
	if ( index >= int( m_Wheels.size() ) )
		return nullptr;

	return m_Wheels[ index ].pObject;
}

bool JoltPhysicsVehicleController::GetWheelContactPoint( int index, Vector *pContactPoint, int *pSurfaceProps )
{
	if ( index < int( m_Wheels.size() ) && m_VehicleConstraint->GetWheels()[ index ]->HasContact() )
	{
		if ( pContactPoint )
			*pContactPoint = JoltToSource::Distance( m_VehicleConstraint->GetWheels()[ index ]->GetContactPosition() );

		// TODO(Josh): This!
		if ( pSurfaceProps )
			*pSurfaceProps = 0;

		return true;
	}
	else
	{
		if ( pContactPoint )
			*pContactPoint = vec3_origin;

		if ( pSurfaceProps )
			*pSurfaceProps = 0;

		return false;
	}
}

void JoltPhysicsVehicleController::SetSpringLength( int wheelIndex, float length )
{

}

void JoltPhysicsVehicleController::SetWheelFriction( int wheelIndex, float friction )
{

}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::OnVehicleEnter()
{
	// Undo any damping we may have set to slow the boat when
	// we got out.
	if ( m_VehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST )
	{
		float flDampSpeed = 0.0f;
		float flDampRotSpeed = 0.0f;
		m_pCarBodyObject->SetDamping( &flDampSpeed, &flDampRotSpeed );
	}
}

void JoltPhysicsVehicleController::OnVehicleExit()
{
	// If we are an airboat, set a bunch of damping to slow us down.
	if ( m_VehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST )
	{
		float flDampSpeed = 1.0f;
		float flDampRotSpeed = 1.0f;
		m_pCarBodyObject->SetDamping( &flDampSpeed, &flDampRotSpeed );
	}

	SetEngineDisabled( false );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::SetEngineDisabled( bool bDisable )
{
	m_InternalState.EngineDisabled = bDisable;
}

bool JoltPhysicsVehicleController::IsEngineDisabled()
{
	return m_InternalState.EngineDisabled;
}

void JoltPhysicsVehicleController::GetCarSystemDebugData( vehicle_debugcarsystem_t &debugCarSystem )
{

}

void JoltPhysicsVehicleController::VehicleDataReload()
{

}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsVehicleController::GetSpeed()
{
	const Vector orientation = GetColumn( GetBodyMatrix(), MatrixAxis::Left );
	return orientation.Dot( m_pCarBodyObject->GetVelocity() );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::HandleBoostKey()
{
	// Handle triggering boosting if the key is pressed and we aren't currently boosting or in cooldown.
	if ( m_ControlParams.boost && !m_InternalState.BoostDelay && !m_InternalState.BoosterRemainingTime )
	{
		m_InternalState.BoosterRemainingTime	= m_VehicleParams.engine.boostDuration;
		m_InternalState.BoostDelay				= m_VehicleParams.engine.boostDuration + m_VehicleParams.engine.boostDelay;
	}
}

void JoltPhysicsVehicleController::HandleBoostDecay()
{
	// Decay the boost time if we are currently boosting or have a delay.
	if ( m_VehicleParams.engine.boostDuration || m_VehicleParams.engine.boostDelay )
	{
		m_OperatingParams.boostTimeLeft = m_InternalState.BoostDelay
			? 100.0f - ( 100.0f * ( m_InternalState.BoostDelay / ( m_VehicleParams.engine.boostDuration + m_VehicleParams.engine.boostDelay ) ) )
			: 100.0f;
	}
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::OnPreSimulate( float flDeltaTime )
{
	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	// With any user input, assure that the car is active
	if ( m_ControlParams.steering != 0.0f || m_ControlParams.throttle != 0.0f || m_ControlParams.brake != 0.0f || m_ControlParams.handbrake )
		bodyInterface.ActivateBody( m_pCarBodyObject->GetBodyID() );

	bool bHandbrake = m_ControlParams.handbrake;

	// Don't throttle when holding handbrake (like Source)
	float flThrottle = bHandbrake ? 0.0f : m_ControlParams.throttle;
	
	// Apply a little brake without throttle to stop the vehicle from coasting (like Source).
	const bool bCoasting = flThrottle == 0.0f && m_ControlParams.brake == 0.0f && !bHandbrake;
	const float flBrake = bCoasting ? 0.1f : m_ControlParams.brake;

	const float ThrottleOpositionSpeed = vjolt_vehicle_throttle_opposition_limit.GetFloat();

	// Enable the handbrake when going at low speeds to avoid slipping when going up hill.
	if ( ( flThrottle < 0.0f && m_OperatingParams.speed > ThrottleOpositionSpeed ) ||
		( flThrottle > 0.0f && m_OperatingParams.speed < -ThrottleOpositionSpeed ) )
		bHandbrake = true;

	// Are we boosting?
	float flTotalTorqueMultiplier = 1.0f;
	if ( m_InternalState.BoosterRemainingTime != 0.0f )
	{
		GetWheeledVehicleController()->GetEngine().SetCurrentRPM(m_VehicleParams.engine.maxRPM);
		// Slam the throttle to 1, neeeowm!
		m_ControlParams.throttle = 1.0f;
		flThrottle = 1.0f;

		const float flSpeedFactor = RemapValClamped( fabsf( m_OperatingParams.speed ), 0, m_VehicleParams.engine.maxSpeed, 0.1f, 1.0f );
		const float flTurnFactor = 1.0f - ( fabsf( m_ControlParams.steering ) * 0.95f );
		// Josh: * 2 as the original torque stuff in Source was based around 0.5 being the max, and 1.0 being boost.
		const float flDampedBoost = 2.0f * m_VehicleParams.engine.boostForce * flSpeedFactor * flTurnFactor;

		if ( flDampedBoost > flTotalTorqueMultiplier )
			flTotalTorqueMultiplier = flDampedBoost;
	}

	// Update the torque factors as we may be boosting and be > 1.
	// TODO(Josh): More than 2 wheels per axle.
	VJoltAssert( m_VehicleParams.wheelsPerAxle == 2 );
	for ( int i = 0; i < m_VehicleParams.axleCount; i++ )
		GetWheeledVehicleController()->GetDifferentials()[i].mEngineTorqueRatio = flTotalTorqueMultiplier * m_VehicleParams.axles[i].torqueFactor;

	// Pass the input on to the constraint
	GetWheeledVehicleController()->SetDriverInput( flThrottle, m_ControlParams.steering, flBrake, bHandbrake ? 1.0f : 0.0f );

	// Set the collision tester
	m_VehicleConstraint->SetVehicleCollisionTester( m_Tester );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::OnPostSimulate( float flDeltaTime )
{
	// Draw our wheels (this needs to be done in the pre update since we draw the bodies too in the state before the step)

	float flSteeringAngle = 0.0f;
	m_OperatingParams.wheelsInContact = 0;
	m_OperatingParams.wheelsNotInContact = 0;
	for ( int w = 0; w < GetWheelCount(); w++ )
	{
		const JPH::WheelSettings *settings = m_VehicleConstraint->GetWheels()[w]->GetSettings();
		// The cyclinder we draw is aligned with Y so we specify that as rotational axis
		JPH::Mat44 wheelTransform = m_VehicleConstraint->GetWheelWorldTransform( w, JPH::Vec3( 1, 0, 0 ),  JPH::Vec3( 0, 0, 1 ) );
		
		// Find our greatest steering angle.
		float flWheelSteeringAngle = JoltToSource::Angle( m_VehicleConstraint->GetWheels()[w]->GetSteerAngle() );
		if ( fabsf( flWheelSteeringAngle ) > fabsf( flSteeringAngle ) )
			flSteeringAngle = flWheelSteeringAngle;

		Vector newPos = JoltToSource::Distance( wheelTransform.GetTranslation() );
		// TODO(Josh): This triggers JPH_ASSERT(mCol[3] == Vec4(0, 0, 0, 1));
		// what to do about that?..
		// We just want the local rotation, and this seems to work (?)
		QAngle newQuat = JoltToSource::Angle( wheelTransform.GetQuaternion() );
		m_Wheels[ w ].pObject->EnableCollisions( false );
		// Set dummy wheel object pos/angles so the game code can update pose positions for wheels.
		m_Wheels[ w ].pObject->SetPosition( newPos, newQuat, true );
		// Wake it up so that the game bothers to do pose positions.
		m_Wheels[ w ].pObject->Wake();

		if ( m_VehicleConstraint->GetWheels()[w]->HasContact() )
			m_OperatingParams.wheelsInContact++;
		else
			m_OperatingParams.wheelsNotInContact++;

		IVJoltDebugOverlay *pDebugOverlay = JoltPhysicsInterface::GetInstance().GetDebugOverlay();
		if ( vjolt_vehicle_wheel_debug.GetBool() && pDebugOverlay )
		{
			const Vector vecWheelPos = JoltToSource::Distance( wheelTransform.GetTranslation() );
			const Vector vecWheelSize = JoltToSource::Distance( JPH::Vec3( settings->mWidth / 2.0f, settings->mRadius, settings->mRadius ) );

			pDebugOverlay->AddBoxOverlay(
				vecWheelPos,
				-vecWheelSize, vecWheelSize,
				newQuat,
				255, 0, 255, 100,
				-1.0f );
		}
	}

	m_OperatingParams.gear			= GetWheeledVehicleController()->GetTransmission().GetCurrentGear();
	m_OperatingParams.engineRPM		= GetWheeledVehicleController()->GetEngine().GetCurrentRPM();
	m_OperatingParams.speed			= GetSpeed();
	m_OperatingParams.steeringAngle = -flSteeringAngle;
	m_OperatingParams.boostDelay	= m_InternalState.BoostDelay;
	HandleBoostDecay();
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::CreateWheel( JPH::VehicleConstraintSettings &vehicleSettings, matrix3x4_t& bodyMatrix, int axleIdx, int wheelIdx )
{
	const vehicle_axleparams_t &axle = m_VehicleParams.axles[ axleIdx ];

	const Vector wheelPositionLocal = axle.offset +
		( ( wheelIdx % 2 == 1 ) ? axle.wheelOffset : -axle.wheelOffset );

	Vector wheelPositionWorld;
	VectorTransform( wheelPositionLocal, bodyMatrix, wheelPositionWorld );

	// Josh: Good enough heuristic.
	const float wheelRadius = axle.wheels.radius;
	const float wheelWidth = wheelRadius / 2.0f;

	// Josh: Area of a cylinder = π.h.r^2
	// Using radius in terms of Source units as we pass this to CreateSphereObject.
	const float wheelVolume = M_PI * wheelWidth * Cube( wheelRadius );

	{
		objectparams_t wheelParams =
		{
			.mass		= axle.wheels.mass,
			.inertia	= axle.wheels.inertia,
			.damping	= axle.wheels.damping,
			.rotdamping	= axle.wheels.rotdamping,
			.pName		= "VehicleWheel",
			.pGameData	= m_pCarBodyObject->GetGameData(),
			.volume		= wheelVolume,
		};
		IPhysicsObject *pWheelObject = m_pEnvironment->CreateSphereObject(
			wheelRadius, axle.wheels.materialIndex,
			wheelPositionWorld, QAngle(),
			&wheelParams, false );

		JoltPhysicsObject *pJoltWheelObject = static_cast< JoltPhysicsObject * >( pWheelObject );

		pJoltWheelObject->SetGameFlags( m_pCarBodyObject->GetGameFlags() );
		pJoltWheelObject->SetCallbackFlags( CALLBACK_IS_VEHICLE_WHEEL );
		// Josh: The wheel is a fake object, so disable collisions on it.
		pJoltWheelObject->EnableCollisions( false );

		m_Wheels.push_back( JoltPhysicsWheel{ .pObject = pJoltWheelObject } );
	}

	const float steeringAngle = DEG2RAD( Max( m_VehicleParams.steering.degreesSlow, m_VehicleParams.steering.degreesFast ) );
	const float additionalLength = SourceToJolt::Distance( axle.wheels.springAdditionalLength );

	Vector gravity;
	m_pEnvironment->GetGravity( &gravity );

	JPH::WheelSettingsWV *wheelSettings = new JPH::WheelSettingsWV;
	wheelSettings->mPosition			= SourceToJolt::Distance( wheelPositionLocal );
	wheelSettings->mSuspensionDirection = JPH::Vec3( 0, 0, -1 );
	wheelSettings->mSteeringAxis		= JPH::Vec3( 0, 0, 1 );
	wheelSettings->mWheelUp				= JPH::Vec3( 0, 0, 1 );
	wheelSettings->mWheelForward		= JPH::Vec3( 0, 1, 0 );
	wheelSettings->mAngularDamping		= axle.wheels.rotdamping;
	// TODO(Josh): What about more than 4 wheels?
	wheelSettings->mMaxSteerAngle		= axleIdx == 0 ? steeringAngle : 0.0f;
	wheelSettings->mRadius				= SourceToJolt::Distance( axle.wheels.radius );
	wheelSettings->mWidth				= SourceToJolt::Distance( wheelWidth );
	wheelSettings->mInertia				= 0.5f * axle.wheels.mass * ( wheelSettings->mRadius * wheelSettings->mRadius );
	wheelSettings->mSuspensionMinLength = 0;
	wheelSettings->mSuspensionMaxLength = additionalLength;
	wheelSettings->mSuspensionSpring.mMode = JPH::ESpringMode::StiffnessAndDamping;
	// Source has these divided by the mass of the vehicle for some reason.
	// Convert these to a stiffness of k, in N/m...
	wheelSettings->mSuspensionSpring.mStiffness = axle.suspension.springConstant * m_pCarBodyObject->GetMass();
	wheelSettings->mSuspensionSpring.mDamping = axle.suspension.springDamping * m_pCarBodyObject->GetMass();
	if ( axle.wheels.frictionScale )
	{
		wheelSettings->mLateralFriction.AddPoint( 1.0f, axle.wheels.frictionScale );
		wheelSettings->mLongitudinalFriction.AddPoint( 1.0f, axle.wheels.frictionScale );
	}

	// TODO: We may want to update this every pre-simulation to account for changing gravity.
	wheelSettings->mMaxBrakeTorque =
		0.5f *
		SourceToJolt::Distance( gravity.Length() ) *
		( m_pCarBodyObject->GetMass() + m_TotalWheelMass ) *
		axle.brakeFactor *
		SourceToJolt::Distance( axle.wheels.radius );

	vehicleSettings.mWheels.push_back( wheelSettings );
	m_InternalState.LargestWheelRadius = Max( m_InternalState.LargestWheelRadius, SourceToJolt::Distance( wheelWidth ) );
}

void JoltPhysicsVehicleController::CreateWheels( JPH::VehicleConstraintSettings &vehicleSettings )
{
	matrix3x4_t carBodyMtx = GetBodyMatrix();

	m_Wheels.reserve( m_VehicleParams.axleCount * m_VehicleParams.wheelsPerAxle );
	vehicleSettings.mAntiRollBars.reserve( m_VehicleParams.axleCount );

	m_TotalWheelMass = 0.0f;
	for ( int axle = 0; axle < m_VehicleParams.axleCount; axle++ )
		m_TotalWheelMass += m_VehicleParams.axles[ axle ].wheels.mass * m_VehicleParams.wheelsPerAxle;

	for ( int axle = 0; axle < m_VehicleParams.axleCount; axle++ )
	{
		for ( int wheel = 0; wheel < m_VehicleParams.wheelsPerAxle; wheel++ )
			CreateWheel( vehicleSettings, carBodyMtx, axle, wheel );

		// TODO(Josh): More than 2 wheels per axle.
		VJoltAssert( m_VehicleParams.wheelsPerAxle == 2 );
		JPH::VehicleAntiRollBar rollbar;
		rollbar.mLeftWheel	= ( axle * m_VehicleParams.wheelsPerAxle );
		rollbar.mRightWheel	= ( axle * m_VehicleParams.wheelsPerAxle ) + 1;
		vehicleSettings.mAntiRollBars.push_back( rollbar );
	}
}

JPH::WheeledVehicleControllerSettings *JoltPhysicsVehicleController::CreateVehicleController()
{
	static constexpr float HorsePowerToWatts = 745.7f;

	JPH::WheeledVehicleControllerSettings *pController = new JPH::WheeledVehicleControllerSettings;
	// Josh:
	// T = ( 745.7 * P ) / ( 2 * PI * ( RPM / 60 ) )
	pController->mEngine.mMaxTorque = ( HorsePowerToWatts * m_VehicleParams.engine.horsepower ) / ( 2.0f * M_PI * ( m_VehicleParams.engine.maxRPM / 60.0f ) );
	// Josh: Fudge
	pController->mEngine.mMinRPM = Max( m_VehicleParams.engine.shiftDownRPM - 300, 0.0f );
	pController->mEngine.mMaxRPM = m_VehicleParams.engine.maxRPM;
	pController->mEngine.mAngularDamping = 0.0f;

	pController->mTransmission.mMode = m_VehicleParams.engine.isAutoTransmission ? JPH::ETransmissionMode::Auto : JPH::ETransmissionMode::Manual;
	pController->mTransmission.mGearRatios.clear();
	for ( int i = 0; i < m_VehicleParams.engine.gearCount; i++ )
		pController->mTransmission.mGearRatios.push_back( m_VehicleParams.engine.gearRatio[ i ] );

	pController->mTransmission.mReverseGearRatios.clear();
	pController->mTransmission.mReverseGearRatios.push_back( -m_VehicleParams.engine.gearRatio[0] );

	pController->mTransmission.mShiftUpRPM = m_VehicleParams.engine.shiftUpRPM;
	pController->mTransmission.mShiftDownRPM = m_VehicleParams.engine.shiftDownRPM;

	pController->mDifferentials.reserve( m_VehicleParams.axleCount );
	for ( int i = 0; i < m_VehicleParams.axleCount; i++ )
	{
		// TODO(Josh): More than 2 wheels per axle.
		VJoltAssert( m_VehicleParams.wheelsPerAxle == 2 );
		JPH::VehicleDifferentialSettings differential;
		differential.mLeftWheel			= ( i * m_VehicleParams.wheelsPerAxle );
		differential.mRightWheel		= ( i * m_VehicleParams.wheelsPerAxle ) + 1;
		differential.mEngineTorqueRatio = m_VehicleParams.axles[ i ].torqueFactor;

		pController->mDifferentials.push_back( differential );
	}

	return pController;
}

JPH::WheeledVehicleController *JoltPhysicsVehicleController::GetWheeledVehicleController()
{
	return static_cast<JPH::WheeledVehicleController *>( m_VehicleConstraint->GetController() );
}

//------------------------------------------------------------------------------------------------

matrix3x4_t JoltPhysicsVehicleController::GetBodyMatrix() const
{
	matrix3x4_t value;
	m_pCarBodyObject->GetPositionMatrix( &value );
	return value;
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	if ( m_pCarBodyObject == pObject )
		DetachObject();
}

void JoltPhysicsVehicleController::DetachObject()
{
	if ( m_pCarBodyObject )
	{
		m_pCarBodyObject->RemoveDestroyedListener( this );

		// Remove the listeners and constraint now, we can never
		// attach to another body.
		m_pPhysicsSystem->RemoveConstraint( m_VehicleConstraint );
		m_pPhysicsSystem->RemoveStepListener( m_VehicleConstraint );

		m_pCarBodyObject = nullptr;
	}
}
