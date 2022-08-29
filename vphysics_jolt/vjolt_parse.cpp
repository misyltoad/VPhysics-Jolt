
#include "cbase.h"

#include "vjolt_keyvalues_schema.h"
#include "vjolt_surfaceprops.h"

#include "vjolt_parse.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

class JoltPhysicsParseKV final : public IVPhysicsKeyParser
{
public:
	JoltPhysicsParseKV( KeyValues *pKV );
	~JoltPhysicsParseKV() override;

	const char* GetCurrentBlockName() override;
	bool		Finished() override;
	void		ParseSolid( solid_t *pSolid, IVPhysicsKeyHandler *unknownKeyHandler ) override;
	void		ParseFluid( fluid_t *pFluid, IVPhysicsKeyHandler *unknownKeyHandler ) override;
	void		ParseRagdollConstraint( constraint_ragdollparams_t *pConstraint, IVPhysicsKeyHandler *unknownKeyHandler ) override;
	void		ParseSurfaceTable( int *table, IVPhysicsKeyHandler *unknownKeyHandler ) override;
	void		ParseCustom( void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler ) override;
	void		ParseVehicle( vehicleparams_t *pVehicle, IVPhysicsKeyHandler *unknownKeyHandler ) override;
	void		SkipBlock() override;
	void		ParseCollisionRules( ragdollcollisionrules_t *pRules, IVPhysicsKeyHandler *unknownKeyHandler ) override_asw;
	void		ParseRagdollAnimatedFriction( ragdollanimatedfriction_t *pFriction, IVPhysicsKeyHandler *unknownKeyHandler ) override_asw;	

private:

	void		NextBlock();

	KeyValues *m_pKV;
	KeyValues *m_pCurrentBlock;
};

//-------------------------------------------------------------------------------------------------

struct JoltPhysicsCollisionRulesHelper
{
	ragdollcollisionrules_t	Rules;
	IVPhysicsKeyHandler		*pUnknownKeyHandler;

	JoltPhysicsIntPair		CollisionPair;
};

//-------------------------------------------------------------------------------------------------

static const JoltKVSchemaProp_t kSolidDescs[] =
{
	{ "Index",					KVSCHEMA_DESC( solid_t, index ),							FillIntProp },
#ifdef GAME_ASW_OR_NEWER
	{ "Contents",				KVSCHEMA_DESC( solid_t, contents ),							FillIntProp },
#endif
	{ "Name",					KVSCHEMA_DESC( solid_t, name ),								FillStringProp },
	{ "Parent",					KVSCHEMA_DESC( solid_t, parent ),							FillStringProp },
	{ "Mass",					KVSCHEMA_DESC( solid_t, params.mass ),						FillFloatProp },
	{ "SurfaceProp",			KVSCHEMA_DESC( solid_t, surfaceprop ),						FillStringProp },
	{ "MassCenterOverride",		KVSCHEMA_DESC( solid_t, massCenterOverride ),				FillVectorProp,
		[]( void *pBaseObject )
		{
			// Josh:
			// If we ended up setting this, we need to update the pointer
			// of the params to point to our Vector.
			solid_t *pSolid = reinterpret_cast< solid_t * >( pBaseObject );
			pSolid->params.massCenterOverride = &pSolid->massCenterOverride;
		}
	},
	{ "Damping",				KVSCHEMA_DESC( solid_t, params.damping ),					FillFloatProp },
	{ "RotDamping",				KVSCHEMA_DESC( solid_t, params.rotdamping ),				FillFloatProp },
	{ "Drag",					KVSCHEMA_DESC( solid_t, params.dragCoefficient ),			FillFloatProp },
	//{ "RollingDrag",			KVSCHEMA_DESC( solid_t, params.rollingDrag ),				FillFloatProp },
	{ "Inertia",				KVSCHEMA_DESC( solid_t, params.inertia ),					FillFloatProp },
	{ "RotInertiaLimit",		KVSCHEMA_DESC( solid_t, params.rotInertiaLimit ),			FillFloatProp },
	{ "Volume",					KVSCHEMA_DESC( solid_t, params.volume ),					FillFloatProp },
};

static const JoltKVSchemaProp_t kFluidDescs[] =
{
	{ "Index",					KVSCHEMA_DESC( fluid_t, index ),							FillIntProp },
	{ "SurfaceProp",			KVSCHEMA_DESC( fluid_t, surfaceprop ),						FillStringProp },
	{ "Damping",				KVSCHEMA_DESC( fluid_t, params.damping ),					FillFloatProp },
	{ "SurfacePlane",			KVSCHEMA_DESC( fluid_t, params.surfacePlane ),				FillVector4DProp },
	{ "CurrentVelocity",		KVSCHEMA_DESC( fluid_t, params.currentVelocity ),			FillVectorProp },
	{ "Contents",				KVSCHEMA_DESC( fluid_t, params.contents ),					FillIntProp },
};

static const JoltKVSchemaProp_t kRagdollDescs[] =
{
	{ "Parent",					KVSCHEMA_DESC( constraint_ragdollparams_t, parentIndex ),			FillIntProp },
	{ "Child",					KVSCHEMA_DESC( constraint_ragdollparams_t, childIndex ),			FillIntProp },

	{ "XMin",					KVSCHEMA_DESC( constraint_ragdollparams_t, axes[0].minRotation ),	FillFloatProp },
	{ "YMin",					KVSCHEMA_DESC( constraint_ragdollparams_t, axes[1].minRotation ),	FillFloatProp },
	{ "ZMin",					KVSCHEMA_DESC( constraint_ragdollparams_t, axes[2].minRotation ),	FillFloatProp },

	{ "XMax",					KVSCHEMA_DESC( constraint_ragdollparams_t, axes[0].maxRotation ),	FillFloatProp },
	{ "YMax",					KVSCHEMA_DESC( constraint_ragdollparams_t, axes[1].maxRotation ),	FillFloatProp },
	{ "ZMax",					KVSCHEMA_DESC( constraint_ragdollparams_t, axes[2].maxRotation ),	FillFloatProp },

	{ "XFriction",				KVSCHEMA_DESC( constraint_ragdollparams_t, axes[0].torque ),		FillFloatProp,
		[]( void *pBaseObject )
		{
			constraint_ragdollparams_t *pParams = reinterpret_cast< constraint_ragdollparams_t * >( pBaseObject );
			pParams->axes[0].angularVelocity = 0;
		},
	},
	{ "YFriction",				KVSCHEMA_DESC( constraint_ragdollparams_t, axes[1].torque ),		FillFloatProp,
		[]( void *pBaseObject )
		{
			constraint_ragdollparams_t *pParams = reinterpret_cast< constraint_ragdollparams_t * >( pBaseObject );
			pParams->axes[1].angularVelocity = 0;
		},
	},
	{ "ZFriction",				KVSCHEMA_DESC( constraint_ragdollparams_t, axes[2].torque ),		FillFloatProp,
		[]( void *pBaseObject )
		{
			constraint_ragdollparams_t *pParams = reinterpret_cast< constraint_ragdollparams_t * >( pBaseObject );
			pParams->axes[2].angularVelocity = 0;
		},
	},
};

static const JoltKVSchemaProp_t kVehicleAxleWheelDescs[] =
{
	{ "Radius",			KVSCHEMA_DESC( vehicle_wheelparams_t, radius ),				FillFloatProp },
	{ "Mass",			KVSCHEMA_DESC( vehicle_wheelparams_t, mass ),				FillFloatProp },
	{ "Intertia",		KVSCHEMA_DESC( vehicle_wheelparams_t, inertia ),			FillFloatProp },
	{ "Damping",		KVSCHEMA_DESC( vehicle_wheelparams_t, damping ),			FillFloatProp },
	{ "RotDamping",		KVSCHEMA_DESC( vehicle_wheelparams_t, rotdamping ),			FillFloatProp },
	{ "FrictionScale",	KVSCHEMA_DESC( vehicle_wheelparams_t, frictionScale ),		FillFloatProp },
	{ "Material",		KVSCHEMA_DESC( vehicle_wheelparams_t, materialIndex ),		FillSurfaceProp },
	{ "SkidMaterial",	KVSCHEMA_DESC( vehicle_wheelparams_t, skidMaterialIndex ),	FillSurfaceProp },
	{ "BrakeMaterial",	KVSCHEMA_DESC( vehicle_wheelparams_t, brakeMaterialIndex ),	FillSurfaceProp },
};

static const JoltKVSchemaProp_t kVehicleAxleSuspensionDescs[] =
{
	{ "SpringConstant",				KVSCHEMA_DESC( vehicle_suspensionparams_t, springConstant ),			FillFloatProp },
	{ "SpringDamping",				KVSCHEMA_DESC( vehicle_suspensionparams_t, springDamping ),				FillFloatProp },
	{ "StabilizerConstant",			KVSCHEMA_DESC( vehicle_suspensionparams_t, stabilizerConstant ),		FillFloatProp },
	{ "SpringDampingCompression",	KVSCHEMA_DESC( vehicle_suspensionparams_t, springDampingCompression ),	FillFloatProp },
	{ "MaxBodyForce",				KVSCHEMA_DESC( vehicle_suspensionparams_t, maxBodyForce ),				FillFloatProp },
};

static const JoltKVSchemaProp_t kVehicleAxleDescs[] =
{
	{ "Wheel",			KVSCHEMA_DESC( vehicle_axleparams_t, wheels ),
		{
			sizeof( vehicle_wheelparams_t ),
			[]( KeyValues *pProp, void *pPtr, size_t size )
			{
				vehicle_wheelparams_t *pWheelParams = reinterpret_cast< vehicle_wheelparams_t * >( pPtr );
				ParseJoltKVSchema( pProp, kVehicleAxleWheelDescs, ARRAYSIZE( kVehicleAxleWheelDescs ), pWheelParams );
			}
		}
	},
	{ "Suspension",		KVSCHEMA_DESC( vehicle_axleparams_t, suspension ),
		{
			sizeof( vehicle_suspensionparams_t ),
			[]( KeyValues *pProp, void *pPtr, size_t size )
			{
				vehicle_suspensionparams_t *pSuspensionParams = reinterpret_cast< vehicle_suspensionparams_t * >( pPtr );
				ParseJoltKVSchema( pProp, kVehicleAxleSuspensionDescs, ARRAYSIZE( kVehicleAxleSuspensionDescs ), pSuspensionParams );
			}
		}
	},
	{ "Offset",			KVSCHEMA_DESC( vehicle_axleparams_t, offset ),			FillVectorProp },
	{ "WheelOffset",	KVSCHEMA_DESC( vehicle_axleparams_t, wheelOffset ),		FillVectorProp },
	{ "TorqueFactor",	KVSCHEMA_DESC( vehicle_axleparams_t, torqueFactor ),	FillFloatProp },
	{ "BrakeFactor",	KVSCHEMA_DESC( vehicle_axleparams_t, brakeFactor ),		FillFloatProp },
};

static const JoltKVSchemaProp_t kVehicleBodyDescs[] =
{
	{ "MassCenterOverride",		KVSCHEMA_DESC( vehicle_bodyparams_t, massCenterOverride ),	FillVectorProp },
	{ "AddGravity",				KVSCHEMA_DESC( vehicle_bodyparams_t, addGravity ),			FillFloatProp },
	{ "MaxAngularVelocity",		KVSCHEMA_DESC( vehicle_bodyparams_t, maxAngularVelocity ),	FillFloatProp },
	{ "MassOverride",			KVSCHEMA_DESC( vehicle_bodyparams_t, massOverride ),		FillFloatProp },
	{ "TiltForce",				KVSCHEMA_DESC( vehicle_bodyparams_t, tiltForce ),			FillFloatProp },
	{ "TiltForceHeight",		KVSCHEMA_DESC( vehicle_bodyparams_t, tiltForceHeight ),		FillFloatProp },
	{ "CounterTorqueFactor",	KVSCHEMA_DESC( vehicle_bodyparams_t, counterTorqueFactor ),	FillFloatProp },
	{ "KeepUprightTorque",		KVSCHEMA_DESC( vehicle_bodyparams_t, keepUprightTorque ),	FillFloatProp },
};

static const JoltKVSchemaProp_t kVehicleEngineBoostDescs[] =
{
	{ "Force",			KVSCHEMA_DESC( vehicle_engineparams_t, boostForce ),	FillFloatProp },
	{ "Duration",		KVSCHEMA_DESC( vehicle_engineparams_t, boostDuration ),	FillFloatProp },
	{ "Delay",			KVSCHEMA_DESC( vehicle_engineparams_t, boostDelay ),	FillFloatProp },
	{ "MaxSpeed",		KVSCHEMA_DESC( vehicle_engineparams_t, boostMaxSpeed ),	FillFloatProp },
	{ "TorqueBoost",	KVSCHEMA_DESC( vehicle_engineparams_t, torqueBoost ),	FillBoolProp },
};

static const JoltKVSchemaProp_t kVehicleEngineDescs[] =
{
	{ "Boost",					KVSCHEMA_DESC_NO_OFFSET( vehicle_engineparams_t ),
		{
			sizeof( vehicle_engineparams_t ),
			[]( KeyValues *pProp, void *pPtr, size_t size )
			{
				vehicle_engineparams_t *pEngineParams = reinterpret_cast< vehicle_engineparams_t * >( pPtr );
				ParseJoltKVSchema( pProp, kVehicleEngineBoostDescs, ARRAYSIZE( kVehicleEngineBoostDescs ), pEngineParams );
			}
		}
	},
	{ "Gear",					KVSCHEMA_DESC_ARRAY( vehicle_engineparams_t, gearRatio, gearCount ),	FillFloatProp },
	{ "Horsepower",				KVSCHEMA_DESC( vehicle_engineparams_t, horsepower ),					FillFloatProp },
	{ "MaxSpeed",				KVSCHEMA_DESC( vehicle_engineparams_t, maxSpeed ),						FillFloatProp },
	{ "MaxReverseSpeed",		KVSCHEMA_DESC( vehicle_engineparams_t, maxRevSpeed ),					FillFloatProp },
	{ "AxleRatio",				KVSCHEMA_DESC( vehicle_engineparams_t, axleRatio ),						FillFloatProp },
	{ "MaxRPM",					KVSCHEMA_DESC( vehicle_engineparams_t, maxRPM ),						FillFloatProp },
	{ "ThrottleTime",			KVSCHEMA_DESC( vehicle_engineparams_t, throttleTime ),					FillFloatProp },
	{ "AutoTransmission",		KVSCHEMA_DESC( vehicle_engineparams_t, isAutoTransmission ),			FillBoolProp },
	{ "ShiftUpRPM",				KVSCHEMA_DESC( vehicle_engineparams_t, shiftUpRPM ),					FillFloatProp },
	{ "ShiftDownRPM",			KVSCHEMA_DESC( vehicle_engineparams_t, shiftDownRPM ),					FillFloatProp },
	{ "AutobrakeSpeedGain",		KVSCHEMA_DESC( vehicle_engineparams_t, autobrakeSpeedGain ),			FillFloatProp },
	{ "AutobrakeSpeedFactor",	KVSCHEMA_DESC( vehicle_engineparams_t, autobrakeSpeedFactor ),			FillFloatProp },
};

static const JoltKVSchemaProp_t kVehicleSteeringDescs[] =
{
	{ "DegreesSlow",					KVSCHEMA_DESC( vehicle_steeringparams_t, degreesSlow ),						FillFloatProp },
	{ "DegreesFast",					KVSCHEMA_DESC( vehicle_steeringparams_t, degreesFast ),						FillFloatProp },
	{ "DegreesBoost",					KVSCHEMA_DESC( vehicle_steeringparams_t, degreesBoost ),					FillFloatProp },

	{ "SlowCarSpeed",					KVSCHEMA_DESC( vehicle_steeringparams_t, speedSlow ),						FillFloatProp },
	{ "FastCarSpeed",					KVSCHEMA_DESC( vehicle_steeringparams_t, speedFast ),						FillFloatProp },

	{ "SlowSteeringRate",				KVSCHEMA_DESC( vehicle_steeringparams_t, steeringRateSlow ),				FillFloatProp },
	{ "FastSteeringRate",				KVSCHEMA_DESC( vehicle_steeringparams_t, steeringRateFast ),				FillFloatProp },

	{ "SteeringRestRateSlow",			KVSCHEMA_DESC( vehicle_steeringparams_t, steeringRestRateSlow ),			FillFloatProp },
	{ "SteeringRestRateFast",			KVSCHEMA_DESC( vehicle_steeringparams_t, steeringRestRateFast ),			FillFloatProp },

	{ "ThrottleSteeringRestRateFactor",	KVSCHEMA_DESC( vehicle_steeringparams_t, throttleSteeringRestRateFactor ),	FillFloatProp },
	{ "BoostSteeringRestRateFactor",	KVSCHEMA_DESC( vehicle_steeringparams_t, boostSteeringRestRateFactor ),		FillFloatProp },
	{ "BoostSteeringRateFactor",		KVSCHEMA_DESC( vehicle_steeringparams_t, boostSteeringRateFactor ),			FillFloatProp },

	{ "SteeringExponent",				KVSCHEMA_DESC( vehicle_steeringparams_t, steeringExponent ),				FillFloatProp },
	{ "TurnThrottleReduceSlow",			KVSCHEMA_DESC( vehicle_steeringparams_t, turnThrottleReduceSlow ),			FillFloatProp },
	{ "TurnThrottleReduceFast",			KVSCHEMA_DESC( vehicle_steeringparams_t, turnThrottleReduceFast ),			FillFloatProp },

	{ "BrakeSteeringRateFactor",		KVSCHEMA_DESC( vehicle_steeringparams_t, brakeSteeringRateFactor ),			FillFloatProp },
	{ "PowerSlideAccel",				KVSCHEMA_DESC( vehicle_steeringparams_t, powerSlideAccel ),					FillFloatProp },

	{ "SkidAllowed",					KVSCHEMA_DESC( vehicle_steeringparams_t, isSkidAllowed ),					FillBoolProp },
	{ "DustCloud",						KVSCHEMA_DESC( vehicle_steeringparams_t, dustCloud ),						FillBoolProp },
};

static const JoltKVSchemaProp_t kVehicleDescs[] =
{
	{ "Axle",			KVSCHEMA_DESC_ARRAY( vehicleparams_t, axles, axleCount ),
		{
			sizeof( vehicle_axleparams_t ),
			[]( KeyValues *pProp, void *pPtr, size_t size )
			{
				vehicle_axleparams_t *pAxleParams = reinterpret_cast< vehicle_axleparams_t * >( pPtr );
				ParseJoltKVSchema( pProp, kVehicleAxleDescs, ARRAYSIZE( kVehicleAxleDescs ), pAxleParams );
			}
		}
	},
	{ "Body",			KVSCHEMA_DESC( vehicleparams_t, body ),
		{
			sizeof( vehicle_bodyparams_t ),
			[]( KeyValues *pProp, void *pPtr, size_t size )
			{
				vehicle_bodyparams_t *pBodyParams = reinterpret_cast< vehicle_bodyparams_t * >( pPtr );
				ParseJoltKVSchema( pProp, kVehicleBodyDescs, ARRAYSIZE( kVehicleBodyDescs ), pBodyParams );
			}
		}
	},
	{ "Engine",			KVSCHEMA_DESC( vehicleparams_t, engine ),
		{
			sizeof( vehicle_engineparams_t ),
			[]( KeyValues *pProp, void *pPtr, size_t size )
			{
				vehicle_engineparams_t *pEngineParams = reinterpret_cast< vehicle_engineparams_t * >( pPtr );
				ParseJoltKVSchema( pProp, kVehicleEngineDescs, ARRAYSIZE( kVehicleEngineDescs ), pEngineParams );
			}
		}
	},
	{ "Steering",		KVSCHEMA_DESC( vehicleparams_t, steering ),
		{
			sizeof( vehicle_steeringparams_t ),
			[]( KeyValues *pProp, void *pPtr, size_t size )
			{
				vehicle_steeringparams_t *pSteeringParams = reinterpret_cast< vehicle_steeringparams_t * >( pPtr );
				ParseJoltKVSchema( pProp, kVehicleSteeringDescs, ARRAYSIZE( kVehicleSteeringDescs ), pSteeringParams );
			}
		}
	},
	{ "WheelsPerAxle",	KVSCHEMA_DESC( vehicleparams_t, wheelsPerAxle ),	FillIntProp },
};

static const JoltKVSchemaProp_t kCollisionRulesDescs[] =
{
	{ "SelfCollisions",			KVSCHEMA_DESC( JoltPhysicsCollisionRulesHelper, Rules.bSelfCollisions ),	FillIntProp },
	{ "CollisionPair",			KVSCHEMA_DESC( JoltPhysicsCollisionRulesHelper, CollisionPair ),			FillIntPairProp,
		[]( void *pBaseObject )
		{
			// Josh:
			// Now that it's been parsed, set it on the collision set.
			JoltPhysicsCollisionRulesHelper* pHelper = reinterpret_cast<JoltPhysicsCollisionRulesHelper*>( pBaseObject );
			if ( pHelper->Rules.bSelfCollisions )
				pHelper->Rules.pCollisionSet->EnableCollisions( pHelper->CollisionPair.Index0, pHelper->CollisionPair.Index1 );
		}
	},
};

static const JoltKVSchemaProp_t kRagdollAnimatedFrictionDescs[] =
{
	{ "AnimFrictionMin",		KVSCHEMA_DESC( ragdollanimatedfriction_t, minFriction ),	FillFloatProp },
	{ "AnimFrictionMax",		KVSCHEMA_DESC( ragdollanimatedfriction_t, maxFriction ),	FillFloatProp },
	{ "AnimFrictionTimeIn",		KVSCHEMA_DESC( ragdollanimatedfriction_t, timeIn ),			FillFloatProp },
	{ "AnimFrictionTimeOut",	KVSCHEMA_DESC( ragdollanimatedfriction_t, timeOut ),		FillFloatProp },
	{ "AnimFrictionTimeHold",	KVSCHEMA_DESC( ragdollanimatedfriction_t, timeHold ),		FillFloatProp },
};

//-------------------------------------------------------------------------------------------------

JoltPhysicsParseKV::JoltPhysicsParseKV( KeyValues *pKV )
	: m_pKV( pKV )
	, m_pCurrentBlock( m_pKV->GetFirstSubKey() )
{
}

JoltPhysicsParseKV::~JoltPhysicsParseKV()
{
}

//-------------------------------------------------------------------------------------------------

const char* JoltPhysicsParseKV::GetCurrentBlockName()
{
	if ( !m_pCurrentBlock )
		return nullptr;

	return m_pCurrentBlock->GetName();
}

bool		JoltPhysicsParseKV::Finished()
{
	return !m_pCurrentBlock;
}

void		JoltPhysicsParseKV::ParseSolid( solid_t *pSolid, IVPhysicsKeyHandler *unknownKeyHandler )
{
	if ( unknownKeyHandler )
		unknownKeyHandler->SetDefaults( pSolid );
	else
		V_memset( pSolid, 0, sizeof( *pSolid ) );

	ParseJoltKVSchema( m_pCurrentBlock, kSolidDescs, ARRAYSIZE( kSolidDescs ), pSolid, pSolid, unknownKeyHandler );

	NextBlock();
}

void		JoltPhysicsParseKV::ParseFluid( fluid_t *pFluid, IVPhysicsKeyHandler *unknownKeyHandler )
{
	if ( unknownKeyHandler )
		unknownKeyHandler->SetDefaults( pFluid );
	else
	{
		V_memset( pFluid, 0, sizeof( *pFluid ) );
		V_strncpy( pFluid->surfaceprop, "water", sizeof( pFluid->surfaceprop ) );
	}

	ParseJoltKVSchema( m_pCurrentBlock, kFluidDescs, ARRAYSIZE( kFluidDescs ), pFluid, pFluid, unknownKeyHandler );

	NextBlock();
}

void		JoltPhysicsParseKV::ParseRagdollConstraint( constraint_ragdollparams_t *pConstraint, IVPhysicsKeyHandler *unknownKeyHandler )
{
	if ( unknownKeyHandler )
		unknownKeyHandler->SetDefaults( pConstraint );
	else
	{
		V_memset( pConstraint, 0, sizeof( *pConstraint ) );
		pConstraint->childIndex = -1;
		pConstraint->parentIndex = -1;
	}

	// Josh: The KV specifies clockwise rotations.  
	pConstraint->useClockwiseRotations = true;

	ParseJoltKVSchema( m_pCurrentBlock, kRagdollDescs, ARRAYSIZE( kRagdollDescs ), pConstraint, pConstraint, unknownKeyHandler );

	NextBlock();
}

void		JoltPhysicsParseKV::ParseSurfaceTable( int *table, IVPhysicsKeyHandler *unknownKeyHandler )
{
	for ( KeyValues* pProp = m_pCurrentBlock->GetFirstSubKey(); pProp != nullptr; pProp = pProp->GetNextKey() )
	{
		int nPropIdx  = JoltPhysicsSurfaceProps::GetInstance().GetSurfaceIndex( pProp->GetName() );
		int nTableIdx = pProp->GetInt();

		if ( nTableIdx < 128 )
			table[nTableIdx] = nPropIdx;
	}

	NextBlock();
}

void		JoltPhysicsParseKV::ParseCustom( void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler )
{
	if ( unknownKeyHandler )
		unknownKeyHandler->SetDefaults( pCustom );

	ParseJoltKVCustom( m_pCurrentBlock, pCustom, unknownKeyHandler );

	NextBlock();
}

void		JoltPhysicsParseKV::ParseVehicle( vehicleparams_t *pVehicle, IVPhysicsKeyHandler *unknownKeyHandler )
{
	if ( unknownKeyHandler )
		unknownKeyHandler->SetDefaults( pVehicle );
	else
		V_memset( pVehicle, 0, sizeof( *pVehicle ) );

	ParseJoltKVSchema( m_pCurrentBlock, kVehicleDescs, ARRAYSIZE( kVehicleDescs ), pVehicle, pVehicle, unknownKeyHandler);

	NextBlock();
}

void		JoltPhysicsParseKV::SkipBlock()
{
	NextBlock();
}

void		JoltPhysicsParseKV::ParseCollisionRules( ragdollcollisionrules_t *pRules, IVPhysicsKeyHandler *unknownKeyHandler )
{
	if ( unknownKeyHandler )
		unknownKeyHandler->SetDefaults( pRules );

	JoltPhysicsCollisionRulesHelper helper =
	{
		.Rules				= pRules ? *pRules : ragdollcollisionrules_t{},
		.pUnknownKeyHandler = unknownKeyHandler,
	};

	ParseJoltKVSchema( m_pCurrentBlock, kCollisionRulesDescs, ARRAYSIZE( kCollisionRulesDescs ), &helper, pRules, unknownKeyHandler );

	if ( pRules )
		*pRules = helper.Rules;

	NextBlock();
}

void		JoltPhysicsParseKV::ParseRagdollAnimatedFriction( ragdollanimatedfriction_t *pFriction, IVPhysicsKeyHandler *unknownKeyHandler )
{
	if ( unknownKeyHandler )
		unknownKeyHandler->SetDefaults( pFriction );
	else
		V_memset( pFriction, 0, sizeof( *pFriction ) );

	ParseJoltKVSchema( m_pCurrentBlock, kRagdollAnimatedFrictionDescs, ARRAYSIZE( kRagdollAnimatedFrictionDescs ), pFriction, pFriction, unknownKeyHandler );

	NextBlock();
}

//-------------------------------------------------------------------------------------------------

void		JoltPhysicsParseKV::NextBlock()
{
	if ( m_pCurrentBlock )
		m_pCurrentBlock = m_pCurrentBlock->GetNextKey();
}

//-------------------------------------------------------------------------------------------------

static constexpr const char* DummyParserKeyValues = R"(
"PhysProps_Fallback"
{
	"solid"
	{
		"dummy" "1"
	}
	"vehicle"
	{
		"dummy" "1"
	}
	"vehicle_sounds"
	{
		"dummy" "1"
	}
	"vehicle_view"
	{
		"dummy" "1"
	}
	"ragdollconstraint"
	{
		"dummy" "1"
	}
	"collisionrules"
	{
		"dummy" "1"
	}
}
)";

IVPhysicsKeyParser *CreateVPhysicsKeyParser( const char *pKeyData, bool bIsPacked )
{
	VJoltAssertMsg( !bIsPacked, "Packed VPhysics KV not supported. You should not get here anyway as we do not emit it." );
	if ( bIsPacked )
		return nullptr;

	KeyValues *pszKV = HeaderlessKVBufferToKeyValues( pKeyData, "VPhysicsKeyParse" );

	// Josh: Ideally we would return nullptr here, but that breaks a lot of things.
	// If we fail to parse the KV, simply just fall-back to a dummy KV that will cause things
	// to get zero-initialized.
	// In the future, we may want to add a KV patching pass to fix up broken model and vehicle data.
	if ( !pszKV )
	{
		Log_Warning( LOG_VJolt, "CreateVPhysicsKeyParser: Encountered invalid KV data. Falling back to a dummy KV. You may notice a broken prop/vehicle.\n" );

		pszKV = new KeyValues( "VPhysicsKeyParse_Fallback" );
		pszKV->LoadFromBuffer( "VPhysicsKeyParse_Fallback", DummyParserKeyValues );
	}

	return new JoltPhysicsParseKV( pszKV );
}

void DestroyVPhysicsKeyParser( IVPhysicsKeyParser *pParser )
{
	JoltPhysicsParseKV *pJoltParser = static_cast< JoltPhysicsParseKV * >( pParser );
	delete pJoltParser;
}
