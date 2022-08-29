
#pragma once

#include "vjolt_object.h" // IJoltObjectDestroyedListener
#include "vjolt_environment.h" // IJoltPhysicsController

struct JoltPhysicsWheel
{
	JoltPhysicsObject* pObject = nullptr;
	bool InWater = false;
	float Depth = 0.0f;
};

struct JoltPhysicsInternalVehicleState
{
	bool  EngineDisabled = false;
	float BoostDelay = 0.0f;
	float BoosterRemainingTime = 0.0f;
	float LargestWheelRadius = 0.0f;
};

class JoltPhysicsVehicleController final : public IPhysicsVehicleController, public IJoltObjectDestroyedListener, public IJoltPhysicsController
{
public:
	static constexpr int MaxWheels = VEHICLE_MAX_WHEEL_COUNT;

	JoltPhysicsVehicleController( JoltPhysicsEnvironment *pEnvironment, JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace );
	~JoltPhysicsVehicleController() override;

	void				Update( float dt, vehicle_controlparams_t &controls ) override;
	const vehicle_operatingparams_t & GetOperatingParams() override;
	const vehicleparams_t & GetVehicleParams() override;
	vehicleparams_t &	GetVehicleParamsForChange() override;
	float				UpdateBooster( float dt ) override;
	int					GetWheelCount( void ) override;
	IPhysicsObject *	GetWheel( int index ) override;
	bool				GetWheelContactPoint( int index, Vector *pContactPoint, int *pSurfaceProps ) override;
	void				SetSpringLength( int wheelIndex, float length ) override;
	void				SetWheelFriction( int wheelIndex, float friction ) override;

	void				OnVehicleEnter( void ) override;
	void				OnVehicleExit( void ) override;

	void				SetEngineDisabled( bool bDisable ) override;
	bool				IsEngineDisabled( void ) override;

	void				GetCarSystemDebugData( vehicle_debugcarsystem_t &debugCarSystem ) override;
	void				VehicleDataReload() override;

public:
	// IJoltObjectDestroyedListener
	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) override;

	float GetSpeed();

	// IJoltPhysicsController
	void OnPreSimulate( float flDeltaTime ) override;
	void OnPostSimulate( float flDeltaTime ) override;

private:

	void HandleBoostKey();
	void HandleBoostDecay();

	void CreateWheel( JPH::VehicleConstraintSettings &vehicleSettings, matrix3x4_t &bodyMatrix, int axleIdx, int wheelIdx );
	void CreateWheels( JPH::VehicleConstraintSettings& vehicleSettings );

	JPH::WheeledVehicleControllerSettings *CreateVehicleController();
	JPH::WheeledVehicleController *GetWheeledVehicleController();

	matrix3x4_t GetBodyMatrix() const;

	void DetachObject();

	JoltPhysicsEnvironment					*m_pEnvironment = nullptr;
	JPH::PhysicsSystem						*m_pPhysicsSystem = nullptr;
	JoltPhysicsObject						*m_pCarBodyObject = nullptr;
	vehicleparams_t							m_VehicleParams = {};
	unsigned int							m_VehicleType = 0u;

	vehicle_operatingparams_t				m_OperatingParams = {};
	vehicle_controlparams_t					m_ControlParams = {};

	std::vector< JoltPhysicsWheel >			m_Wheels;

	float									m_TotalWheelMass = 0.0f;
	JoltPhysicsInternalVehicleState			m_InternalState;

	JPH::Ref< JPH::VehicleConstraint >		m_VehicleConstraint;
	JPH::Ref< JPH::VehicleCollisionTester >	m_Tester;


};
