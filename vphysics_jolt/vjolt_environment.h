//=================================================================================================
//
// Interface to a physics scene
//
//=================================================================================================

#pragma once

#include "vjolt_interface.h"
#include "vjolt_object.h"
#include "vjolt_constraints.h"
#include "vjolt_listener_contact.h"

class JoltBroadPhaseLayerInterface;
class JoltObjectVsBroadPhaseLayerFilter;
class JoltObjectLayerPairFilter;

// StateRecorder implementation that saves to a fixed buffer
class VJoltStateRecorder final : public JPH::StateRecorder, public CUtlBuffer
{
public:
	using CUtlBuffer::CUtlBuffer;

	// StreamIn
	void ReadBytes( void* outData, size_t inNumBytes ) override
	{
		Get( outData, static_cast<int>( inNumBytes ) );
	}

	bool IsEOF() const override { return false; }

	// StreamOut
	void WriteBytes( const void* inData, size_t inNumBytes )
	{
		Put( inData, static_cast<int>( inNumBytes ) );
	}

	// Both
	bool IsFailed() const override { return false; }
};

class JoltPhysicsEnvironment final : public IPhysicsEnvironment
{
public:
	JoltPhysicsEnvironment();
	~JoltPhysicsEnvironment() override;

	void SetDebugOverlay( CreateInterfaceFn debugOverlayFactory ) override;
	IVPhysicsDebugOverlay* GetDebugOverlay( void ) override;

	void SetGravity( const Vector& gravityVector ) override;
	void GetGravity( Vector* pGravityVector ) const override;

	void SetAirDensity( float density ) override;
	float GetAirDensity() const override;

	IPhysicsObject* CreatePolyObject( const CPhysCollide* pCollisionModel, int materialIndex, const Vector& position, const QAngle& angles, objectparams_t* pParams ) override;
	IPhysicsObject* CreatePolyObjectStatic( const CPhysCollide* pCollisionModel, int materialIndex, const Vector& position, const QAngle& angles, objectparams_t* pParams ) override;
	IPhysicsObject* CreateSphereObject( float radius, int materialIndex, const Vector& position, const QAngle& angles, objectparams_t* pParams, bool isStatic ) override;
	void DestroyObject( IPhysicsObject* ) override;

	IPhysicsFluidController* CreateFluidController( IPhysicsObject* pFluidObject, fluidparams_t* pParams ) override;
	void DestroyFluidController( IPhysicsFluidController* ) override;

	IPhysicsSpring* CreateSpring( IPhysicsObject* pObjectStart, IPhysicsObject* pObjectEnd, springparams_t* pParams ) override;
	void DestroySpring( IPhysicsSpring* ) override;

	IPhysicsConstraint* CreateRagdollConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_ragdollparams_t& ragdoll ) override;
	IPhysicsConstraint* CreateHingeConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_hingeparams_t& hinge ) override;
	IPhysicsConstraint* CreateFixedConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_fixedparams_t& fixed ) override;
	IPhysicsConstraint* CreateSlidingConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_slidingparams_t& sliding ) override;
	IPhysicsConstraint* CreateBallsocketConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_ballsocketparams_t& ballsocket ) override;
	IPhysicsConstraint* CreatePulleyConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_pulleyparams_t& pulley ) override;
	IPhysicsConstraint* CreateLengthConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_lengthparams_t& length ) override;

	void DestroyConstraint( IPhysicsConstraint* ) override;

	IPhysicsConstraintGroup* CreateConstraintGroup( const constraint_groupparams_t& groupParams ) override;
	void DestroyConstraintGroup( IPhysicsConstraintGroup* pGroup ) override;

	IPhysicsShadowController* CreateShadowController( IPhysicsObject* pObject, bool allowTranslation, bool allowRotation ) override;
	void DestroyShadowController( IPhysicsShadowController* ) override;

	IPhysicsPlayerController* CreatePlayerController( IPhysicsObject* pObject ) override;
	void DestroyPlayerController( IPhysicsPlayerController* ) override;

	IPhysicsMotionController* CreateMotionController( IMotionEvent* pHandler ) override;
	void DestroyMotionController( IPhysicsMotionController* pController ) override;

	IPhysicsVehicleController* CreateVehicleController( IPhysicsObject* pVehicleBodyObject, const vehicleparams_t& params, unsigned int nVehicleType, IPhysicsGameTrace* pGameTrace ) override;
	void DestroyVehicleController( IPhysicsVehicleController* ) override;

	void SetCollisionSolver( IPhysicsCollisionSolver* pSolver ) override;

	void Simulate( float deltaTime ) override;
	bool IsInSimulation() const override;

	float GetSimulationTimestep() const override;
	void SetSimulationTimestep( float timestep ) override;

	float GetSimulationTime() const override;
	void ResetSimulationClock() override;
	float GetNextFrameTime() const override;

	void SetCollisionEventHandler( IPhysicsCollisionEvent* pCollisionEvents ) override;
	void SetObjectEventHandler( IPhysicsObjectEvent* pObjectEvents ) override;
	virtual void SetConstraintEventHandler( IPhysicsConstraintEvent* pConstraintEvents ) override;

	void SetQuickDelete( bool bQuick ) override;

	int GetActiveObjectCount() const override;
	void GetActiveObjects( IPhysicsObject** pOutputObjectList ) const override;
	const IPhysicsObject** GetObjectList( int* pOutputObjectCount ) const override;
	bool TransferObject( IPhysicsObject* pObject, IPhysicsEnvironment* pDestinationEnvironment ) override;

	void CleanupDeleteList() override;
	void EnableDeleteQueue( bool enable ) override;

	bool Save( const physsaveparams_t& params ) override;
	void PreRestore( const physprerestoreparams_t& params ) override;
	bool Restore( const physrestoreparams_t& params ) override;
	void PostRestore() override;

	bool IsCollisionModelUsed( CPhysCollide* pCollide ) const override;

	void TraceRay( const Ray_t& ray, unsigned int fMask, IPhysicsTraceFilter* pTraceFilter, trace_t* pTrace ) override;
	void SweepCollideable( const CPhysCollide* pCollide, const Vector& vecAbsStart, const Vector& vecAbsEnd,
		const QAngle& vecAngles, unsigned int fMask, IPhysicsTraceFilter* pTraceFilter, trace_t* pTrace ) override;

	void GetPerformanceSettings( physics_performanceparams_t* pOutput ) const override;
	void SetPerformanceSettings( const physics_performanceparams_t* pSettings ) override;

	void ReadStats( physics_stats_t* pOutput ) override;
	void ClearStats() override;

	unsigned int GetObjectSerializeSize( IPhysicsObject* pObject ) const override;
	void SerializeObjectToBuffer( IPhysicsObject* pObject, unsigned char* pBuffer, unsigned int bufferSize ) override;
	IPhysicsObject* UnserializeObjectFromBuffer( void* pGameData, unsigned char* pBuffer, unsigned int bufferSize, bool enableCollisions ) override;

	void EnableConstraintNotify( bool bEnable ) override;
	void DebugCheckContacts() override;

	void SetAlternateGravity( const Vector& gravityVector ) override_asw;
	void GetAlternateGravity( Vector* pGravityVector ) const override_asw;

	float GetDeltaFrameTime( int maxTicks ) const override_asw;
	void ForceObjectsToSleep( IPhysicsObject** pList, int listCount ) override_asw;

	void SetPredicted( bool bPredicted ) override_portal2;
	bool IsPredicted() override_portal2;
	void SetPredictionCommandNum( int iCommandNum ) override_portal2;
	int GetPredictionCommandNum() override_portal2;
	void DoneReferencingPreviousCommands( int iCommandNum ) override_portal2;
	void RestorePredictedSimulation() override_portal2;

	void DestroyCollideOnDeadObjectFlush( CPhysCollide* ) override_portal2;

public:
	JPH::PhysicsSystem* GetPhysicsSystem() { return &m_PhysicsSystem; }

	void ObjectTransferHandOver( JoltPhysicsObject* pObject );

	JoltPhysicsContactListener* GetContactListener() { return &m_ContactListener; }

	IPhysicsConstraintEvent* GetConstraintEvents() { return m_pConstraintListener; }

	void NotifyConstraintDisabled( JoltPhysicsConstraint* pConstraint );

	void AddDirtyStaticBody( const JPH::BodyID &id );
	void RemoveDirtyStaticBody( const JPH::BodyID &id );

private:

	void RemoveBodyAndDeleteObject( JoltPhysicsObject* pObject );
	void DeleteDeadObjects();

	template <typename T>
	void AddPhysicsSaveRestorePointer( uintp oldPtr, T* newPtr );

	template <typename T>
	T* LookupPhysicsSaveRestorePointer( uintp oldPtr );

	void HandleDebugDumpingEnvironment( void* pReturnAddress );

	bool m_bSimulating = false;
	bool m_bEnableDeleteQueue = false;
	bool m_bWakeObjectsOnConstraintDeletion = false;
	bool m_bOptimizedBroadPhase = false;
	bool m_bUseLinearCast = true;
	float m_flStepTime = 1.0f / 60.0f;
	float m_flAirDensity = 2.0f;

	static JoltBroadPhaseLayerInterface s_BroadPhaseLayerInterface;
	static JoltObjectVsBroadPhaseLayerFilter s_BroadPhaseFilter;
	static JoltObjectLayerPairFilter s_LayerPairFilter;

	// For GetObjectList
	mutable JPH::BodyIDVector m_CachedBodies;
	mutable std::vector< const IPhysicsObject * > m_CachedObjects;

	// For GetActiveObjectCount and GetActiveObjects
	mutable JPH::BodyIDVector m_CachedActiveBodies;

	JPH::PhysicsSystem m_PhysicsSystem;

	// A vector of objects that were awake, and changed their
	// motion type from Dynamic -> Static, so that they can be
	// retrieved in GetActiveObjects, and have their visuals updated.
	// If we don't do this, objects that get moved, woken, and their
	// movement type changed to static will not get their transforms
	// updated on the game side.
	mutable JPH::BodyIDVector m_DirtyStaticBodies;

	std::vector< JoltPhysicsObject * > m_pDeadObjects;
	std::vector< JoltPhysicsConstraint * > m_pDeadConstraints;
	std::vector< CPhysCollide * > m_pDeadObjectCollides;

	std::vector< IJoltPhysicsController * > m_pPhysicsControllers;

	std::unordered_map< uintp, void * > m_SaveRestorePointerMap;

	// The physics system that simulates the world
	// The debug overlay to render with (if it was ever passed to us)
	IVJoltDebugOverlay *m_pDebugOverlay = nullptr;

	JoltPhysicsContactListener m_ContactListener;
	IPhysicsConstraintEvent *m_pConstraintListener = nullptr;

	bool m_EnableConstraintNotify = false;

	mutable bool m_bActiveObjectCountFirst = true;

	physics_performanceparams_t m_PerformanceParams;
};
