//=================================================================================================
//
// A physics object
//
//=================================================================================================

#pragma once

class IPredictedPhysicsObject;

class IJoltObjectDestroyedListener;
class JoltPhysicsShadowController;
class JoltPhysicsFluidController;
class JoltPhysicsEnvironment;
class JoltPhysicsObject;

#if defined( GAME_CSGO_OR_NEWER )
using IPhysicsObjectInterface = IPredictedPhysicsObject;
#else
using IPhysicsObjectInterface = IPhysicsObject;
#endif

class JoltPhysicsObject final : public IPhysicsObjectInterface
{
public:
	JoltPhysicsObject( JPH::Body *pBody, JoltPhysicsEnvironment *pEnvironment, bool bStatic, int nMaterialIndex, const objectparams_t *pParams );
	JoltPhysicsObject( JPH::Body *pBody, JoltPhysicsEnvironment *pEnvironment, void *pGameData, JPH::StateRecorder &recorder );
	~JoltPhysicsObject() override;

	bool			IsStatic() const override;
	bool			IsAsleep() const override;
	bool			IsTrigger() const override;
	bool			IsFluid() const override;
	bool			IsHinged() const override;
	bool			IsCollisionEnabled() const override;
	bool			IsGravityEnabled() const override;
	bool			IsDragEnabled() const override;
	bool			IsMotionEnabled() const override;
	bool			IsMoveable() const override; // legacy: IsMotionEnabled() && !IsStatic()
	bool			IsAttachedToConstraint( bool bExternalOnly ) const override;

	void			EnableCollisions( bool enable ) override;
	void			EnableGravity( bool enable ) override;
	void			EnableDrag( bool enable ) override;
	void			EnableMotion( bool enable ) override;

	void			SetGameData( void *pGameData ) override;
	void *			GetGameData() const override;
	void			SetGameFlags( unsigned short userFlags ) override;
	unsigned short	GetGameFlags() const override;
	void			SetGameIndex( unsigned short gameIndex ) override;
	unsigned short	GetGameIndex() const override;

	void			SetCallbackFlags( unsigned short callbackflags ) override;
	unsigned short	GetCallbackFlags() const override;

	void			Wake() override;
	void			Sleep() override;
	void			RecheckCollisionFilter() override;
	void			RecheckContactPoints( bool bSearchForNewContacts ) override_portal2;
	void			RecheckContactPoints() override_not_portal2 { RecheckContactPoints( false ); }

	void			SetMass( float mass ) override;
	float			GetMass() const override;
	float			GetInvMass() const override;
	Vector			GetInertia() const override;
	Vector			GetInvInertia() const override;
	void			SetInertia( const Vector &inertia ) override;

	void			SetDamping( const float *speed, const float *rot ) override;
	void			GetDamping( float *speed, float *rot ) const override;

	void			SetDragCoefficient( float *pDrag, float *pAngularDrag ) override;
	void			SetBuoyancyRatio( float ratio ) override;

	int				GetMaterialIndex() const override;
	void			SetMaterialIndex( int materialIndex ) override;

	unsigned int	GetContents() const override;
	void			SetContents( unsigned int contents ) override;

	float			GetSphereRadius() const override;
	void			SetSphereRadius( float radius ) override_asw;
	float			GetEnergy() const override;
	Vector			GetMassCenterLocalSpace() const override;

	void			SetPosition( const Vector &worldPosition, const QAngle &angles, bool isTeleport ) override;
	void			SetPositionMatrix( const matrix3x4_t &matrix, bool isTeleport ) override;

	void			GetPosition( Vector *worldPosition, QAngle *angles ) const override;
	void			GetPositionMatrix( matrix3x4_t *positionMatrix ) const override;
	void			SetVelocity( const Vector *velocity, const AngularImpulse *angularVelocity ) override;

	void			SetVelocityInstantaneous( const Vector *velocity, const AngularImpulse *angularVelocity ) override;

	void			GetVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const override;

	void			AddVelocity( const Vector *velocity, const AngularImpulse *angularVelocity ) override;
	void			GetVelocityAtPoint( const Vector &worldPosition, Vector *pVelocity ) const override;
	void			GetImplicitVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const override;
	void			LocalToWorld( Vector *worldPosition, const Vector &localPosition ) const override;
	void			WorldToLocal( Vector *localPosition, const Vector &worldPosition ) const override;

	void			LocalToWorldVector( Vector *worldVector, const Vector &localVector ) const override;
	void			WorldToLocalVector( Vector *localVector, const Vector &worldVector ) const override;

	void			ApplyForceCenter( const Vector &forceVector ) override;
	void			ApplyForceOffset( const Vector &forceVector, const Vector &worldPosition ) override;
	void			ApplyTorqueCenter( const AngularImpulse &torque ) override;

	void			CalculateForceOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerForce, AngularImpulse *centerTorque ) const override;
	void			CalculateVelocityOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerVelocity, AngularImpulse *centerAngularVelocity ) const override;
	float			CalculateLinearDrag( const Vector &unitDirection ) const override;
	float			CalculateAngularDrag( const Vector &objectSpaceRotationAxis ) const override;

	bool			GetContactPoint( Vector *contactPoint, IPhysicsObject **contactObject ) const override;

	void			SetShadow( float maxSpeed, float maxAngularSpeed, bool allowPhysicsMovement, bool allowPhysicsRotation ) override;
	void			UpdateShadow( const Vector &targetPosition, const QAngle &targetAngles, bool tempDisableGravity, float timeOffset ) override;

	int							GetShadowPosition( Vector *position, QAngle *angles ) const override;
	IPhysicsShadowController *	GetShadowController() const override;
	void						RemoveShadowController() override;
	float						ComputeShadowControl( const hlshadowcontrol_params_t &params, float secondsToArrival, float dt ) override;


	const CPhysCollide *	GetCollide() const override;
	const char *			GetName() const override;

	void			BecomeTrigger() override;
	void			RemoveTrigger() override;

	void			BecomeHinged( int localAxis ) override;
	void			RemoveHinged() override;

	IPhysicsFrictionSnapshot *CreateFrictionSnapshot() override;
	void DestroyFrictionSnapshot( IPhysicsFrictionSnapshot *pSnapshot ) override;

	void			OutputDebugInfo() const override;

#if OBJECT_WELDING
	void			WeldToObject( IPhysicsObject *pParent ) override;
	void			RemoveWeld( IPhysicsObject *pOther ) override;
	void			RemoveAllWelds() override;
#endif

	void			SetUseAlternateGravity( bool bSet ) override_asw;
	void			SetCollisionHints( uint32 collisionHints ) override_asw;
	uint32			GetCollisionHints() const override_asw;

	IPredictedPhysicsObject *	GetPredictedInterface() const override_csgo;
	void						SyncWith( IPhysicsObject *pOther ) override_csgo;

	void SetErrorDelta_Position( const Vector& vPosition ) override_csgo {}
	void SetErrorDelta_Velocity( const Vector& vVelocity ) override_csgo {}

public:
	JoltPhysicsEnvironment *GetEnvironment() { return m_pEnvironment; }

	JPH::BodyID GetBodyID() { return m_pBody->GetID(); }
	JPH::Body *GetBody() { return m_pBody; }

	void UpdateEnvironment( JoltPhysicsEnvironment *pEnvironment );

	void AddDestroyedListener( IJoltObjectDestroyedListener *pListener );
	void RemoveDestroyedListener( IJoltObjectDestroyedListener *pListener );

	// Grabs the position, adds addPos and teleports the object
	void AddToPosition( JPH::Vec3Arg addPos );

	// Only sets the position, and nothing else.
	void SetPosition( const Vector &worldPosition );

	// Adds to the velocity (Source space)
	void AddVelocity( const Vector &worldPosition );

	Vector GetVelocity();

	void CalculateBuoyancy();

	float GetMaterialDensity() const;
	float GetBuoyancyRatio() const;
	float GetVolume() const { return m_flVolume; }

	bool IsControlledByGame() const;

	void AddCallbackFlags( uint16 flags ) { m_callbackFlags |= flags; }
	void RemoveCallbackFlags( uint16 flags ) { m_callbackFlags &= ~flags; }

	void SaveObjectState( JPH::StateRecorder &recorder );
	void RestoreObjectState( JPH::StateRecorder &recorder );

	unsigned short GetGameMaterial() const
	{
		return m_GameMaterial;
	}

	bool GetGameMaterialAllowsSounds() const
	{
		return m_GameMaterial != 'X';
	}

	JoltPhysicsFluidController *GetFluidController()
	{
		return m_pFluidController;
	}

	void SetFluidController( JoltPhysicsFluidController *pFluidController )
	{
		m_pFluidController = pFluidController;
	}

	// Fakes a linear velocity so we can have correct before/after velocity
	// when going between PreCollision and PostCollision callbacks.
	JPH::Vec3 FakeJoltLinearVelocity( JPH::Vec3Arg fakeVelocity )
	{
		if ( m_pBody->IsStatic() )
			return JPH::Vec3::sZero();

		JPH::Vec3 oldVel = m_pBody->GetLinearVelocity();
		m_pBody->SetLinearVelocity( fakeVelocity );
		return oldVel;
	}

	void RestoreJoltLinearVelocity( JPH::Vec3Arg realVelocity )
	{
		if ( m_pBody->IsStatic() )
			return;

		m_pBody->SetLinearVelocity( realVelocity );
	}

private:
	void UpdateMaterialProperties();
	void UpdateLayer();

	// Josh:
	// Always put m_pGameData first. Some games that will
	// remain un-named offset by the vtable to get to this
	// instead of calling GetGameData().
	void *m_pGameData = nullptr;
	const char *m_pName = "NoName";

	uint16 m_gameFlags = 0;
	uint16 m_gameIndex = 0;
	uint16 m_callbackFlags = CALLBACK_GLOBAL_COLLISION|CALLBACK_GLOBAL_FRICTION|CALLBACK_FLUID_TOUCH|CALLBACK_GLOBAL_TOUCH|CALLBACK_GLOBAL_COLLIDE_STATIC|CALLBACK_DO_FLUID_SIMULATION;
	uint32 m_collisionHints = 0;

	bool m_bStatic = false;
	bool m_bPinned = false;

	int m_materialIndex = 0;
	uint m_contents = CONTENTS_SOLID;

	// Need this as Jolt gets very unhappy about reading motion
	// properties of static objects.
	float m_flCachedMass = 0.0f;
	float m_flCachedInvMass = 0.0f;
	bool m_bCachedCollisionEnabled = true;

	float m_flMaterialDensity = 1.0f; // Material density in Jolt space.
	float m_flBuoyancyRatio = 0.0f;
	float m_flVolume = 0.0f;

	unsigned short m_GameMaterial = 0;


	CUtlVector< IJoltObjectDestroyedListener * > m_destroyedListeners;

	// Shadow variables
	JoltPhysicsShadowController *m_pShadowController = nullptr;
	JoltPhysicsFluidController *m_pFluidController = nullptr;
	bool m_bShadowTemporarilyDisableGravity = false;

	JPH::Body *m_pBody = nullptr;						// Underlying Jolt body
	JoltPhysicsEnvironment *m_pEnvironment = nullptr;	// Physics environment this body belongs to
	JPH::PhysicsSystem *m_pPhysicsSystem = nullptr;		// Physics system this body belongs to
};

// Josh: This doesn't handle mass change and is kind of a hack and sliightly wrong.
// Would be nice to just specify spring constant directly in Jolt.
inline float GetInvEffectiveMass( JoltPhysicsObject *pObject0, JoltPhysicsObject *pObject1 )
{
	return ( pObject0->IsStatic() ? 0.0f : pObject0->GetInvMass() ) + ( pObject1->IsStatic() ? 0.0f : pObject1->GetInvMass() );
}

