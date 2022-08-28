
#pragma once

#include "vjolt_internal_listeners.h"

class JoltPhysicsFluidController final : public IPhysicsFluidController, public IJoltObjectDestroyedListener, public IJoltPhysicsController
{
public:
	JoltPhysicsFluidController( JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsObject *pFluidObject, const fluidparams_t *pParams );
	~JoltPhysicsFluidController() override;

	void	SetGameData( void *pGameData ) override;
	void *	GetGameData() const override;

	void	GetSurfacePlane( Vector *pNormal, float *pDist ) const override;
	float	GetDensity() const override;
	void	WakeAllSleepingObjects() override;
	int		GetContents() const override;

public:

	// IJoltObjectDestroyedListener
	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) override;
	// IJoltPhysicsController
	void OnPreSimulate( float flDeltaTime ) override;

private:

	cplane_t GetSurfacePlane() const;
	void ClearCachedObjectsInShape();

	JPH::PhysicsSystem *				m_pPhysicsSystem;
	JoltPhysicsObject *					m_pFluidObject;
	std::vector<JoltPhysicsObject *>	m_ObjectsInShape;

	fluidparams_t						m_Params;
	cplane_t							m_LocalPlane;
};
