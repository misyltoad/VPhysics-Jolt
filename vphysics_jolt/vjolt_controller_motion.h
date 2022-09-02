
#pragma once

#include "vjolt_object.h"
#include "vjolt_environment.h"

class JoltPhysicsMotionController : public IPhysicsMotionController, public IJoltObjectDestroyedListener, public IJoltPhysicsController
{
public:
	JoltPhysicsMotionController( IMotionEvent *pHandler );
	~JoltPhysicsMotionController() override;

	void SetEventHandler( IMotionEvent *handler ) override;
	void AttachObject( IPhysicsObject *pObject, bool checkIfAlreadyAttached ) override;
	void DetachObject( IPhysicsObject *pObject ) override;

	int CountObjects( void ) override;
	void GetObjects( IPhysicsObject **pObjectList ) override;
	void ClearObjects( void ) override;
	void WakeObjects( void ) override;

	void SetPriority( priority_t priority ) override;

public:

	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) override;
	void OnPreSimulate( float flDeltaTime ) override;

private:
	IMotionEvent *m_pMotionEvent;

	std::vector< JoltPhysicsObject * > m_pObjects;
};
