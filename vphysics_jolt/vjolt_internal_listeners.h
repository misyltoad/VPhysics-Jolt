
#pragma once

class JoltPhysicsObject;

abstract_class IJoltPhysicsController
{
public:
	virtual ~IJoltPhysicsController() {}

	// Called before the simulation is run
	virtual void OnPreSimulate( float flDeltaTime ) {};
	// Called after the simulation is run
	virtual void OnPostSimulate( float flDeltaTime ) {};
};

abstract_class IJoltObjectDestroyedListener
{
public:
	virtual ~IJoltObjectDestroyedListener() {}

	// Called whenever a physics object is destroyed
	virtual void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) = 0;
};
