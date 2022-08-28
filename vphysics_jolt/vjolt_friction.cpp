
#include "cbase.h"

#include "vjolt_friction.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// Josh:
// Friction snapshots are used for the following:
//  - Making objects on NPCs phase through them after some time
//  - Something something physics damage, but that seems to work anyway... :think:
//  - Game code callback for freezing objects if too many objects (IVP performance backdoor) -- not relevant for us
//  - Disabling last portal stuff for objects after teleporting, but our trigger callbacks are robust so this game code hack doesn't seem to matter *touch wood*
//  - Something to do with changing paint powers
//  - Train physics blockers
// All in all, it doesn't seem to be *that* important, most things still work
// without it implemented.
// Right now, we do not have an efficient way to implement this with Jolt.
// Hence it is just stubby.

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsFrictionSnapshot::IsValid()
{
	return false;
}

//-------------------------------------------------------------------------------------------------

IPhysicsObject *JoltPhysicsFrictionSnapshot::GetObject( int index )
{
	return nullptr;
}

int JoltPhysicsFrictionSnapshot::GetMaterial( int index )
{
	return 0;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::GetContactPoint( Vector &out )
{
	out.Zero();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::GetSurfaceNormal( Vector &out )
{
	out.Zero();
}

float JoltPhysicsFrictionSnapshot::GetNormalForce()
{
	return 0.0f;
}

float JoltPhysicsFrictionSnapshot::GetEnergyAbsorbed()
{
	return 0.0f;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::RecomputeFriction()
{
	
}

void JoltPhysicsFrictionSnapshot::ClearFrictionForce()
{

}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::MarkContactForDelete()
{

}

void JoltPhysicsFrictionSnapshot::DeleteAllMarkedContacts( bool wakeObjects )
{

}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::NextFrictionData()
{

}

float JoltPhysicsFrictionSnapshot::GetFrictionCoefficient()
{
	return 0.0f;
}
