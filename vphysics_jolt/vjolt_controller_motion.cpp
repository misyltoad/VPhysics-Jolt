
#include "cbase.h"

#include "vjolt_controller_motion.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

JoltPhysicsMotionController::JoltPhysicsMotionController( IMotionEvent *pHandler )
	: m_pMotionEvent( pHandler )
{
}

JoltPhysicsMotionController::~JoltPhysicsMotionController()
{
	for ( JoltPhysicsObject *pObject : m_pObjects )
		pObject->RemoveDestroyedListener( this );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsMotionController::SetEventHandler( IMotionEvent *pHandler )
{
	m_pMotionEvent = pHandler;
}

void JoltPhysicsMotionController::AttachObject( IPhysicsObject *pObject, bool bCheckIfAlreadyAttached )
{
	if ( !pObject || pObject->IsStatic() )
		return;

	JoltPhysicsObject *pPhysicsObject = static_cast< JoltPhysicsObject * >( pObject );
	if ( bCheckIfAlreadyAttached && VectorContains( m_pObjects, pPhysicsObject ) )
		return;

	pPhysicsObject->AddDestroyedListener( this );
	m_pObjects.push_back( pPhysicsObject );
}

void JoltPhysicsMotionController::DetachObject( IPhysicsObject *pObject )
{
	if ( !pObject )
		return;

	JoltPhysicsObject *pPhysicsObject = static_cast< JoltPhysicsObject * >( pObject );
	Erase( m_pObjects, pPhysicsObject );
	pPhysicsObject->RemoveDestroyedListener( this );
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsMotionController::CountObjects( void )
{
	return int( m_pObjects.size() );
}

void JoltPhysicsMotionController::GetObjects( IPhysicsObject **pObjectList )
{
	for ( size_t i = 0; i < m_pObjects.size(); i++ )
		pObjectList[ i ] = m_pObjects[ i ];
}

void JoltPhysicsMotionController::ClearObjects( void )
{
	m_pObjects.clear();
}

void JoltPhysicsMotionController::WakeObjects( void )
{
	for ( JoltPhysicsObject *pObject : m_pObjects )
		pObject->Wake();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsMotionController::SetPriority( priority_t priority )
{
	// Not relevant to us.
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsMotionController::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	JoltPhysicsObject *pPhysicsObject = static_cast< JoltPhysicsObject * >( pObject );
	Erase( m_pObjects, pPhysicsObject );
}

void JoltPhysicsMotionController::OnPreSimulate( float flDeltaTime )
{
	if ( !m_pMotionEvent )
		return;

	for ( JoltPhysicsObject *pObject : m_pObjects )
	{
		if ( !pObject->IsMoveable() )
			return;

		Vector vecVelocity = vec3_origin;
		Vector vecLocalAngularVelocity = vec3_origin;
		IMotionEvent::simresult_e simResult = m_pMotionEvent->Simulate( this, pObject, flDeltaTime, vecVelocity, vecLocalAngularVelocity );

		vecVelocity *= flDeltaTime;
		vecLocalAngularVelocity *= flDeltaTime;

		// GLOBAL/LOCAL refer to the coordinate system of vecVelocity's values.
		// ACCELERATION/FORCE determine whether or not mass is divided out.
		// 
		// vecLocalAngularVelocity from Simulate is always in given local space, regardless.
		// We must convert it to global space for it to be applied correctly.

		QAngle qObjectAngles;
		pObject->GetPosition( nullptr, &qObjectAngles );

		// vecLocalAngularVelocity is always local space.
		Vector vecWorldAngularVelocity;
		pObject->LocalToWorldVector( &vecWorldAngularVelocity, vecLocalAngularVelocity );

		Vector vecWorldVelocity = vecVelocity;
		if ( simResult == IMotionEvent::SIM_LOCAL_ACCELERATION || simResult == IMotionEvent::SIM_LOCAL_FORCE )
		{
			pObject->LocalToWorldVector( &vecWorldVelocity, vecVelocity );
		}

		switch ( simResult )
		{
			case IMotionEvent::SIM_NOTHING:
			{
				break;
			}

			case IMotionEvent::SIM_GLOBAL_ACCELERATION:
			case IMotionEvent::SIM_LOCAL_ACCELERATION:
			{
				pObject->AddVelocity( &vecWorldVelocity, &vecWorldAngularVelocity );
				break;
			}

			case IMotionEvent::SIM_GLOBAL_FORCE:
			case IMotionEvent::SIM_LOCAL_FORCE:
			{
				pObject->ApplyForceCenter( vecWorldVelocity );
				pObject->ApplyTorqueCenter( vecWorldAngularVelocity );
				break;
			}

			default:
			{
				Log_Warning( LOG_VJolt, "Invalid motion event\n" );
				break;
			}
		}
	}
}
