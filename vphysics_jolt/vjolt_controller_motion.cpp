
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

		Vector vecLocalVelocity = vec3_origin;
		Vector vecAngularVelocity = vec3_origin;
		IMotionEvent::simresult_e simResult = m_pMotionEvent->Simulate( this, pObject, flDeltaTime, vecLocalVelocity, vecAngularVelocity );

		vecLocalVelocity *= flDeltaTime;
		vecAngularVelocity *= flDeltaTime;

		// Convert linear velocity to world space
		Vector vecWorldLinearVelocity = vec3_origin;
		pObject->LocalToWorldVector( &vecWorldLinearVelocity, vecLocalVelocity );

		switch ( simResult )
		{
			case IMotionEvent::SIM_NOTHING:
			{
				break;
			}

			case IMotionEvent::SIM_LOCAL_ACCELERATION:
			{
				pObject->AddVelocity( &vecWorldLinearVelocity, &vecAngularVelocity );
				break;
			}

			case IMotionEvent::SIM_LOCAL_FORCE:
			{
				pObject->ApplyForceCenter( vecWorldLinearVelocity );
				pObject->ApplyTorqueCenter( vecAngularVelocity );
				break;
			}

			case IMotionEvent::SIM_GLOBAL_ACCELERATION:
			{
				pObject->AddVelocity( &vecLocalVelocity, &vecAngularVelocity );
				break;
			}

			case IMotionEvent::SIM_GLOBAL_FORCE:
			{
				pObject->ApplyForceCenter( vecLocalVelocity );
				pObject->ApplyTorqueCenter( vecAngularVelocity );
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
