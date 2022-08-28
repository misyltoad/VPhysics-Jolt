//=================================================================================================
//
// The base physics DLL interface
//
//=================================================================================================

#include "cbase.h"

#include "vjolt_environment.h"
#include "vjolt_collide.h"
#include "vjolt_surfaceprops.h"
#include "vjolt_objectpairhash.h"

#include "vjolt_interface.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

// Slart:
// Pre-allocate 64 megabytes for physics allocations.
// I don't think we've tuned this value. It's just a big number that we probably won't ever hit.
static constexpr uint kTempAllocSize = 64 * 1024 * 1024;

// Josh:
// We cannot support more than 64 threads doing physics work because
// of the code I wrote in vjolt_listener_contact to dispatch events.
// It uses a single uint64_t bitmask that is iterated on for the thread-local
// event vectors.
// This isn't an issue, the benefits of more threads tends to trail off between
// 8-16 threads anyway.
static constexpr uint kMaxPhysicsThreads = 64;

DEFINE_LOGGING_CHANNEL_NO_TAGS( LOG_VJolt, "VJolt", 0, LS_MESSAGE, Color( 205, 142, 212, 255 ) );
DEFINE_LOGGING_CHANNEL_NO_TAGS( LOG_JoltInternal, "Jolt" );

JoltPhysicsInterface JoltPhysicsInterface::s_PhysicsInterface;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR( JoltPhysicsInterface, IPhysics, VPHYSICS_INTERFACE_VERSION, JoltPhysicsInterface::GetInstance() );

//-------------------------------------------------------------------------------------------------

// Slart:
// Instead of using Jolt's allocator override functionality, we disable it and just define the
// functions here, all of Jolt's memory allocation goes through here, besides new and delete
// which use the Valve overrides in memoverride.cpp.
// For Desolation we use mi-malloc rather than dlmalloc, that also gets built into the statically
// linked releases for gmod (along with all of tier0 and vstdlib).
namespace JPH {

	void *Allocate( size_t inSize )
	{
		return MemAlloc_Alloc( inSize );
	}

	void Free( void *inBlock )
	{
		MemAlloc_Free( inBlock );
	}

	void *AlignedAllocate( size_t inSize, size_t inAlignment )
	{
		return MemAlloc_AllocAligned( inSize, inAlignment );
	}

	void AlignedFree( void *inBlock )
	{
		MemAlloc_FreeAligned( inBlock );
	}
}

//-------------------------------------------------------------------------------------------------

InitReturnVal_t JoltPhysicsInterface::Init()
{
	const InitReturnVal_t nRetVal = BaseClass::Init();
	if ( nRetVal != INIT_OK )
	{
		return nRetVal;
	}

	MathLib_Init();

	// Install callbacks
	JPH::Trace = JoltPhysicsInterface::OnTrace;
	JPH_IF_ENABLE_ASSERTS( JPH::AssertFailed = JoltPhysicsInterface::OnAssert; )

	// Create a factory
	JPH::Factory::sInstance = new JPH::Factory();

	// Register all Jolt physics types
	JPH::RegisterTypes();

	// Create an allocator for temporary allocations during physics simulations
	m_pTempAllocator = new JPH::TempAllocatorImpl( kTempAllocSize );

	// Josh:
	// We may want to replace this with a better heuristic, or add a launch arg for this in future.
	// Right now, this does what -1 does in Jolt, but limits it to 64 threads, as we cannot support
	// more than this (see above).
	const uint32 threadCount = Min( std::thread::hardware_concurrency() - 1, kMaxPhysicsThreads );
	m_pJobSystem = new JPH::JobSystemThreadPool( JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, threadCount );

	return INIT_OK;
}

void JoltPhysicsInterface::Shutdown()
{
	delete m_pJobSystem;
	delete m_pTempAllocator;
	delete JPH::Factory::sInstance;

	BaseClass::Shutdown();
}

void *JoltPhysicsInterface::QueryInterface( const char *pInterfaceName )
{
	CreateInterfaceFn factory = Sys_GetFactoryThis();
	return factory( pInterfaceName, NULL );	
}

//-------------------------------------------------------------------------------------------------

IPhysicsEnvironment *JoltPhysicsInterface::CreateEnvironment()
{
	return new JoltPhysicsEnvironment();
}

void JoltPhysicsInterface::DestroyEnvironment( IPhysicsEnvironment *pEnvironment )
{
	delete static_cast<JoltPhysicsEnvironment *>( pEnvironment );
}

IPhysicsEnvironment *JoltPhysicsInterface::GetActiveEnvironmentByIndex( int index )
{
	// Josh: Nothing uses this... ever.
	Log_Stub( LOG_VJolt );
	return nullptr;
}

//-------------------------------------------------------------------------------------------------

IPhysicsObjectPairHash *JoltPhysicsInterface::CreateObjectPairHash()
{
	return new JoltPhysicsObjectPairHash;
}

void JoltPhysicsInterface::DestroyObjectPairHash( IPhysicsObjectPairHash *pHash )
{
	delete static_cast<JoltPhysicsObjectPairHash *>( pHash );
}

//-------------------------------------------------------------------------------------------------

IPhysicsCollisionSet *JoltPhysicsInterface::FindOrCreateCollisionSet( unsigned int id, int maxElementCount )
{
	if ( maxElementCount > 32 )
		return nullptr;

	if ( IPhysicsCollisionSet *pSet = FindCollisionSet( id ) )
		return pSet;

	auto result = m_CollisionSets.emplace( id, JoltPhysicsCollisionSet{} );
	return &result.first->second;
}

IPhysicsCollisionSet *JoltPhysicsInterface::FindCollisionSet( unsigned int id )
{
	auto iter = m_CollisionSets.find( id );
	if ( iter != m_CollisionSets.end() )
		return &iter->second;

	return nullptr;
}

void JoltPhysicsInterface::DestroyAllCollisionSets()
{
	m_CollisionSets.clear();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsInterface::OnTrace( const char *fmt, ... )
{
	va_list args;
	char msg[MAX_LOGGING_MESSAGE_LENGTH];

	va_start( args, fmt );
	V_vsnprintf( msg, sizeof( msg ), fmt, args );
	va_end( args );

	Log_Msg( LOG_JoltInternal, "%s\n", msg );
}

bool JoltPhysicsInterface::OnAssert( const char *inExpression, const char *inMessage, const char *inFile, uint inLine )
{
	const char *message = inMessage ? inMessage : inExpression;
	(void) message;
	AssertMsg_Internal( false, inLine, inFile, message );
	return false;
}
