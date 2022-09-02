//=================================================================================================
//
// The base physics DLL interface
// 
// This is a thin CPU-agnostic wrapper for the actual Volt DLLs which are named
// vphysics_jolt_sse2.dll, vphysics_jolt_sse42.dll and vphysics_jolt_avx2.dll
//
//=================================================================================================

#include "tier0/basetypes.h"
#include "tier1/interface.h"
#include "vphysics_interface.h"

#ifdef _WIN32
#include <intrin.h>
#else
#include <cpuid.h>
#endif

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

class PhysicsWrapper final : public CBaseAppSystem<IPhysics>
{
public:
	bool Connect( CreateInterfaceFn factory ) override;
	void Disconnect() override;

	InitReturnVal_t Init() override;
	void Shutdown() override;
	void *QueryInterface( const char *pInterfaceName ) override;

	IPhysicsEnvironment *CreateEnvironment() override;
	void DestroyEnvironment( IPhysicsEnvironment *pEnvironment ) override;
	IPhysicsEnvironment *GetActiveEnvironmentByIndex( int index ) override;

	IPhysicsObjectPairHash *CreateObjectPairHash() override;
	void DestroyObjectPairHash( IPhysicsObjectPairHash *pHash ) override;

	IPhysicsCollisionSet *FindOrCreateCollisionSet( unsigned int id, int maxElementCount ) override;
	IPhysicsCollisionSet *FindCollisionSet( unsigned int id ) override;
	void DestroyAllCollisionSets() override;

public:
	static PhysicsWrapper &GetInstance() { return s_PhysicsInterface; }

private:
	bool InitWrapper();

	CSysModule *m_pActualPhysicsModule;
	IPhysics *m_pActualPhysicsInterface;

	static PhysicsWrapper s_PhysicsInterface;
};

PhysicsWrapper PhysicsWrapper::s_PhysicsInterface;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR( PhysicsWrapper, IPhysics, VPHYSICS_INTERFACE_VERSION, PhysicsWrapper::GetInstance() );

//-------------------------------------------------------------------------------------------------

enum CPULevel_t
{
	CPU_HAS_SSE2,
	CPU_HAS_SSE42,
	CPU_HAS_AVX2,
};

static void GetCPUID( int *pInfo, int func, int subfunc )
{
#ifdef _WIN32
	__cpuidex( pInfo, func, subfunc );
#else
	__cpuid_count( func, subfunc, pInfo[0], pInfo[1], pInfo[2], pInfo[3] );
#endif
}

static CPULevel_t GetCPULevel()
{
	int cpuInfo[4];
	CPULevel_t cpuLevel = CPU_HAS_SSE2;

	GetCPUID( cpuInfo, 0, 0 ); // Get the number of functions
	const int numFuncs = cpuInfo[0];
	if ( numFuncs >= 7 )
	{
		GetCPUID( cpuInfo, 7, 0 ); // Call function 7
		bool hasAVX2 = cpuInfo[1] & ( 1 << 5 ); // 5 is the AVX2 bit
		if ( hasAVX2 )
			cpuLevel = CPU_HAS_AVX2;
	}
	else
	{
		GetCPUID( cpuInfo, 1, 0 ); // Call function 1
		bool hasSSE42 = cpuInfo[2] & ( 1 << 20 ); // 20 is the SSE42 bit
		if ( hasSSE42 )
			cpuLevel = CPU_HAS_SSE42;
	}

	return cpuLevel;
}

static const char *GetModuleFromCPULevel( CPULevel_t level )
{
	switch ( level )
	{
		case CPU_HAS_AVX2:		return "vphysics_jolt_avx2" DLL_EXT_STRING;
		case CPU_HAS_SSE42:		return "vphysics_jolt_sse42" DLL_EXT_STRING;
		default:				return "vphysics_jolt_sse2" DLL_EXT_STRING;
	}
}

// Tries to load the actual vphysics DLL
bool PhysicsWrapper::InitWrapper()
{
	if ( m_pActualPhysicsInterface )
		return true;

	const char *pModuleName = GetModuleFromCPULevel( GetCPULevel() );

	if ( !Sys_LoadInterface( pModuleName, VPHYSICS_INTERFACE_VERSION, &m_pActualPhysicsModule, (void **)&m_pActualPhysicsInterface ) )
		return false;

	return true;
}

bool PhysicsWrapper::Connect( CreateInterfaceFn factory )
{
	if ( !InitWrapper() )
		return false;

	return m_pActualPhysicsInterface->Connect( factory );
}

void PhysicsWrapper::Disconnect()
{
	m_pActualPhysicsInterface->Disconnect();

	Sys_UnloadModule( m_pActualPhysicsModule );
}

//-------------------------------------------------------------------------------------------------

InitReturnVal_t PhysicsWrapper::Init()
{
	return m_pActualPhysicsInterface->Init();
}

void PhysicsWrapper::Shutdown()
{
	m_pActualPhysicsInterface->Shutdown();
}

void *PhysicsWrapper::QueryInterface( const char *pInterfaceName )
{
	// This function can be called before Connect, so try and load the real DLL early
	if ( !InitWrapper() )
		return nullptr;

	return m_pActualPhysicsInterface->QueryInterface( pInterfaceName );
}

//-------------------------------------------------------------------------------------------------

IPhysicsEnvironment *PhysicsWrapper::CreateEnvironment()
{
	return m_pActualPhysicsInterface->CreateEnvironment();
}

void PhysicsWrapper::DestroyEnvironment( IPhysicsEnvironment *pEnvironment )
{
	m_pActualPhysicsInterface->DestroyEnvironment( pEnvironment );
}

IPhysicsEnvironment *PhysicsWrapper::GetActiveEnvironmentByIndex( int index )
{
	return m_pActualPhysicsInterface->GetActiveEnvironmentByIndex( index );
}

//-------------------------------------------------------------------------------------------------

IPhysicsObjectPairHash *PhysicsWrapper::CreateObjectPairHash()
{
	return m_pActualPhysicsInterface->CreateObjectPairHash();
}

void PhysicsWrapper::DestroyObjectPairHash( IPhysicsObjectPairHash *pHash )
{
	m_pActualPhysicsInterface->DestroyObjectPairHash( pHash );
}

//-------------------------------------------------------------------------------------------------

IPhysicsCollisionSet *PhysicsWrapper::FindOrCreateCollisionSet( unsigned int id, int maxElementCount )
{
	return m_pActualPhysicsInterface->FindOrCreateCollisionSet( id, maxElementCount );
}

IPhysicsCollisionSet *PhysicsWrapper::FindCollisionSet( unsigned int id )
{
	return m_pActualPhysicsInterface->FindCollisionSet( id );
}

void PhysicsWrapper::DestroyAllCollisionSets()
{
	m_pActualPhysicsInterface->DestroyAllCollisionSets();
}
