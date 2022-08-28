
#pragma once

#include "compat/better_winlite.h"

#ifdef _MSC_VER
#define VJOLT_RETURN_ADDRESS() _ReturnAddress()
#else
#define VJOLT_RETURN_ADDRESS() __builtin_return_address(0)
#endif

FORCEINLINE void GetCallingFunctionModulePath( void *pReturnAddress, char *pszModulePath, size_t len )
{
#ifdef _WIN32
    MEMORY_BASIC_INFORMATION mbi;
    if ( ::VirtualQuery( pReturnAddress, &mbi, sizeof(mbi)) ) {
        HMODULE module = reinterpret_cast< HMODULE >( mbi.AllocationBase );

        ::GetModuleFileNameA( module, pszModulePath, DWORD( len ) );
        return;
    }
#else
    V_strncpy( pszModulePath, "Unknown", len);
#endif
}
