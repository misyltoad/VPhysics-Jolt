
#pragma once

#include "memalloc.h"

#define DevAssert( ... )
#define DevAssertMsg( ... )
#define AssertMsg_Internal( ... )

template< typename A, typename B, typename C >
[[nodiscard]] constexpr A clamp( const A val, const B minVal, const C maxVal )
{
	return MIN( MAX( val, minVal ), maxVal );
}

template< typename T >
[[nodiscard]] constexpr T Min( const T valMin, const T valMax )
{
	return valMin < valMax ? valMin : valMax;
}

template< typename T >
[[nodiscard]] constexpr T Max( const T valMin, const T valMax )
{
	return valMin > valMax ? valMin : valMax;
}

template< typename T >
[[nodiscard]] constexpr T Clamp( const T val, const T minVal, const T maxVal )
{
	return Min( Max( val, minVal ), maxVal );
}

constexpr bool V_isdigit( char c )
{
	return c >= '0' && c <= '9';
}

inline void MemAlloc_Free( void *pMemBlock )
{
	g_pMemAlloc->Free( pMemBlock );
}

enum PlayerContactState_t
{
    PLAYER_CONTACT_PHYSICS = 1,
    PLAYER_CONTACT_GAMEOBJECT = 2,
};

