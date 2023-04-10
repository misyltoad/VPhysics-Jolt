
#pragma once

#if defined( GAME_CSGO ) || defined( GAME_DESOLATION )
#define GAME_CSGO_OR_NEWER
#define override_csgo override
#define override_not_csgo
#else
#define override_csgo
#define override_not_csgo override
#endif

// GMod SDK2013 x86 branch
#if defined( GAME_GMOD )
#define override_gmod override
#define override_not_gmod
#else
#define override_gmod
#define override_not_gmod override
#endif

#if defined( GAME_CSGO ) || defined( GAME_DESOLATION ) || defined( GAME_PORTAL2 )
#define GAME_PORTAL2_OR_NEWER
#define override_portal2 override
#define override_not_portal2
#else
#define override_portal2
#define override_not_portal2 override
#endif

#if defined( GAME_CSGO ) || defined( GAME_DESOLATION ) || defined( GAME_PORTAL2 ) || defined( GAME_L4D2 )
#define GAME_L4D2_OR_NEWER
#define override_l4d2 override
#define override_not_l4d2
#else
#define override_l4d2
#define override_not_l4d2 override
#endif

#if defined( GAME_CSGO ) || defined( GAME_DESOLATION ) || defined( GAME_PORTAL2 ) || defined( GAME_L4D2 ) || defined( GAME_ASW )
#define GAME_ASW_OR_NEWER
#define override_asw override
#define override_not_asw
#else
#define override_asw
#define override_not_asw override
#endif

#ifndef GAME_DESOLATION
using strlen_t = int;
#endif

#ifndef GAME_CSGO_OR_NEWER
#define FastASCIIToUpper( c ) ( ( ( (c) >= 'a' ) && ( (c) <= 'z' ) ) ? ( (c) - 32 ) : (c) )
#define FastASCIIToLower( c ) ( ( ( (c) >= 'A' ) && ( (c) <= 'Z' ) ) ? ( (c) + 32 ) : (c) )
#endif
