
#pragma once

//-------------------------------------------------------------------------------------------------

class IVPhysicsKeyParser;

IVPhysicsKeyParser* CreateVPhysicsKeyParser( const char* pKeyData, bool bIsPacked );
void DestroyVPhysicsKeyParser( IVPhysicsKeyParser* pParser );
