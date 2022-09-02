
#include "cbase.h"

#include "vjolt_objectpairhash.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

static std::pair< void *, void * > CreateSortedPair( void *pObject0, void *pObject1 )
{
    return std::make_pair(
        pObject0 <= pObject1 ? pObject0 : pObject1,
        pObject0 <= pObject1 ? pObject1 : pObject0 );
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsObjectPairHash::JoltPhysicsObjectPairHash()
{
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObjectPairHash::AddObjectPair( void *pObject0, void *pObject1 )
{
    auto pair = CreateSortedPair( pObject0, pObject1 );

    if ( IsObjectPairInHash( pObject0, pObject1 ) )
        return;

    m_PairHashes[ GetHashArrayIndex( PointerHasher{}( pair ) ) ].emplace( pair );
    m_ObjectHashes[ GetHashArrayIndex( std::hash< void* >()( pObject0 ) ) ].emplace( pair );
    m_ObjectHashes[ GetHashArrayIndex( std::hash< void* >()( pObject1 ) ) ].emplace( pair );
    m_Objects.emplace( pObject0 );
    m_Objects.emplace( pObject1 );
}

void JoltPhysicsObjectPairHash::RemoveObjectPair( void *pObject0, void *pObject1 )
{
    auto pair = CreateSortedPair( pObject0, pObject1 );

    if ( !IsObjectPairInHash( pObject0, pObject1 ) )
        return;

    m_PairHashes[ GetHashArrayIndex( PointerHasher{}( pair ) ) ].erase( pair );
    m_ObjectHashes[ GetHashArrayIndex( std::hash< void* >()( pObject0 ) ) ].erase( pair );
    m_ObjectHashes[ GetHashArrayIndex( std::hash< void* >()( pObject1 ) ) ].erase( pair );
    m_Objects.erase( pObject0 );
    m_Objects.erase( pObject1 );
}

bool JoltPhysicsObjectPairHash::IsObjectPairInHash( void *pObject0, void *pObject1 )
{
    auto pair = CreateSortedPair( pObject0, pObject1 );
	return Contains( m_PairHashes[GetHashArrayIndex( PointerHasher{}( pair ) )], pair );
}

void JoltPhysicsObjectPairHash::RemoveAllPairsForObject( void *pObject0 )
{
    auto &objectHashes = m_ObjectHashes[ GetHashArrayIndex( std::hash< void* >()( pObject0 ) ) ];

    for ( auto it = objectHashes.begin(); it != objectHashes.end(); )
    {
        auto pair = *it++;

        RemoveObjectPair( pair.first, pair.second );
    }
}

bool JoltPhysicsObjectPairHash::IsObjectInHash( void *pObject0 )
{
	return Contains( m_Objects, pObject0 );
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsObjectPairHash::GetPairCountForObject( void *pObject0 )
{
    return int( m_Objects.count( pObject0 ) );
}

int JoltPhysicsObjectPairHash::GetPairListForObject( void *pObject0, int nMaxCount, void **ppObjectList )
{
    auto& objectHashes = m_ObjectHashes[GetHashArrayIndex( std::hash< void* >()( pObject0 ) )];

    int nCount = 0;
    for ( auto it = objectHashes.begin(); it != objectHashes.end() && nCount < nMaxCount; ++it, ++nCount )
    {
        auto pair = *it;
        ppObjectList[ nCount ] = pair.second != pObject0 ? pair.second : pair.first;
    }

    return nCount;
}
