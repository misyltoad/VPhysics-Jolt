
#pragma once

class JoltPhysicsObjectPairHash : public IPhysicsObjectPairHash
{
public:
	JoltPhysicsObjectPairHash();

	void AddObjectPair( void *pObject0, void *pObject1 ) override;
	void RemoveObjectPair( void *pObject0, void *pObject1 ) override;
	bool IsObjectPairInHash( void *pObject0, void *pObject1 ) override;
	void RemoveAllPairsForObject( void *pObject0 ) override;
	bool IsObjectInHash( void *pObject0 ) override;

	int GetPairCountForObject( void *pObject0 ) override;
	int GetPairListForObject( void *pObject0, int nMaxCount, void **ppObjectList ) override;

private:

	struct PointerHasher
	{
		template < typename T >
		static void HashCombine( size_t& seed, const T& v )
		{
			std::hash< T > hasher;
			seed ^= hasher( v ) + 0x9e3779b9 + ( seed << 6 ) + ( seed >> 2 );
		}

		size_t operator() ( const std::pair< void*, void* >& val ) const
		{
			size_t hash = 0;
			HashCombine( hash, val.first );
			HashCombine( hash, val.second );
			return hash;
		}
	};

	static constexpr size_t HashSize = 1024;
	static constexpr size_t GetHashArrayIndex( size_t hash )
	{
		return hash & ( HashSize - 1u );
	}

	using HashEntries = std::unordered_set< std::pair< void*, void* >, PointerHasher >;

	std::array< HashEntries, HashSize > m_PairHashes;
	std::array< HashEntries, HashSize > m_ObjectHashes;
	std::unordered_multiset< void* > m_Objects;
};
