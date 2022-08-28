
#include "cbase.h"

#include "vjolt_querymodel.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

JoltCollisionQuery::JoltCollisionQuery( JPH::Shape *pShape )
	: m_pShape ( pShape )
{
}

//-------------------------------------------------------------------------------------------------

int JoltCollisionQuery::ConvexCount()
{
	const JPH::StaticCompoundShape *pCompoundShape = GetCompoundShape( m_pShape );
	if ( pCompoundShape )
		return int( pCompoundShape->GetNumSubShapes() );

	// If we aren't a compound shape, then we have one.
	return 1;
}

int JoltCollisionQuery::TriangleCount( int convexIndex )
{
	return ActOnSubShape<int, JPH::Shape>( m_pShape, convexIndex, []( const JPH::Shape* pShape ) -> int
	{
		JPH::Shape::Stats stats = pShape->GetStats();
		return stats.mNumTriangles;
	} );
}

//-------------------------------------------------------------------------------------------------

unsigned int JoltCollisionQuery::GetGameData( int convexIndex )
{
	return static_cast< unsigned int >( m_pShape->GetUserData() );
}

//-------------------------------------------------------------------------------------------------

void JoltCollisionQuery::GetTriangleVerts( int convexIndex, int triangleIndex, Vector *verts )
{
	ActOnSubShape<int, JPH::Shape>( m_pShape, convexIndex, [&]( const JPH::Shape* pShape ) -> int
	{
		static constexpr int kRequestedTriangles = 2048;

		JPH::Shape::GetTrianglesContext ctx;
		pShape->GetTrianglesStart( ctx, JPH::AABox::sBiggest(), JPH::Vec3::sZero(), JPH::Quat::sIdentity(), JPH::Vec3( 1.0f, 1.0f, 1.0f ) );
		int i = 0;
		for ( ;; )
		{
			JPH::Float3 vertices[ kRequestedTriangles * 3 ];
			int count = pShape->GetTrianglesNext( ctx, kRequestedTriangles, vertices, nullptr /* materials */ );
			if ( count == 0 )
				break;

			if ( triangleIndex >= i && triangleIndex < i + count )
			{
				verts[ 0 ] = JoltToSource::Distance( vertices[ ( triangleIndex % kRequestedTriangles ) * 3 + 0 ] );
				verts[ 1 ] = JoltToSource::Distance( vertices[ ( triangleIndex % kRequestedTriangles ) * 3 + 1 ] );
				verts[ 2 ] = JoltToSource::Distance( vertices[ ( triangleIndex % kRequestedTriangles ) * 3 + 2 ] );
				return 0;
			}

			i += kRequestedTriangles;
		}

		return 1;
	} );

}

void JoltCollisionQuery::SetTriangleVerts( int convexIndex, int triangleIndex, const Vector *verts )
{
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

int JoltCollisionQuery::GetTriangleMaterialIndex( int convexIndex, int triangleIndex )
{
	Log_Stub( LOG_VJolt );
	return 0;
}

void JoltCollisionQuery::SetTriangleMaterialIndex( int convexIndex, int triangleIndex, int index7bits )
{
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------
