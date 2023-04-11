
#include "cbase.h"

#include "coordsize.h"
#include "mathlib/polyhedron.h"

#include "vjolt_parse.h"
#include "vjolt_querymodel.h"

#include "vjolt_collide.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

// Also in vjolt_collide_trace.cpp, should unify or just remove entirely
static constexpr float kMaxConvexRadius = JPH::cDefaultConvexRadius;

JoltPhysicsCollision JoltPhysicsCollision::s_PhysicsCollision;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR( JoltPhysicsCollision, IPhysicsCollision, VPHYSICS_COLLISION_INTERFACE_VERSION, JoltPhysicsCollision::GetInstance() );

//-------------------------------------------------------------------------------------------------

// Creates a shape from shape settings, resolving the reference
template < typename ShapeType, typename T >
ShapeType *ShapeSettingsToShape( const T& settings )
{
	auto result = settings.Create();
	if ( !result.IsValid() )
	{
		// Josh:
		// Need to handle degenerate convexes and stuff.
		const char *error = result.HasError()
			? result.GetError().c_str()
			: "Unknown";
 
		Log_Warning( LOG_VJolt, "Failed to create shape: %s.\n", error );
		return nullptr;
	}

	return static_cast< ShapeType * >( ToDanglingRef( result.Get() ) );
}

// Creates a JPH::ConvexShape from shape settings, and casts to a CPhysConvex
template < typename T >
CPhysConvex *ShapeSettingsToPhysConvex( const T& settings )
{
	return CPhysConvex::FromConvexShape( ShapeSettingsToShape< JPH::ConvexShape >( settings ) );
}

// Creates a JPH::Shape from shape settings, and casts to a CPhysCollide
template < typename T >
CPhysCollide *ShapeSettingsToPhysCollide( const T& settings )
{
	return CPhysCollide::FromShape( ShapeSettingsToShape< JPH::Shape >( settings ) );
}

//-------------------------------------------------------------------------------------------------

CPhysConvex *JoltPhysicsCollision::ConvexFromVerts( Vector **pVerts, int vertCount )
{
	// This uses this and not a CUtlVector or std::vector or whatever for two reasons:
	// 1: This is a single allocation we can then toss away instead of growing
	// 2: We do not want to initialize these vectors on allocation
	// 3: Automatic deletion when out of scope
	// Please do not change me to either.
	std::unique_ptr< JPH::Vec3[] > verts = std::make_unique< JPH::Vec3[] >( vertCount );
	for ( int i = 0; i < vertCount; i++ )
		verts[ i ] = SourceToJolt::Distance( *pVerts[ i ] );

	JPH::ConvexHullShapeSettings settings( verts.get(), vertCount, kMaxConvexRadius, nullptr /* material */ );
	settings.mHullTolerance = 0.0f;
	return ShapeSettingsToPhysConvex( settings );
}

CPhysConvex *JoltPhysicsCollision::ConvexFromPlanes( float *pPlanes, int planeCount, float mergeDistance )
{
	Log_Stub( LOG_VJolt );
	return nullptr;
}

float JoltPhysicsCollision::ConvexVolume( CPhysConvex *pConvex )
{
	return JoltToSource::Volume( pConvex->ToConvexShape()->GetVolume() );
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsCollision::ConvexSurfaceArea( CPhysConvex *pConvex )
{
	Log_Stub( LOG_VJolt );
	return 0.0f;
}

void JoltPhysicsCollision::SetConvexGameData( CPhysConvex *pConvex, unsigned int gameData )
{
	pConvex->ToConvexShape()->SetUserData( gameData );
}

void JoltPhysicsCollision::ConvexFree( CPhysConvex *pConvex )
{
	pConvex->ToConvexShape()->Release();
}

CPhysConvex *JoltPhysicsCollision::BBoxToConvex( const Vector &mins, const Vector &maxs )
{
	JPH::AABox aabox = SourceToJolt::AABBBounds( mins, maxs );

	JPH::BoxShape *pBoxShape = new JPH::BoxShape( aabox.GetExtent(), kMaxConvexRadius, nullptr /* material */ );

	JPH::RotatedTranslatedShapeSettings rotatedSettings( aabox.mMin + aabox.GetExtent(), JPH::Quat::sIdentity(), pBoxShape );
	return ShapeSettingsToPhysConvex( rotatedSettings );
}

CPhysConvex *JoltPhysicsCollision::ConvexFromConvexPolyhedron( const CPolyhedron &ConvexPolyhedron )
{
	std::unique_ptr< JPH::Vec3[] > pPoints = std::make_unique< JPH::Vec3[] >( ConvexPolyhedron.iVertexCount );

	// This loop fills me with rage
	for ( unsigned short i = 0; i < ConvexPolyhedron.iVertexCount; ++i )
		pPoints[i] = SourceToJolt::Distance( ConvexPolyhedron.pVertices[i] );

	JPH::ConvexHullShapeSettings settings( pPoints.get(), ConvexPolyhedron.iVertexCount, kMaxConvexRadius, nullptr /* material */);
	settings.mHullTolerance = 0.0f; // Slart: Otherwise some polyhedrons crash :(
	CPhysConvex *pPhysConvex = ShapeSettingsToPhysConvex( settings );

	return pPhysConvex;
}

void JoltPhysicsCollision::ConvexesFromConvexPolygon( const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput )
{
	// Slart: Unused
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

CPhysPolysoup *JoltPhysicsCollision::PolysoupCreate()
{
	return new CPhysPolysoup;
}

void JoltPhysicsCollision::PolysoupDestroy( CPhysPolysoup *pSoup )
{
	delete pSoup;
}

void JoltPhysicsCollision::PolysoupAddTriangle( CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits )
{
	// Add both windings to make this two-faced.
	pSoup->Triangles.push_back( JPH::Triangle( SourceToJolt::DistanceFloat3( c ), SourceToJolt::DistanceFloat3( b ), SourceToJolt::DistanceFloat3( a ) ) );
	pSoup->Triangles.push_back( JPH::Triangle( SourceToJolt::DistanceFloat3( a ), SourceToJolt::DistanceFloat3( b ), SourceToJolt::DistanceFloat3( c ) ) );
}

CPhysCollide *JoltPhysicsCollision::ConvertPolysoupToCollide( CPhysPolysoup *pSoup, bool useMOPP )
{
	VJoltAssertMsg( !useMOPP, "MOPPs not supported\n" );

	if ( useMOPP )
		return nullptr;

	// ConvertPolysoupToCollide does NOT free the Polysoup.
	JPH::MeshShapeSettings settings( pSoup->Triangles );
	return ShapeSettingsToPhysCollide( settings );
}

//-------------------------------------------------------------------------------------------------

CPhysCollide *JoltPhysicsCollision::ConvertConvexToCollide( CPhysConvex **pConvex, int convexCount )
{
	// If we only have one convex shape, we can just use that directly,
	// without making a compound shape.
	if ( convexCount == 1 )
		return pConvex[0]->ToPhysCollide();

	JPH::StaticCompoundShapeSettings settings;
	for ( int i = 0; i < convexCount; i++ )
		settings.AddShape( JPH::Vec3::sZero(), JPH::Quat::sIdentity(), pConvex[i]->ToConvexShape() );

	CPhysCollide *pCollide = ShapeSettingsToPhysCollide( settings );

	// This function also 'frees' the convexes.
	for ( int i = 0; i < convexCount; i++ )
		pConvex[i]->ToConvexShape()->Release();

	return pCollide;
}

CPhysCollide *JoltPhysicsCollision::ConvertConvexToCollideParams( CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams )
{
	// Slart: The parameters are just IVP crap and the only one that isn't is never ever used. HA!
	return ConvertConvexToCollide( pConvex, convexCount );
}

void JoltPhysicsCollision::DestroyCollide( CPhysCollide *pCollide )
{
	if ( !pCollide )
		return;

	pCollide->ToShape()->Release();
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsCollision::CollideSize( CPhysCollide *pCollide )
{
	Log_Stub( LOG_VJolt );
	return 0;
}

int JoltPhysicsCollision::CollideWrite( char *pDest, CPhysCollide *pCollide, bool bSwap /*= false*/ )
{
	Log_Stub( LOG_VJolt );
	return 0;
}

CPhysCollide *JoltPhysicsCollision::UnserializeCollide( char *pBuffer, int size, int index )
{
	Log_Stub( LOG_VJolt );
	return nullptr;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsCollision::CollideVolume( CPhysCollide *pCollide )
{
	return JoltToSource::Volume( pCollide->ToShape()->GetVolume() );
}

float JoltPhysicsCollision::CollideSurfaceArea( CPhysCollide *pCollide )
{
	Log_Stub( LOG_VJolt );
	return 0.0f;
}

//-------------------------------------------------------------------------------------------------

Vector JoltPhysicsCollision::CollideGetExtent( const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction )
{
	if ( !pCollide )
		return collideOrigin;

	const JPH::Shape *pShape = pCollide->ToShape();

	JPH::Vec3 vecDirection = JPH::Vec3( direction.x, direction.y, direction.z );

	JPH::Mat44 matCollideTransform = JPH::Mat44::sRotationTranslation(
		SourceToJolt::Angle( collideAngles ), SourceToJolt::Distance( collideOrigin ) - pShape->GetCenterOfMass() );

	float flMaxDot = -FLT_MAX;
	JPH::Vec3 vecMaxExtent = JPH::Vec3::sZero();
	ActOnSubShapes< JPH::ConvexShape >( pShape, [&]( const JPH::ConvexShape* pConvexShape, JPH::Mat44Arg matSubShapeTransform )
	{
		JPH::Mat44 matTransform = matCollideTransform * matSubShapeTransform;
		JPH::ConvexShape::SupportingFace supportingFace;
		pConvexShape->GetSupportingFace( JPH::SubShapeID(), vecDirection, JPH::Vec3::sReplicate( 1.0f ), matTransform, supportingFace );

		for ( const JPH::Vec3 &vecVertex : supportingFace )
		{
			const float flDot = vecVertex.Dot( vecDirection );
			if ( flDot > flMaxDot )
			{
				vecMaxExtent = vecVertex;
				flMaxDot = flDot;
			}
		}
	});

	return JoltToSource::Distance( vecMaxExtent );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsCollision::CollideGetAABB( Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles )
{
	JPH::Vec3 position = SourceToJolt::Distance( collideOrigin );
	JPH::Quat rotation = SourceToJolt::Angle( collideAngles );

	const JPH::Shape *pShape = pCollide->ToShape();

	const JPH::Mat44 translation = JPH::Mat44::sRotationTranslation( rotation, position + rotation * pShape->GetCenterOfMass() );
	const JPH::AABox worldBounds = pShape->GetWorldSpaceBounds( translation, JPH::Vec3::sReplicate( 1.0f ) );

	JoltToSource::AABBBounds( worldBounds, *pMins, *pMaxs );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsCollision::CollideGetMassCenter( CPhysCollide *pCollide, Vector *pOutMassCenter )
{
	*pOutMassCenter = JoltToSource::Distance( pCollide->ToShape()->GetCenterOfMass() );
}

void JoltPhysicsCollision::CollideSetMassCenter( CPhysCollide *pCollide, const Vector &massCenter )
{
	// Slart: Only used in studiomdl
	Log_Stub( LOG_VJolt );
}

Vector JoltPhysicsCollision::CollideGetOrthographicAreas( const CPhysCollide *pCollide )
{
	// Slart: Only used in studiomdl... In a part that is #if 0'd out...
	Log_Stub( LOG_VJolt );
	return vec3_origin;
}

void JoltPhysicsCollision::CollideSetOrthographicAreas( CPhysCollide *pCollide, const Vector &areas )
{
	// Slart: Never used
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsCollision::CollideIndex( const CPhysCollide *pCollide )
{
	// Slart: Only used by code behind #ifdef _DEBUG
	Log_Stub( LOG_VJolt );
	return 0;
}

//-------------------------------------------------------------------------------------------------

CPhysCollide *JoltPhysicsCollision::BBoxToCollide( const Vector &mins, const Vector &maxs )
{
	return BBoxToConvex( mins, maxs )->ToPhysCollide();
}

int JoltPhysicsCollision::GetConvexesUsedInCollideable( const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit )
{
	const JPH::Shape *pShape = pCollideable->ToShape();
	JPH::EShapeType shapeType = pShape->GetType();
	if ( shapeType != JPH::EShapeType::Compound )
	{
		pOutputArray[0] = const_cast<CPhysConvex *>( CPhysConvex::FromConvexShape( static_cast<const JPH::ConvexShape *>( pShape ) ) );
		return 1;
	}

	const JPH::StaticCompoundShape *pCompoundShape = static_cast<const JPH::StaticCompoundShape *>( pShape );
	const JPH::StaticCompoundShape::SubShapes &subShapes = pCompoundShape->GetSubShapes();

	const uint maxNumShapes = Min<uint>( pCompoundShape->GetNumSubShapes(), iOutputArrayLimit );
	for ( uint i = 0; i < maxNumShapes; ++i )
		pOutputArray[i] = const_cast<CPhysConvex *>( CPhysConvex::FromConvexShape( static_cast<const JPH::ConvexShape *>( subShapes[i].mShape.GetPtr() ) ) );

	return maxNumShapes;
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsCollision::IsBoxIntersectingCone( const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone )
{
	// Slart: Never used
	Log_Stub( LOG_VJolt );
	return false;
}

//-------------------------------------------------------------------------------------------------

//
// See studiobyteswap from the public 2013 SDK for more info about these defines.
// https://github.com/ValveSoftware/source-sdk-2013/blob/master/sp/src/common/studiobyteswap.cpp
//

namespace ivp_compat
{
	struct collideheader_t
	{
		int		vphysicsID;
		short	version;
		short	modelType;
	};

	struct compactsurfaceheader_t
	{
		int		surfaceSize;
		Vector	dragAxisAreas;
		int		axisMapSize;
	};

	struct moppsurfaceheader_t
	{
		int moppSize;
	};

	struct compactsurface_t
	{
		float	mass_center[3];
		float	rotation_inertia[3];
		float	upper_limit_radius;

		unsigned int	max_factor_surface_deviation	: 8;
		int		        byte_size						: 24;
		int		        offset_ledgetree_root;
		int		        dummy[3];
	};

	struct compactmopp_t
	{
		float	mass_center[3];
		float	rotation_inertia[3];
		float	upper_limit_radius;

		unsigned int max_factor_surface_deviation	: 8;
		int          byte_size						: 24;
		int	         offset_ledgetree_root;
		int          offset_ledges;
		int          size_convex_hull;
		int	         dummy;
	};

	struct compactledge_t
	{
		int		c_point_offset;

		union
		{
			int	ledgetree_node_offset;
			int	client_data;
		};

		struct
		{
			uint	has_children_flag	: 2;
			int		is_compact_flag		: 2;
			uint	dummy				: 4;
			uint	size_div_16			: 24; 
		};

		short	n_triangles;
		short	for_future_use;
	};

	struct compactedge_t
	{
		uint    start_point_index	: 16;
		int		opposite_index		: 15;
		uint	is_virtual			: 1;
	};

	struct compacttriangle_t
	{
		uint	tri_index		: 12;
		uint	pierce_index	: 12;
		uint	material_index	: 7;
		uint	is_virtual		: 1;
		compactedge_t c_three_edges[3];
	};

	struct compactledgenode_t
	{
		int		offset_right_node;
		int		offset_compact_ledge;
		float	center[3];
		float	radius;
		unsigned char box_sizes[3];
		unsigned char free_0;

		const compactledge_t *GetCompactLedge() const
		{
			VJoltAssert( this->offset_right_node == 0 );
			return ( compactledge_t * )( ( char * )this + this->offset_compact_ledge );
		}

		const compactledgenode_t *GetLeftChild() const
		{
			VJoltAssert( this->offset_right_node );
			return this + 1;
		}

		const compactledgenode_t *GetRightChild() const
		{
			VJoltAssert( this->offset_right_node );
			return ( compactledgenode_t * )( ( char * )this + this->offset_right_node );
		}

		bool IsTerminal() const
		{
			return this->offset_right_node == 0;
		}

		const compactledge_t *GetCompactHull() const
		{
			if ( this->offset_compact_ledge )
				return ( compactledge_t * )( ( char * )this + this->offset_compact_ledge );
			else
				return nullptr;
		}
	};

	static constexpr int	IVP_COMPACT_SURFACE_SUPER_LEGACY	= 0; // Really old .phy files, don't have anything here, and were just serialized compact headers directly.
	static constexpr int	IVP_COMPACT_SURFACE_ID				= MAKEID('I','V','P','S');
	static constexpr int	IVP_COMPACT_SURFACE_ID_SWAPPED		= MAKEID('S','P','V','I');
	static constexpr int	IVP_COMPACT_MOPP_ID					= MAKEID('M','O','P','P');
	static constexpr int	VPHYSICS_COLLISION_ID				= MAKEID('V','P','H','Y');
	static constexpr short	VPHYSICS_COLLISION_VERSION			= 0x0100;

	enum
	{
		COLLIDE_POLY = 0,
		COLLIDE_MOPP = 1,
		COLLIDE_BALL = 2,
		COLLIDE_VIRTUAL = 3,
	};

	JPH::ConvexShape *IVPLedgeToConvexShape( const compactledge_t *pLedge )
	{
		if ( !pLedge->n_triangles )
			return nullptr;

		const char *pVertices = reinterpret_cast< const char * >( pLedge ) + pLedge->c_point_offset;
		const compacttriangle_t *pTriangles = reinterpret_cast< const compacttriangle_t * >( pLedge + 1 );

		const int nVertCount = pLedge->n_triangles * 3;

		std::unique_ptr< JPH::Vec3[] > verts = std::make_unique< JPH::Vec3[] >( nVertCount );

		// Each triangle
		for ( int i = 0; i < pLedge->n_triangles; i++ )
		{
			// For each point of the current triangle
			for ( int j = 0; j < 3; j++ )
			{
				static constexpr size_t IVPAlignedVectorSize = 16;

				const int nIndex = pTriangles[ i ].c_three_edges[ j ].start_point_index;
				const float *pVertex = reinterpret_cast< const float * >( pVertices + ( nIndex * IVPAlignedVectorSize ) );

				verts[ ( i * 3 ) + j ] = JPH::Vec3( pVertex[ 0 ], pVertex[ 2 ], -pVertex[ 1 ] );
			}
		}

		JPH::ConvexHullShapeSettings settings{ verts.get(), nVertCount, kMaxConvexRadius, nullptr /* material */ };
		settings.mHullTolerance = 0.0f;
		JPH::ConvexShape *pConvexShape = ShapeSettingsToShape< JPH::ConvexShape >( settings );
		if ( !pConvexShape )
		{
			// Wow that sucks, just mock up a small sphere to subsitute.
			// This can happen for models with extremely broken collision hulls.
			// If we don't do this, we'll crash later on because older versions of Source are missing
			// an important nullptr check.
			// A better solution would be to generate a valid convex hull from the points provided.
			JPH::SphereShapeSettings sphereSettings( 1.0f );
			pConvexShape = ShapeSettingsToShape< JPH::ConvexShape >( sphereSettings );
			if ( !pConvexShape )
			{
				// This should never fail, but catching anyway
				return nullptr;
			}
		}
		
		pConvexShape->SetUserData( pLedge->client_data );
		return pConvexShape;
	}

	void GetAllIVPEdges( const compactledgenode_t *pNode, CUtlVector< const compactledge_t * >& vecOut )
	{
		if ( !pNode )
			return;

		if ( !pNode->IsTerminal() )
		{
			GetAllIVPEdges( pNode->GetRightChild(), vecOut );
			GetAllIVPEdges( pNode->GetLeftChild(), vecOut );
		}
		else
			vecOut.AddToTail( pNode->GetCompactLedge() );
	}

	CPhysCollide *DeserializeIVP_Poly( const compactsurface_t* pSurface )
	{
		const compactledgenode_t *pFirstLedgeNode = reinterpret_cast< const compactledgenode_t * >(
				reinterpret_cast< const char * >( pSurface ) + pSurface->offset_ledgetree_root );

		CUtlVector< const compactledge_t * > ledges;
		GetAllIVPEdges( pFirstLedgeNode, ledges );

		VJoltAssert( !ledges.IsEmpty() );

		if ( ledges.Count() != 1 )
		{
			JPH::StaticCompoundShapeSettings settings{};
			// One compound convex per ledge.
			for ( int i = 0; i < ledges.Count(); i++ )
			{
				const JPH::Shape* pShape = IVPLedgeToConvexShape( ledges[i] );
				// Josh:
				// Some models have degenerate convexes which fails to make
				// a subshape in Jolt, so we need to ignore those ledges.
				if ( pShape )
					settings.AddShape( JPH::Vec3::sZero(), JPH::Quat::sIdentity(), pShape );
			}
			CPhysCollide* pCollide = ShapeSettingsToPhysCollide( settings );
			return pCollide;
		}
		else
		{
			JPH::ConvexShape *pShape = IVPLedgeToConvexShape( ledges[ 0 ] );
			return CPhysConvex::FromConvexShape( pShape )->ToPhysCollide();
		}
	}

	CPhysCollide *DeserializeIVP_Poly( const collideheader_t *pCollideHeader )
	{
		const compactsurfaceheader_t *pSurfaceHeader = reinterpret_cast< const compactsurfaceheader_t* >( pCollideHeader + 1 );
		const compactsurface_t *pSurface = reinterpret_cast< const compactsurface_t* >( pSurfaceHeader + 1 );

		return DeserializeIVP_Poly( pSurface );
	}
}

void JoltPhysicsCollision::VCollideLoad( vcollide_t *pOutput, int solidCount, const char *pBuffer, int size, bool swap /*= false*/ )
{
	if ( swap )
	{
		Log_Error( LOG_VJolt, "If you got here. Tell me what you did!\n" );
		return;
	}

	pOutput->solidCount = solidCount;
	pOutput->solids = new CPhysCollide*[ solidCount ];

	const char *pCursor = pBuffer;
	for ( int i = 0; i < solidCount; i++ )
	{
		// Be safe ahead of time as so much can go wrong with
		// this mess! :p
		pOutput->solids[ i ] = nullptr;

		const int solidSize = *reinterpret_cast<const int *>( pCursor );
		pCursor += sizeof( int );

		const ivp_compat::collideheader_t *pCollideHeader = reinterpret_cast<const ivp_compat::collideheader_t *>( pCursor );

		if ( pCollideHeader->vphysicsID == ivp_compat::VPHYSICS_COLLISION_ID )
		{
			// This is the main path that everything falls down for a modern
			// .phy file with the collide header.

			if ( pCollideHeader->version != ivp_compat::VPHYSICS_COLLISION_VERSION )
				Log_Warning( LOG_VJolt, "Solid with unknown version: 0x%x, may crash!\n", pCollideHeader->version );

			switch ( pCollideHeader->modelType )
			{
				case ivp_compat::COLLIDE_POLY:
					pOutput->solids[ i ] = DeserializeIVP_Poly( pCollideHeader );
					break;

				case ivp_compat::COLLIDE_MOPP:
					Log_Warning( LOG_VJolt, "Unsupported solid type COLLIDE_MOPP on solid %d. Skipping...\n", i );
					break;
	
				case ivp_compat::COLLIDE_BALL:
					Log_Warning( LOG_VJolt, "Unsupported solid type COLLIDE_BALL on solid %d. Skipping...\n", i );
					break;

				case ivp_compat::COLLIDE_VIRTUAL:
					Log_Warning( LOG_VJolt, "Unsupported solid type COLLIDE_VIRTUAL on solid %d. Skipping...\n", i );
					break;

				default:
					Log_Warning( LOG_VJolt, "Unsupported solid type 0x%x on solid %d. Skipping...\n", (int)pCollideHeader->modelType, i );
					break;
			}
		}
		else
		{
			// This must be a legacy style .phy where it is just a dumped compact surface.
			// Some props in shipping HL2 still use this format, as they have a .phy, even after their
			// .qc had the $collisionmodel removed, as they didn't get the stale .phy in the game files deleted.

			const ivp_compat::compactsurface_t *pCompactSurface = reinterpret_cast<const ivp_compat::compactsurface_t *>( pCursor );
			const int legacyModelType = pCompactSurface->dummy[2];
			switch ( legacyModelType )
			{
				case ivp_compat::IVP_COMPACT_SURFACE_SUPER_LEGACY:
				case ivp_compat::IVP_COMPACT_SURFACE_ID:
				case ivp_compat::IVP_COMPACT_SURFACE_ID_SWAPPED:
					pOutput->solids[i] = DeserializeIVP_Poly( pCompactSurface );
					break;

				case ivp_compat::IVP_COMPACT_MOPP_ID:
					Log_Warning( LOG_VJolt, "Unsupported legacy solid type IVP_COMPACT_MOPP_ID on solid %d. Skipping...\n", i );
					break;

				default:
					Log_Warning( LOG_VJolt, "Unsupported legacy solid type 0x%x on solid %d. Skipping...\n", legacyModelType, i);
					break;
			}
		}

		pCursor += solidSize;
	}

	// The rest of the buffer is KV.
	const int keyValuesSize = size - ( uintp( pCursor ) - uintp( pBuffer ) );

	pOutput->pKeyValues = new char[ keyValuesSize + 1 ];
	V_memcpy( pOutput->pKeyValues, pCursor, keyValuesSize );
	pOutput->pKeyValues[ keyValuesSize ] = '\0';

	pOutput->descSize = keyValuesSize;
	pOutput->isPacked = false;
#ifdef GAME_ASW_OR_NEWER
	pOutput->pUserData = nullptr;
#endif

}

void JoltPhysicsCollision::VCollideUnload( vcollide_t *pVCollide )
{
	VCollideFreeUserData( pVCollide );
	for ( int i = 0; i < pVCollide->solidCount; i++ )
		delete pVCollide->solids[ i ]->ToShape();

	delete[] pVCollide->solids;
	delete[] pVCollide->pKeyValues;
	V_memset( pVCollide, 0, sizeof( *pVCollide ) );
}

//-------------------------------------------------------------------------------------------------

IVPhysicsKeyParser *JoltPhysicsCollision::VPhysicsKeyParserCreate( const char *pKeyData )
{
	return CreateVPhysicsKeyParser( pKeyData, false );
}

IVPhysicsKeyParser *JoltPhysicsCollision::VPhysicsKeyParserCreate( vcollide_t *pVCollide )
{
	return CreateVPhysicsKeyParser( pVCollide->pKeyValues, pVCollide->isPacked );
}

void JoltPhysicsCollision::VPhysicsKeyParserDestroy( IVPhysicsKeyParser *pParser )
{
	DestroyVPhysicsKeyParser( pParser );
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsCollision::CreateDebugMesh( CPhysCollide const *pCollisionModel, Vector **outVerts )
{
	const JPH::Shape *pShape = pCollisionModel->ToShape();
	JPH::Shape::VisitedShapes visitedShapes;
	JPH::Shape::Stats stats = pShape->GetStatsRecursive( visitedShapes );

	const int nMaxTriCount = int( stats.mNumTriangles );
	const int nRequestCount = Max( nMaxTriCount, JPH::Shape::cGetTrianglesMinTrianglesRequested );

	const int nRequestVertCount = nRequestCount * 3;

	Vector *pVerts = new Vector[ nRequestVertCount ];

	JPH::AllHitCollisionCollector<JPH::TransformedShapeCollector> collector;
	JPH::ShapeFilter filter;
	pShape->CollectTransformedShapes( JPH::AABox::sBiggest(), pShape->GetCenterOfMass() * JoltToSource::Factor, JPH::Quat::sIdentity(), JPH::Vec3::sReplicate( JoltToSource::Factor ), JPH::SubShapeIDCreator(), collector, filter );

	int nAccumTris = 0;
	for ( auto &shape : collector.mHits )
	{
		JPH::Shape::GetTrianglesContext ctx;
		shape.GetTrianglesStart( ctx, JPH::AABox::sBiggest(), JPH::Vec3::sZero() );
		for ( ;; )
		{
			int nSubShapeTriCount = shape.GetTrianglesNext( ctx, nRequestCount, reinterpret_cast<JPH::Float3*>( &pVerts[ nAccumTris * 3 ] ), nullptr /* materials */);
			if ( nSubShapeTriCount == 0 || nAccumTris + nSubShapeTriCount > nMaxTriCount )
				break;

			nAccumTris += nSubShapeTriCount;
		}
	}

	// Swap the winding of the triangles to match original VPhysics behaviour.
	for ( int i = 0; i < nAccumTris; i++ )
		std::swap( pVerts[ ( i * 3 ) + 0 ], pVerts[ ( i * 3 ) + 2 ] );

	*outVerts = pVerts;
	return nAccumTris * 3;
}

void JoltPhysicsCollision::DestroyDebugMesh( int vertCount, Vector *outVerts )
{
	delete[] outVerts;
}

//-------------------------------------------------------------------------------------------------

ICollisionQuery *JoltPhysicsCollision::CreateQueryModel( CPhysCollide *pCollide )
{
	return new JoltCollisionQuery( pCollide->ToShape() );
}

void JoltPhysicsCollision::DestroyQueryModel( ICollisionQuery *pQuery )
{
	delete pQuery;
}

//-------------------------------------------------------------------------------------------------

IPhysicsCollision *JoltPhysicsCollision::ThreadContextCreate()
{
	return this;
}

void JoltPhysicsCollision::ThreadContextDestroy( IPhysicsCollision *pThreadContex )
{
	// Does nothing in VPhysics.
}

//-------------------------------------------------------------------------------------------------

CPhysCollide *JoltPhysicsCollision::CreateVirtualMesh( const virtualmeshparams_t &params )
{
	IVirtualMeshEvent *event = params.pMeshEventHandler;

	virtualmeshlist_t meshList;

	event->GetVirtualMesh( params.userData, &meshList );

	JPH::VertexList vertexList;
	vertexList.resize( meshList.vertexCount );
	for ( int i = 0; i < meshList.vertexCount; ++i )
		vertexList[i] = SourceToJolt::DistanceFloat3( meshList.pVerts[i] );

	JPH::IndexedTriangleList indexedTriangleList;
	indexedTriangleList.resize( meshList.indexCount * 2 );

	for ( int i = 0; i < meshList.triangleCount; ++i )
	{
		// Add both windings to make this two-faced.
		// Probably doesn't matter too much but matches what used to happen.
		indexedTriangleList[i*2+0].mIdx[0] = meshList.indices[i*3+0];
		indexedTriangleList[i*2+0].mIdx[1] = meshList.indices[i*3+1];
		indexedTriangleList[i*2+0].mIdx[2] = meshList.indices[i*3+2];

		indexedTriangleList[i*2+1].mIdx[2] = meshList.indices[i*3+0];
		indexedTriangleList[i*2+1].mIdx[1] = meshList.indices[i*3+1];
		indexedTriangleList[i*2+1].mIdx[0] = meshList.indices[i*3+2];
	}

	JPH::MeshShapeSettings settings( vertexList, indexedTriangleList );

	return ShapeSettingsToPhysCollide( settings );
}

bool JoltPhysicsCollision::SupportsVirtualMesh()
{
	return true;
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsCollision::GetBBoxCacheSize( int *pCachedSize, int *pCachedCount )
{
	// Josh: We don't use a bbox cache as we have box shapes directly,
	// and this is only used for debug stats.
	*pCachedSize = 0;
	*pCachedCount = 0;
	return true;
}

//-------------------------------------------------------------------------------------------------

CPolyhedron *JoltPhysicsCollision::PolyhedronFromConvex( CPhysConvex * const pConvex, bool bUseTempPolyhedron )
{
	// This is vile
	Log_Stub( LOG_VJolt );
	return nullptr;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsCollision::OutputDebugInfo( const CPhysCollide *pCollide )
{
	Log_Stub( LOG_VJolt );
}

unsigned int JoltPhysicsCollision::ReadStat( int statID )
{
	// Josh:
	// This always returns 0 in VPhysics.
	// It was used by the HL2 Beta physgun at one point for...
	// something...
	return 0;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsCollision::CollideGetRadius( const CPhysCollide *pCollide )
{
	return pCollide->ToShape()->GetInnerRadius();
}

//-------------------------------------------------------------------------------------------------

void *JoltPhysicsCollision::VCollideAllocUserData( vcollide_t *pVCollide, size_t userDataSize )
{
#ifdef GAME_ASW_OR_NEWER
	VCollideFreeUserData( pVCollide );

	if ( userDataSize )
		pVCollide->pUserData = malloc( userDataSize );

	return pVCollide->pUserData;
#else
	return nullptr;
#endif
}

void JoltPhysicsCollision::VCollideFreeUserData( vcollide_t *pVCollide )
{
#ifdef GAME_ASW_OR_NEWER
	if ( pVCollide->pUserData )
	{
		free( pVCollide->pUserData );
		pVCollide->pUserData = nullptr;
	}
#endif
}

void JoltPhysicsCollision::VCollideCheck( vcollide_t *pVCollide, const char *pName )
{
	// Josh:
	// A thing to spew warnings about non-optimal solids in IVP.
	// Entirely useless for us.
}

bool JoltPhysicsCollision::TraceBoxAA( const Ray_t &ray, const CPhysCollide *pCollide, trace_t *ptr )
{
	TraceBox( ray, pCollide, vec3_origin, vec3_angle, ptr );
	return true;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsCollision::DuplicateAndScale( vcollide_t *pOut, const vcollide_t *pIn, float flScale )
{
	CPhysCollide **pSolids = new CPhysCollide * [pIn->solidCount];
	for ( unsigned short i = 0; i < pIn->solidCount; i++ )
	{
		const JPH::Shape* pShape = pIn->solids[i]->ToShape();

		pSolids[i] = CPhysCollide::FromShape( ToDanglingRef( pShape->ScaleShape( JPH::Vec3::sReplicate( flScale ) ).Get() ) );
	}

	char *pKeyValues = new char[ pIn->descSize ];
	V_memcpy( pKeyValues, pIn->pKeyValues, pIn->descSize );

	*pOut = vcollide_t
	{
		.solidCount = pIn->solidCount,
		.isPacked   = pIn->isPacked,
		.descSize   = pIn->descSize,
		.solids     = pSolids,
		.pKeyValues = pKeyValues,
#ifdef GAME_ASW_OR_NEWER
		.pUserData  = nullptr,
#endif
	};
}

//-------------------------------------------------------------------------------------------------

const JPH::Shape *CreateCOMOverrideShape( const JPH::Shape* pShape, JPH::Vec3Arg comOverride )
{
	JPH::Vec3 comOffset = comOverride - pShape->GetCenterOfMass();

	if ( comOffset.IsNearZero() )
		return pShape;

	JPH::OffsetCenterOfMassShapeSettings settings( comOffset, pShape );
	return ShapeSettingsToShape< JPH::OffsetCenterOfMassShape >( settings );
}
