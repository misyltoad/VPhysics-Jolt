//=================================================================================================
//
// Jolt Debug Renderer Implementation
//
//=================================================================================================

#include "cbase.h"

#ifdef JPH_DEBUG_RENDERER

#include <Jolt/Renderer/DebugRenderer.h>

#include "materialsystem/imaterialsystem.h"
#include "materialsystem/imesh.h"

#include "vjolt_debugrender.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// Slart: Not sure if this is still relevant,
// it's the amount of time a debugoverlay element should stay on-screen
#define ONE_SINGLE_FRAME 0.0f

#define JOLT_VERTEX_BUFFER_NAME		"Jolt Debug Renderer Vertices"
#define JOLT_INDEX_BUFFER_NAME		"Jolt Debug Renderer Indices"

static ConVar vjolt_debugrender( "vjolt_debugrender", "0", FCVAR_CHEAT );
#ifndef VJOLT_USE_PHYSICS_DEBUG_OVERLAY
static ConVar vjolt_debugrender_picture_in_picture( "vjolt_debugrender_picture_in_picture", "1" );
static ConVar vjolt_debugrender_clear_rt( "vjolt_debugrender_clear_rt", "1" );
static ConVar vjolt_debugrender_clear_depth( "vjolt_debugrender_clear_depth", "1" );
static ConVar vjolt_debugrender_wireframe( "vjolt_debugrender_wireframe", "0" );
static ConVar vjolt_debugrender_color_mode( "vjolt_debugrender_color_mode", "instance", 0, "One of instance, shape_type, motion_type, sleep, island, material." );
#endif

//-------------------------------------------------------------------------------------------------

JoltPhysicsDebugRenderer::JoltPhysicsDebugRenderer()
{
	DebugRenderer::Initialize();
}

JoltPhysicsDebugRenderer::~JoltPhysicsDebugRenderer()
{
}

void JoltPhysicsDebugRenderer::DrawLine( JPH::Vec3Arg inFrom, JPH::Vec3Arg inTo, JPH::ColorArg inColor )
{
	Vector v1 = JoltToSource::Distance( inFrom );
	Vector v2 = JoltToSource::Distance( inTo );

#ifdef VJOLT_USE_PHYSICS_DEBUG_OVERLAY
	GetDebugOverlay()->AddLineOverlay( v1, v2, inColor.r, inColor.g, inColor.b, true, ONE_SINGLE_FRAME );
#else
	GetDebugOverlay()->AddLineOverlay( v1, v2, inColor.r, inColor.g, inColor.b, inColor.a, true, 0.2f );
#endif
}

void JoltPhysicsDebugRenderer::DrawTriangle( JPH::Vec3Arg inV1, JPH::Vec3Arg inV2, JPH::Vec3Arg inV3, JPH::ColorArg inColor, ECastShadow inCastShadow )
{
	//DrawTriangle_Internal( JPH::Float3( inV1.GetX(), inV1.GetY(), inV1.GetZ() ), JPH::Float3( inV2.GetX(), inV2.GetY(), inV2.GetZ() ), JPH::Float3( inV3.GetX(), inV3.GetY(), inV3.GetZ() ), inColor );
}

JoltPhysicsDebugRenderer::Batch JoltPhysicsDebugRenderer::CreateTriangleBatch( const Triangle* inTriangles, int inTriangleCount )
{
#ifndef VJOLT_USE_PHYSICS_DEBUG_OVERLAY
	const Vertex* inVertices = reinterpret_cast<const Vertex*>( inTriangles );
	int inVertexCount = inTriangleCount * 3;

	CMatRenderContextPtr pRenderContext( g_pMaterialSystem );

	constexpr VertexFormat_t fmt = VERTEX_POSITION | VERTEX_NORMAL | VERTEX_COLOR | VERTEX_TEXCOORD_SIZE(0, 2);

	IMesh* pMesh = pRenderContext->CreateStaticMesh( fmt, JOLT_VERTEX_BUFFER_NAME );

	CMeshBuilder meshBuilder;
	meshBuilder.Begin( pMesh, MATERIAL_TRIANGLES, inTriangleCount );
	{
		for (int i = 0; i < inVertexCount; ++i)
		{
			meshBuilder.Position3f  ( inVertices[i].mPosition.x * JoltToSource::Factor, inVertices[i].mPosition.y * JoltToSource::Factor, inVertices[i].mPosition.z * JoltToSource::Factor );
			meshBuilder.Normal3f    ( inVertices[i].mNormal.x, inVertices[i].mNormal.y, inVertices[i].mNormal.z );
			meshBuilder.TexCoord2f  ( 0, inVertices[i].mUV.x, inVertices[i].mUV.y );
			meshBuilder.Color4Packed( inVertices[i].mColor.mU32 );
			meshBuilder.AdvanceVertex();
		}
	}
	meshBuilder.End();

	return new BatchImpl( pMesh );
#else
	return nullptr;
#endif
}

JoltPhysicsDebugRenderer::Batch JoltPhysicsDebugRenderer::CreateTriangleBatch( const Vertex* inVertices, int inVertexCount, const uint32* inIndices, int inIndexCount )
{
#ifndef VJOLT_USE_PHYSICS_DEBUG_OVERLAY
	CMatRenderContextPtr pRenderContext( g_pMaterialSystem );

	constexpr VertexFormat_t fmt = VERTEX_POSITION | VERTEX_NORMAL | VERTEX_COLOR | VERTEX_TEXCOORD_SIZE(0, 2);

	IMesh* pMesh = pRenderContext->CreateStaticMesh( fmt, JOLT_VERTEX_BUFFER_NAME );

	CMeshBuilder meshBuilder;
	meshBuilder.Begin( pMesh, MATERIAL_TRIANGLES, inVertexCount, inIndexCount );
	{
		for (int i = 0; i < inVertexCount; ++i)
		{
			meshBuilder.Position3f  ( inVertices[i].mPosition.x * JoltToSource::Factor, inVertices[i].mPosition.y * JoltToSource::Factor, inVertices[i].mPosition.z * JoltToSource::Factor );
			meshBuilder.Normal3f    ( inVertices[i].mNormal.x, inVertices[i].mNormal.y, inVertices[i].mNormal.z );
			meshBuilder.TexCoord2f  ( 0, inVertices[i].mUV.x, inVertices[i].mUV.y );
			meshBuilder.Color4Packed( inVertices[i].mColor.mU32 );
			meshBuilder.AdvanceVertex();
		}

		for (int i = 0; i < inIndexCount; ++i)
		{
			meshBuilder.Index( static_cast<unsigned short>( inIndices[i] ) );
			meshBuilder.AdvanceIndex();
		}
	}
	meshBuilder.End();

	return new BatchImpl( pMesh );
#else
	return nullptr;
#endif
}

#ifndef VJOLT_USE_PHYSICS_DEBUG_OVERLAY
MaterialCullMode_t ConvertCullMode( JPH::DebugRenderer::ECullMode mode )
{
	switch (mode)
	{
	case JPH::DebugRenderer::ECullMode::CullFrontFace:
		return MATERIAL_CULLMODE_CCW;
	case JPH::DebugRenderer::ECullMode::CullBackFace:
		return MATERIAL_CULLMODE_CW;
	default: // case JPH::DebugRenderer::ECullMode::Off:
		return MATERIAL_CULLMODE_NONE;
	}
}
#endif

void JoltPhysicsDebugRenderer::DrawGeometry( JPH::Mat44Arg inModelMatrix, const JPH::AABox& inWorldSpaceBounds, float inLODScaleSq, JPH::ColorArg inModelColor, const GeometryRef& inGeometry, ECullMode inCullMode, ECastShadow inCastShadow, EDrawMode inDrawMode )
{
#ifndef VJOLT_USE_PHYSICS_DEBUG_OVERLAY
	matrix3x4_t sourceMatrix = JoltToSource::Matrix( inModelMatrix );

	const BatchImpl* batch = reinterpret_cast< const BatchImpl * >( inGeometry->mLODs[0].mTriangleBatch.GetPtr() );

	color32 clearColor = { .r = 20, .g = 20, .b = 20, .a = 255 };

	color32 modulateColor = JoltToSource::Color( inModelColor ).ToColor32();

	bool bForceWireFrame = vjolt_debugrender_wireframe.GetInt() == 2;

	DebugOverlayMeshDesc_t desc =
	{
		.pMesh				= batch->GetMesh(),
		.matTransform		= sourceMatrix,
		.flDuration			= -1.0f,
		.bIgnoreZ			= false,
		.bWireframe			= inDrawMode == EDrawMode::Wireframe || bForceWireFrame,
		.bClearRT			= vjolt_debugrender_clear_rt.GetBool(),
		.bClearDepth		= vjolt_debugrender_clear_depth.GetBool(),
		.colClearColor		= clearColor,
		.colModulateColor	= modulateColor,
		.eCullMode			= ConvertCullMode( inCullMode ),
		.bPip				= vjolt_debugrender_picture_in_picture.GetBool()
	};

	GetDebugOverlay()->DrawMesh( desc );
#endif
}

void JoltPhysicsDebugRenderer::DrawText3D( JPH::Vec3Arg inPosition, const std::string_view &inString, JPH::ColorArg inColor, float inHeight )
{
	// Josh:
	// Doing a copy of 1024, the max size allowed by a debug overlay
	// because AddTextOverlayRGB takes in a c_str and a string view
	// is not necessarily null terminated.
	char text[1024];
	V_strncpy( text, inString.data(), sizeof( text ) );

	Vector origin = JoltToSource::Distance( inPosition );

	GetDebugOverlay()->AddTextOverlayRGB( origin, 0, 0.5f, inColor.r, inColor.g, inColor.b, inColor.a, "%s", text );
}

void JoltPhysicsDebugRenderer::DrawJoltTVText()
{
	GetDebugOverlay()->AddScreenTextOverlay( 0.75f, 0.55f, 0.5f, 255, 0, 255, 255, "Live Jolt Reaction" ); // "Jolt TV"
}

//-------------------------------------------------------------------------------------------------

#ifndef VJOLT_USE_PHYSICS_DEBUG_OVERLAY

static JPH::BodyManager::EShapeColor GetColorMode()
{
	const char *pszString = vjolt_debugrender_color_mode.GetString();

	if ( !V_stricmp( pszString, "instance" ) )
		return JPH::BodyManager::EShapeColor::InstanceColor;
	else if ( !V_stricmp( pszString, "shape_type" ) )
		return JPH::BodyManager::EShapeColor::ShapeTypeColor;
	else if ( !V_stricmp( pszString, "motion_type" ) )
		return JPH::BodyManager::EShapeColor::MotionTypeColor;
	else if ( !V_stricmp( pszString, "sleep" ) )
		return JPH::BodyManager::EShapeColor::SleepColor;
	else if ( !V_stricmp( pszString, "island" ) )
		return JPH::BodyManager::EShapeColor::IslandColor;
	else if ( !V_stricmp( pszString, "material" ) )
		return JPH::BodyManager::EShapeColor::MaterialColor;

	Log_Msg( LOG_VJolt, "Unknown color mode: %s\n", pszString );
	return JPH::BodyManager::EShapeColor::InstanceColor;
}

#endif

void JoltPhysicsDebugRenderer::RenderPhysicsSystem( JPH::PhysicsSystem &physicsSystem )
{
	if ( !GetDebugOverlay() || !vjolt_debugrender.GetBool())
		return;

#ifndef VJOLT_USE_PHYSICS_DEBUG_OVERLAY
	const bool wireframe = vjolt_debugrender_wireframe.GetBool();

	JPH::BodyManager::DrawSettings drawSettings
	{
		.mDrawGetSupportFunction	= false,			///< Draw the GetSupport() function, used for convex collision detection	
		.mDrawSupportDirection		= false,			///< When drawing the support function, also draw which direction mapped to a specific support point
		.mDrawGetSupportingFace		= false,			///< Draw the faces that were found colliding during collision detection
		.mDrawShape					= true,				///< Draw the shapes of all bodies
		.mDrawShapeWireframe		= wireframe,		///< When mDrawShape is true and this is true, the shapes will be drawn in wireframe instead of solid.
		.mDrawShapeColor			= GetColorMode(),	///< Coloring scheme to use for shapes
		.mDrawBoundingBox			= false,			///< Draw a bounding box per body
		.mDrawCenterOfMassTransform	= false,			///< Draw the center of mass for each body
		.mDrawWorldTransform		= false,			///< Draw the world transform (which can be different than the center of mass) for each body
		.mDrawVelocity				= false,			///< Draw the velocity vector for each body
		.mDrawMassAndInertia		= false,			///< Draw the mass and inertia (as the box equivalent) for each body
		.mDrawSleepStats			= false,			///< Draw stats regarding the sleeping algorithm of each body
	};
#else
	JPH::BodyManager::DrawSettings drawSettings
	{
		.mDrawGetSupportFunction	= false,			///< Draw the GetSupport() function, used for convex collision detection	
		.mDrawSupportDirection		= false,			///< When drawing the support function, also draw which direction mapped to a specific support point
		.mDrawGetSupportingFace		= false,			///< Draw the faces that were found colliding during collision detection
		.mDrawShape					= false,			///< Draw the shapes of all bodies
		.mDrawShapeWireframe		= false,			///< When mDrawShape is true and this is true, the shapes will be drawn in wireframe instead of solid.
		.mDrawBoundingBox			= true,				///< Draw a bounding box per body
		.mDrawCenterOfMassTransform	= false,			///< Draw the center of mass for each body
		.mDrawWorldTransform		= false,			///< Draw the world transform (which can be different than the center of mass) for each body
		.mDrawVelocity				= true,				///< Draw the velocity vector for each body
		.mDrawMassAndInertia		= false,			///< Draw the mass and inertia (as the box equivalent) for each body
		.mDrawSleepStats			= false,			///< Draw stats regarding the sleeping algorithm of each body
	};
#endif

	physicsSystem.DrawBodies( drawSettings, this );

#ifndef VJOLT_USE_PHYSICS_DEBUG_OVERLAY
	// :frog:
	if ( vjolt_debugrender_picture_in_picture.GetBool() )
		DrawJoltTVText();
#endif
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsDebugRenderer &JoltPhysicsDebugRenderer::GetInstance()
{
	static JoltPhysicsDebugRenderer s_DebugRenderer;
	return s_DebugRenderer;
}

IVJoltDebugOverlay *JoltPhysicsDebugRenderer::GetDebugOverlay()
{
	return JoltPhysicsInterface::GetInstance().GetDebugOverlay();
}

//-------------------------------------------------------------------------------------------------

#endif // JPH_DEBUG_RENDERER
