//=================================================================================================
//
// Trace Logic
//
// Split out from vjolt_collide.cpp due to being a can of worms
//
//=================================================================================================

#include "cbase.h"

#include "coordsize.h" // DIST_EPSILON

#include "vjolt_debugrender.h"
#include "vjolt_util.h"

#include "vjolt_collide.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

namespace VJoltTrace
{

static constexpr float kCollisionTolerance = 1.0e-3f;
static constexpr float kCharacterPadding = 0.02f;

// Also in vjolt_collide.cpp, should unify or just remove entirely
static constexpr float kMaxConvexRadius = JPH::cDefaultConvexRadius;

static ConVar vjolt_trace_debug( "vjolt_trace_debug", "0", FCVAR_CHEAT );
static ConVar vjolt_trace_debug_castray( "vjolt_trace_debug_castray", "0", FCVAR_CHEAT );
static ConVar vjolt_trace_debug_collidepoint( "vjolt_trace_debug_collidepoint", "0", FCVAR_CHEAT );
static ConVar vjolt_trace_debug_castbox( "vjolt_trace_debug_castbox", "0", FCVAR_CHEAT );
static ConVar vjolt_trace_debug_collidebox( "vjolt_trace_debug_collidebox", "0", FCVAR_CHEAT );

// Josh: Enables a hack to make portals work. For some reason when we enable colliding with
// backfaces, the player gets easily stuck in all sorts of things!
// Slart and I have not been able to determine the root cause of this problem and have tried for a long time...
//
// Slart: Portal 2 probably passes in a bad winding order in the polyhedron or something, dunno if it affects Portal 1
static ConVar vjolt_trace_portal_hack( "vjolt_trace_portal_hack", "0", FCVAR_NONE );

//-------------------------------------------------------------------------------------------------
//
// Collectors and helpers
//
//-------------------------------------------------------------------------------------------------

#ifdef JPH_DEBUG_RENDERER

static void PrintTrace( const trace_t *tr, JoltPhysicsDebugRenderer &debugRenderer )
{
	Vector textPos = tr->endpos;
	textPos.z += 4.0f;

	constexpr float dur = 0.1f;
	constexpr int r = 255;
	constexpr int g = 255;
	constexpr int b = 255;
	constexpr int a = 255;

	IVJoltDebugOverlay *pDebugOverlay = debugRenderer.GetDebugOverlay();

	pDebugOverlay->AddTextOverlayRGB( textPos, 0, dur, r,g,b,a, "startpos   : [ %g %g %g ]",	tr->startpos.x, tr->startpos.y, tr->startpos.z );
	pDebugOverlay->AddTextOverlayRGB( textPos, 1, dur, r,g,b,a, "endpos     : [ %g %g %g ]",	tr->endpos.x, tr->endpos.y, tr->endpos.z );
	pDebugOverlay->AddTextOverlayRGB( textPos, 2, dur, r,g,b,a, "fraction   : %g",				tr->fraction );
	pDebugOverlay->AddTextOverlayRGB( textPos, 3, dur, r,g,b,a, "contents   : %d",				tr->contents );
	pDebugOverlay->AddTextOverlayRGB( textPos, 4, dur, r,g,b,a, "allsolid   : %d",				(int)tr->allsolid );
	pDebugOverlay->AddTextOverlayRGB( textPos, 5, dur, r,g,b,a, "startsolid : %d",				(int)tr->startsolid );
	pDebugOverlay->AddTextOverlayRGB( textPos, 6, dur, r,g,b,a, "normal     : [ %g %g %g ]",	tr->plane.normal.x, tr->plane.normal.y, tr->plane.normal.z );
	pDebugOverlay->AddTextOverlayRGB( textPos, 7, dur, r,g,b,a, "dist       : %g",				tr->plane.dist );
}

#endif

//
// Collector for VJolt_CastRay
// Returns the closest hit within the contents mask
//
class ContentsCollector_CastRay final : public JPH::CastRayCollector
{
public:
	ContentsCollector_CastRay( const JPH::Shape *pShape, uint32 contentsMask, IConvexInfo *pConvexInfo )
		: m_pShape( pShape ), m_ContentsMask( contentsMask ), m_pConvexInfo( pConvexInfo ) {}

	void AddHit( const JPH::RayCastResult &inResult ) override
	{
		// Test if this collision is closer than the previous one
		const float theirEarlyOut = inResult.GetEarlyOutFraction();
		const float ourEarlyOut = GetEarlyOutFraction();
		if ( !m_DidHit || theirEarlyOut < ourEarlyOut )
		{
			const uint32 gameData = static_cast<uint32>( m_pShape->GetSubShapeUserData( inResult.mSubShapeID2 ) );
			const uint32 contents = m_pConvexInfo ? m_pConvexInfo->GetContents( gameData ) : CONTENTS_SOLID;

			if ( contents & m_ContentsMask )
			{
				// Update our early out fraction
				UpdateEarlyOutFraction( theirEarlyOut );

				// Store hit info locally
				m_Fraction = inResult.mFraction;
				m_SubShapeID = inResult.mSubShapeID2;
				m_ResultContents = contents;

				m_DidHit = true;
			}
		}
	}

private:
	// Inputs
	const JPH::Shape *	m_pShape = nullptr;
	uint32				m_ContentsMask = 0;
	IConvexInfo *		m_pConvexInfo = nullptr;

public:
	// Outputs (only use if m_DidHit is true)
	float				m_Fraction = 1.0f;			// Use this instead of GetEarlyOutFraction
	JPH::SubShapeID		m_SubShapeID;				// Subshape hit
	uint32				m_ResultContents = 0;		// Contents of hit subshape

	bool				m_DidHit = false;
};

//
// Collector for VJolt_CollidePoint
// Returns the first hit within the contents mask
//
class ContentsCollector_CollidePoint final : public JPH::CollidePointCollector
{
public:
	ContentsCollector_CollidePoint( const JPH::Shape *pShape, uint32 contentsMask, IConvexInfo *pConvexInfo )
		: m_pShape( pShape ), m_ContentsMask( contentsMask ), m_pConvexInfo( pConvexInfo ) {}

	void AddHit( const JPH::CollidePointResult &inResult ) override
	{
		// Return the first hit only
		if ( m_DidHit )
			return;

		const uint32 gameData = static_cast<uint32>( m_pShape->GetSubShapeUserData( inResult.mSubShapeID2 ) );
		const uint32 contents = m_pConvexInfo ? m_pConvexInfo->GetContents( gameData ) : CONTENTS_SOLID;

		if ( contents & m_ContentsMask )
		{
			// Store hit info locally
			m_SubShapeID = inResult.mSubShapeID2;
			m_ResultContents = contents;

			m_DidHit = true;
		}
	}

private:
	// Inputs
	const JPH::Shape *	m_pShape = nullptr;
	uint32				m_ContentsMask = 0;
	IConvexInfo *		m_pConvexInfo = nullptr;

public:
	// Outputs (only use if m_DidHit is true)
	JPH::SubShapeID		m_SubShapeID;				// Subshape hit
	uint32				m_ResultContents = 0;		// Contents of hit subshape

	bool				m_DidHit = false;
};

//
// Filter for generic shape casts or collides
// Ensures that objects which do not fit within the contents mask are excluded
//
class ContentsFilter_Shape final : public JPH::ShapeFilter
{
public:
	ContentsFilter_Shape( const JPH::Shape *pShape, uint32 contentsMask, IConvexInfo *pConvexInfo )
		: m_pShape( pShape ), m_ContentsMask( contentsMask ), m_pConvexInfo( pConvexInfo ) {}

	bool ShouldCollide( const JPH::Shape *inShape2, const JPH::SubShapeID& inSubShapeID2 ) const override
	{
		const uint32 gameData = static_cast<uint32>( inShape2->GetSubShapeUserData( inSubShapeID2 ) );
		const uint32 contents = m_pConvexInfo ? m_pConvexInfo->GetContents( gameData ) : CONTENTS_SOLID;

		return !!( contents & m_ContentsMask );
	}

	bool ShouldCollide( const JPH::Shape *inShape1, const JPH::SubShapeID &inSubShapeIDOfShape1, const JPH::Shape *inShape2, const JPH::SubShapeID &inSubShapeIDOfShape2 ) const override
	{
		return ShouldCollide( inShape2, inSubShapeIDOfShape2 );
	}

public:
	// Input
	const JPH::Shape *	m_pShape = nullptr;
	uint32				m_ContentsMask = 0;
	IConvexInfo *		m_pConvexInfo = nullptr;
};

//
// Collector for generic shape casts
//
class ContentsCollector_CastShape final : public JPH::CastShapeCollector
{
public:
	ContentsCollector_CastShape( const JPH::Shape *pShape, uint32 contentsMask, IConvexInfo *pConvexInfo )
		: m_pShape( pShape ), m_ContentsMask( contentsMask ), m_pConvexInfo( pConvexInfo ) {}

	void AddHit( const JPH::ShapeCastResult &inResult ) override
	{
		const uint32 gameData = static_cast<uint32>( m_pShape->GetSubShapeUserData( inResult.mSubShapeID2 ) );
		const uint32 contents = m_pConvexInfo ? m_pConvexInfo->GetContents( gameData ) : CONTENTS_SOLID;

		// Ensure that the contents filter was used
		VJoltAssert( contents & m_ContentsMask );

		// Test if this collision is closer than the previous one
		const float theirEarlyOut = inResult.GetEarlyOutFraction();
		const float ourEarlyOut = GetEarlyOutFraction();
		if ( !m_DidHit || theirEarlyOut < ourEarlyOut )
		{
			// Update our early out fraction
			UpdateEarlyOutFraction( theirEarlyOut );

			m_Fraction = inResult.mFraction;
			m_ResultContents = contents;
			m_SubShapeID = inResult.mSubShapeID2;
			m_ContactPoint = inResult.mContactPointOn2;
			m_PenetrationAxis = inResult.mPenetrationAxis;
			m_PenetrationDepth = inResult.mPenetrationDepth;

			m_DidHit = true;
			m_HitBackFace = inResult.mIsBackFaceHit;
		}
	}

private:
	// Input
	const JPH::Shape *	m_pShape = nullptr;
	uint32				m_ContentsMask = 0;
	IConvexInfo *		m_pConvexInfo = nullptr;

public:
	// Outputs (only use if m_DidHit is true)
	float				m_Fraction = 1.0f;
	uint32				m_ResultContents = 0;			// Contents of the subshape that we hit
	JPH::SubShapeID		m_SubShapeID;					// SubShapeID that we hit
	JPH::Vec3			m_ContactPoint;					// Point of impact relative to the COM of the object we hit
	JPH::Vec3			m_PenetrationAxis;
	float				m_PenetrationDepth = 0.0f;

	bool				m_DidHit = false;				// Set to true if we hit anything
	bool				m_HitBackFace = false;			// Set to true if the hit was against a backface
};

//
// Collector for generic shape collides
//
class ContentsCollector_CollideShape final : public JPH::CollideShapeCollector
{
public:
	ContentsCollector_CollideShape( const JPH::Shape *pShape, uint32 contentsMask, IConvexInfo *pConvexInfo )
		: m_pShape( pShape ), m_ContentsMask( contentsMask ), m_pConvexInfo( pConvexInfo ) {}

	// Called whenever a hit occurs, for compound objects this can be called multiple times
	void AddHit( const JPH::CollideShapeResult &inResult ) override
	{
		// Get the contents of the subshape that we hit
		const uint32 gameData = static_cast<uint32>( m_pShape->GetSubShapeUserData( inResult.mSubShapeID2 ) );
		const uint32 contents = m_pConvexInfo ? m_pConvexInfo->GetContents( gameData ) : CONTENTS_SOLID;

		VJoltAssert( contents & m_ContentsMask );

		if ( inResult.GetEarlyOutFraction() < GetEarlyOutFraction() )
		{
			UpdateEarlyOutFraction( inResult.GetEarlyOutFraction() );

			m_ResultContents = contents;
			m_SubShapeID = inResult.mSubShapeID2;
			m_ContactPoint = inResult.mContactPointOn2;
			m_PenetrationAxis = inResult.mPenetrationAxis;
			m_PenetrationDepth = inResult.mPenetrationDepth;

			m_DidHit = true;
		}
	}

private:
	// Input
	const JPH::Shape *	m_pShape = nullptr;
	uint32				m_ContentsMask = 0;
	IConvexInfo *		m_pConvexInfo = nullptr;

public:
	// Output, only valid if m_didHit is true
	uint32				m_ResultContents = 0;			// Contents of the subshape that we hit
	JPH::SubShapeID		m_SubShapeID;					// SubShapeID that we hit
	JPH::Vec3			m_ContactPoint;					// Point of impact relative to the COM of the object we hit
	JPH::Vec3			m_PenetrationAxis;
	float				m_PenetrationDepth = 0.0f;

	bool				m_DidHit = false;				// Set to true if we hit anything
};

//
// Contents collector for simple collide operations (no contents)
//
class ContentsCollector_SimpleCollide final : public JPH::CollideShapeCollector
{
public:
	void AddHit( const ResultType &inResult ) override
	{
		intersection = true;
	}

	bool intersection = false;
};

//-------------------------------------------------------------------------------------------------
//
// Tracing functions
//
//-------------------------------------------------------------------------------------------------

//
// Casts a ray against a shape
//
static void CastRay( const Ray_t &ray, uint32 contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace )
{
	const JPH::Shape *pShape = pCollide->ToShape();

	JPH::Vec3 position = SourceToJolt::Distance( collideOrigin );
	JPH::Quat rotation = SourceToJolt::Angle( collideAngles );
	JPH::Mat44 queryTransform = JPH::Mat44::sRotationTranslation( rotation, position + rotation * pShape->GetCenterOfMass() );

	// Create ray
	JPH::RayCast joltRay = { SourceToJolt::Distance( ray.m_Start ), SourceToJolt::Distance( ray.m_Delta ) };
	// Transform the ray by the inverse of the position/rotation
	joltRay = joltRay.Transformed( queryTransform.InversedRotationTranslation() );

	// Set-up the settings
	JPH::RayCastSettings settings;
	settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;
	settings.mTreatConvexAsSolid = true;

	//
	// Hello!
	// This code mimics IVP's behaviour where solids are solid, and the ray is totally clipped when inside.
	// the BSP code (the gold standard) allows traces that start inside of solid objects to pass through backfaces on the convex.
	// BSP sets traces that start inside of objects as startsolid 1, IVP doesn't do this :(
	// So in the future, to mimic the BSP logic, we should make traces that begin inside of objects startsolid and only allsolid
	// if they never leave the object
	//

	// Create our collector and cast away!
	ContentsCollector_CastRay collector( pShape, contentsMask, pConvexInfo );
	pShape->CastRay( joltRay, settings, JPH::SubShapeIDCreator(), collector );

	if ( !collector.m_DidHit )
	{
		VJoltAssert( collector.m_Fraction == 1.0f );
	}

	// Populate pTrace's members
	pTrace->fraction     = collector.m_Fraction;
	pTrace->startpos     = ray.m_Start + ray.m_StartOffset;
	pTrace->endpos       = pTrace->startpos + ( ray.m_Delta * pTrace->fraction );
	pTrace->allsolid     = pTrace->fraction == 0.0f;
	pTrace->startsolid   = pTrace->fraction == 0.0f;

	// If we hit, we fill in the plane and contents too
	if ( collector.m_DidHit )
	{
		JPH::Vec3 normal     = queryTransform.GetRotation() * pShape->GetSurfaceNormal( collector.m_SubShapeID, joltRay.GetPointOnRay( collector.m_Fraction ) );
		pTrace->plane.normal = Vector( normal.GetX(), normal.GetY(), normal.GetZ() );
		pTrace->plane.dist   = DotProduct( pTrace->endpos, pTrace->plane.normal );

		pTrace->contents = collector.m_ResultContents;
	}

#if defined JPH_DEBUG_RENDERER

	// Debug trace visualizing
	IVJoltDebugOverlay *pOverlay = JoltPhysicsInterface::GetInstance().GetDebugOverlay();
	if ( vjolt_trace_debug.GetBool() && vjolt_trace_debug_castray.GetBool() && pOverlay )
	{
		JoltPhysicsDebugRenderer &debugRenderer = JoltPhysicsDebugRenderer::GetInstance();

		JPH::Color rayColor( 255, 0, 0, 255 );

		if ( collector.m_DidHit )
		{
			rayColor.r = 0;
			rayColor.g = 255;

			JPH::Vec3 normal = queryTransform.GetRotation() * pShape->GetSurfaceNormal( collector.m_SubShapeID, joltRay.GetPointOnRay( collector.m_Fraction ) );
			debugRenderer.DrawArrow( queryTransform * ( joltRay.mOrigin + ( joltRay.mDirection * collector.GetEarlyOutFraction() ) ), queryTransform * ( joltRay.mOrigin + joltRay.mDirection * collector.GetEarlyOutFraction() ) + normal, JPH::Color::sRed, 0.3f );
		}

		debugRenderer.DrawArrow( queryTransform * joltRay.mOrigin, queryTransform * ( joltRay.mOrigin + ( joltRay.mDirection * collector.GetEarlyOutFraction() ) ), rayColor, 0.3f );
	}

#endif
}

//
// Collides a point against a shape
//
static void CollidePoint( const Ray_t &ray, uint32 contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace )
{
	const JPH::Shape *pShape = pCollide->ToShape();

	JPH::Vec3 position = SourceToJolt::Distance( collideOrigin );
	JPH::Quat rotation = SourceToJolt::Angle( collideAngles );
	JPH::Mat44 queryTransform = JPH::Mat44::sRotationTranslation( rotation, position + rotation * pShape->GetCenterOfMass() );

	JPH::Vec3 point = queryTransform.InversedRotationTranslation() * SourceToJolt::Distance(ray.m_Start);

	ContentsCollector_CollidePoint collector( pShape, contentsMask, pConvexInfo );
	pShape->CollidePoint( point, JPH::SubShapeIDCreator(), collector );

	// Populate pTrace's members
	pTrace->fraction     = collector.m_DidHit ? 0.0f : 1.0f;
	pTrace->startpos     = ray.m_Start + ray.m_StartOffset;
	pTrace->endpos       = pTrace->startpos;
	pTrace->allsolid     = collector.m_DidHit;
	pTrace->startsolid   = collector.m_DidHit;

	// This will be zero if we didn't hit
	pTrace->contents = collector.m_ResultContents;

#if defined JPH_DEBUG_RENDERER

	// Debug trace visualizing
	IVJoltDebugOverlay *pOverlay = JoltPhysicsInterface::GetInstance().GetDebugOverlay();
	if ( vjolt_trace_debug.GetBool() && vjolt_trace_debug_collidepoint.GetBool() && pOverlay )
	{
		JoltPhysicsDebugRenderer &debugRenderer = JoltPhysicsDebugRenderer::GetInstance();

		JPH::Color rayColor( 0, 255, 0, 0 );

		if ( collector.m_DidHit )
		{
			rayColor.a = 255;
		}

		debugRenderer.DrawMarker( point, rayColor, 0.3f );
	}

#endif
}

static float CalculateSourceFraction( const Vector &rayDelta, float fraction, const Vector &normal )
{
	const float rayLength = rayDelta.Length();
	float invBaseLength = 0.0f;
	Vector rayDir = vec3_origin;
	if ( rayLength )
	{
		rayDir = rayDelta.Normalized();
		invBaseLength = 1.0f / rayLength;
	}

	float hitLength = rayLength * fraction;
	float dot = DotProduct( rayDir, normal );
	if ( dot < 0.0f )
		hitLength += DIST_EPSILON / dot;

	hitLength = Max( hitLength, 0.0f );

	return hitLength * invBaseLength;
}

//
// Casts a box against a shape
//
static void CastBoxVsShape( const Ray_t &ray, uint32 contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace )
{
	const JPH::Shape *pShape = pCollide->ToShape();

	JPH::Vec3 origin = SourceToJolt::Distance( ray.m_Start );			// Origin of the box or ray we're casting
	JPH::Vec3 direction = SourceToJolt::Distance( ray.m_Delta );		// Direction and distance of the cast
	JPH::Vec3 halfExtent = SourceToJolt::Distance( ray.m_Extents );

	JPH::Vec3 position = SourceToJolt::Distance( collideOrigin );		// Position of the object we're casting against
	JPH::Quat rotation = SourceToJolt::Angle( collideAngles );		// Rotation of the object we're casting against
	JPH::Mat44 queryTransform = JPH::Mat44::sRotationTranslation( rotation, position + rotation * pShape->GetCenterOfMass() );

	JPH::BoxShape boxShape( halfExtent, kMaxConvexRadius );
	JPH::ShapeCast shapeCast( &boxShape, JPH::Vec3::sReplicate( 1.0f ), JPH::Mat44::sTranslation( origin ), direction );

	JPH::ShapeCastSettings settings;
	//settings.mBackFaceModeTriangles = JPH::EBackFaceMode::CollideWithBackFaces;
	// Josh: Had to re-enable CollideWithBackFaces to allow triggers for the Portal Environment to work.
	// Come back here if we start getting stuck on things again...
	if ( vjolt_trace_portal_hack.GetBool() )
		settings.mBackFaceModeConvex = JPH::EBackFaceMode::CollideWithBackFaces;
	//settings.mCollisionTolerance = kCollisionTolerance;
	settings.mUseShrunkenShapeAndConvexRadius = true;
	settings.mReturnDeepestPoint = true;

	ContentsFilter_Shape filter( pShape, contentsMask, pConvexInfo );
	ContentsCollector_CastShape collector( pShape, contentsMask, pConvexInfo );
	JPH::CollisionDispatch::sCastShapeVsShapeWorldSpace( shapeCast, settings, pShape, JPH::Vec3::sReplicate( 1.0f ), filter, queryTransform, JPH::SubShapeIDCreator(), JPH::SubShapeIDCreator(), collector );

	if ( collector.m_DidHit )
	{
		JPH::Vec3 normal = -( collector.m_PenetrationAxis.Normalized() );
		pTrace->plane.normal = Vector( normal.GetX(), normal.GetY(), normal.GetZ() );

		pTrace->fraction = CalculateSourceFraction( ray.m_Delta, collector.m_Fraction, pTrace->plane.normal );

		//Log_Msg( LOG_VJolt, "Depth: %g, InitialFraction = %g, NewFraction = %g\n", collector.m_PenetrationDepth, flInitialFraction, pTrace->fraction );

		pTrace->startpos = ray.m_Start + ray.m_StartOffset;
		pTrace->endpos = pTrace->startpos + ( ray.m_Delta * pTrace->fraction );

		pTrace->endpos -= pTrace->plane.normal * collector.m_PenetrationDepth;

		pTrace->plane.dist = DotProduct( pTrace->endpos, pTrace->plane.normal );
		pTrace->contents = collector.m_ResultContents;
			
		// If penetrating more than DIST_EPSILON, consider it an intersection
		//constexpr float PenetrationEpsilon = DIST_EPSILON;
		static constexpr float kMinRequiredPenetration = 0.005f + kCharacterPadding;

		pTrace->allsolid = collector.m_PenetrationDepth > kMinRequiredPenetration && pTrace->fraction == 0.0f;
		pTrace->startsolid = collector.m_PenetrationDepth > kMinRequiredPenetration && pTrace->fraction == 0.0f;
	}
	else
	{
		pTrace->fraction = 1.0f;
		pTrace->startpos = ray.m_Start + ray.m_StartOffset;
		pTrace->endpos = pTrace->startpos + ray.m_Delta;

		// We didn't hit anything, so we must be completely free right?
		pTrace->allsolid = false;
		pTrace->startsolid = false;
	}

#if defined JPH_DEBUG_RENDERER

	// Debug trace visualizing
	IVJoltDebugOverlay *pOverlay = JoltPhysicsInterface::GetInstance().GetDebugOverlay();
	if ( vjolt_trace_debug.GetBool() && vjolt_trace_debug_castbox.GetBool() && pOverlay )
	{
		JoltPhysicsDebugRenderer &debugRenderer = JoltPhysicsDebugRenderer::GetInstance();

		JPH::Color color( 255, 64, 64, 255 );

		if ( collector.m_DidHit )
		{
			color.r = 64;
			color.g = 255;

			JPH::Vec3 hitPos = origin + ( direction * collector.m_Fraction );
			debugRenderer.DrawArrow( hitPos, hitPos + collector.m_PenetrationAxis, JPH::Color::sRed, 0.3f );
		}

		boxShape.Draw( &debugRenderer, queryTransform, JPH::Vec3::sReplicate( 1.0f ), color, false, false );
	}

#endif
}

//
// Collides a box against a shape
//
static void CollideBoxVsShape( const Ray_t &ray, uint32 contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace )
{
	const JPH::Shape *pShape = pCollide->ToShape();

	JPH::Vec3 origin = SourceToJolt::Distance( ray.m_Start );			// Origin of the box or ray we're casting
	JPH::Vec3 direction = SourceToJolt::Distance( ray.m_Delta );		// Direction and distance of the cast
	JPH::Vec3 halfExtent = SourceToJolt::Distance( ray.m_Extents );

	JPH::Vec3 position = SourceToJolt::Distance( collideOrigin );		// Position of the object we're casting against
	JPH::Quat rotation = SourceToJolt::Angle( collideAngles );			// Rotation of the object we're casting against
	JPH::Mat44 queryTransform = JPH::Mat44::sRotationTranslation( rotation, position + rotation * pShape->GetCenterOfMass() );

	JPH::BoxShape boxShape( halfExtent, kMaxConvexRadius );

	JPH::CollideShapeSettings settings;
	//settings.mMaxSeparationDistance = DIST_EPSILON;
	//settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;

	ContentsCollector_CollideShape collector( pShape, contentsMask, pConvexInfo );
	JPH::CollisionDispatch::sCollideShapeVsShape(
		&boxShape, pShape,
		JPH::Vec3::sReplicate( 1.0f ), JPH::Vec3::sReplicate( 1.0f ),
		JPH::Mat44::sIdentity(), queryTransform,
		JPH::SubShapeIDCreator(), JPH::SubShapeIDCreator(),
		settings, collector );

	pTrace->fraction = collector.m_DidHit ? 0.0f : 1.0f;
	pTrace->startpos = ray.m_Start + ray.m_StartOffset;
	pTrace->endpos = pTrace->startpos;
	pTrace->allsolid = collector.m_DidHit;
	pTrace->startsolid = collector.m_DidHit;

	if ( collector.m_DidHit )
	{
		JPH::Vec3 normal = -( ( queryTransform.GetRotation() * collector.m_PenetrationAxis ).Normalized() );
		pTrace->plane.normal = Vector( normal.GetX(), normal.GetY(), normal.GetZ() );
		pTrace->plane.dist = DotProduct( pTrace->endpos, pTrace->plane.normal );

		pTrace->contents = collector.m_ResultContents;
	}

#if defined JPH_DEBUG_RENDERER

	// Debug trace visualizing
	IVJoltDebugOverlay *pOverlay = JoltPhysicsInterface::GetInstance().GetDebugOverlay();
	if ( vjolt_trace_debug.GetBool() && vjolt_trace_debug_collidebox.GetBool() && pOverlay )
	{
		JoltPhysicsDebugRenderer &debugRenderer = JoltPhysicsDebugRenderer::GetInstance();

		JPH::Color color( 255, 64, 64, 255 );

		if ( collector.m_DidHit )
		{
			color.r = 64;
			color.g = 255;
		}

		boxShape.Draw( &debugRenderer, queryTransform, JPH::Vec3::sReplicate( 1.0f ), color, false, false );
	}

#endif
}

//
// Collides a shape against a shape
//
// Slart: This sucks, it could be implemented better in the environment interface using a bool and two physics objects instead... Basically all the code using this wants it to see
// if a shape is inside another shape.
//
static void CollideShapeVsShape( const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace )
{
	ClearTrace( pTrace );

	// Are we sweeping, or just doing a point test?
	bool isSweeping = false;

	if ( !VectorCompare( start, end ) )
	{
		isSweeping = true;

		// Slart: Exactly one piece of code "may" want a sweeping trace, ffs
		Log_Warning( LOG_VJolt, "Attemping an unsupported TraceCollide\n" );
		return;
	}

	// Glue variable
	const Vector &sweepOrigin = start;

	const JPH::Shape *pSweepShape = pSweepCollide->ToShape();
	JPH::Vec3 sweepJoltPosition = SourceToJolt::Distance( sweepOrigin );
	JPH::Quat sweepJoltRotation = SourceToJolt::Angle( sweepAngles );
	JPH::Mat44 sweepTransform = JPH::Mat44::sRotationTranslation( sweepJoltRotation, sweepJoltPosition + sweepJoltRotation * pSweepShape->GetCenterOfMass() );

	const JPH::Shape *pCollideShape = pCollide->ToShape();
	JPH::Vec3 collideJoltPosition = SourceToJolt::Distance( collideOrigin );
	JPH::Quat collideJoltRotation = SourceToJolt::Angle( collideAngles );
	JPH::Mat44 collideTransform = JPH::Mat44::sRotationTranslation( collideJoltRotation, collideJoltPosition + collideJoltRotation * pCollideShape->GetCenterOfMass() );
	//JPH::Mat44 collideTransform = JPH::Mat44::sTranslation( collideJoltPosition );

	if ( !isSweeping )
	{
		JPH::AABox sweepAABB = pSweepShape->GetWorldSpaceBounds( sweepTransform, JPH::Vec3::sReplicate( 1.0f ) );
		JPH::AABox collideAABB = pCollideShape->GetWorldSpaceBounds( collideTransform, JPH::Vec3::sReplicate( 1.0f ) );

		// Debug trace visualizing
		IVJoltDebugOverlay *pOverlay = JoltPhysicsInterface::GetInstance().GetDebugOverlay();
		if ( vjolt_trace_debug.GetBool() && pOverlay && !pTrace->allsolid )
		{
			Vector mins, maxs;

			mins = JoltToSource::Distance( sweepAABB.mMin );
			maxs = JoltToSource::Distance( sweepAABB.mMax );
			pOverlay->AddBoxOverlay( vec3_origin, mins, maxs, vec3_angle, 255, 64, 64, 255, -1.0f );

			mins = JoltToSource::Distance( collideAABB.mMin );
			maxs = JoltToSource::Distance( collideAABB.mMax );
			pOverlay->AddBoxOverlay( vec3_origin, mins, maxs, vec3_angle, 255, 64, 64, 255, -1.0f );
		}

		// Don't bother doing any work if we don't contain the thing to be tested
		if ( !collideAABB.Contains( sweepAABB ) )
		{
			// TODO(Slart): Uncomment this
			//return;
		}

		ContentsCollector_SimpleCollide collector;

		JPH::CollisionDispatch::sCollideShapeVsShape(
			pSweepShape, pCollideShape,
			JPH::Vec3::sReplicate( 1.0f ), JPH::Vec3::sReplicate( 1.0f ),
			sweepTransform, collideTransform,
			JPH::SubShapeIDCreator(), JPH::SubShapeIDCreator(),
			JPH::CollideShapeSettings(), collector );

		if ( collector.intersection )
		{
			pTrace->allsolid = true;
			pTrace->startsolid = true;
		}
	}
}

//-------------------------------------------------------------------------------------------------
//
// Where we figure out which type this trace actually is
//
//-------------------------------------------------------------------------------------------------

static void TraceBase( const Ray_t &ray, uint32 contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace )
{
	VJoltAssert( pCollide && pTrace );

	// Default out our trace
	ClearTrace( pTrace );

	// We can't trust Ray_t's settings because after conversion from Source > Jolt the coordinates might become tiny
	bool isPoint = SourceToJolt::Distance( ray.m_Extents ).ReduceMin() < kMaxConvexRadius;
	bool isCollide = !ray.m_IsSwept;

	if ( isPoint )
	{
		if ( isCollide )
		{
			CollidePoint( ray, contentsMask, pConvexInfo, pCollide, collideOrigin, collideAngles, pTrace );
		}
		else
		{
			CastRay( ray, contentsMask, pConvexInfo, pCollide, collideOrigin, collideAngles, pTrace );
		}
	}
	else
	{
		if ( isCollide )
		{
			// TODO(Slart): This should be CollideBoxVsShape, but I can't remember why it wasn't good enough...
			CastBoxVsShape( ray, contentsMask, pConvexInfo, pCollide, collideOrigin, collideAngles, pTrace );
		}
		else
		{
			CastBoxVsShape( ray, contentsMask, pConvexInfo, pCollide, collideOrigin, collideAngles, pTrace );
		}
	}
}

} // namespace VJoltTrace

//-------------------------------------------------------------------------------------------------
//
// IPhysicsCollision Interface
//
//-------------------------------------------------------------------------------------------------

void JoltPhysicsCollision::TraceBox( const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	Ray_t ray;
	ray.Init( start, end, mins, maxs );
	VJoltTrace::TraceBase( ray, MASK_ALL, nullptr, pCollide, collideOrigin, collideAngles, ptr );
}

void JoltPhysicsCollision::TraceBox( const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	VJoltTrace::TraceBase( ray, MASK_ALL, nullptr, pCollide, collideOrigin, collideAngles, ptr );
}

void JoltPhysicsCollision::TraceBox( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	VJoltTrace::TraceBase( ray, contentsMask, pConvexInfo, pCollide, collideOrigin, collideAngles, ptr );
}

void JoltPhysicsCollision::TraceCollide( const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace )
{
	VJoltTrace::CollideShapeVsShape( start, end, pSweepCollide, sweepAngles, pCollide, collideOrigin, collideAngles, pTrace );
}
