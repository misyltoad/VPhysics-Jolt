//=================================================================================================
//
// Source / Jolt utilities
//
//=================================================================================================

#pragma once

inline constexpr float InchesToMetres = 0.0254f;
inline constexpr float MetresToInches = 1.0f / 0.0254f;

// TODO! Remove loam_expr -> constexpr when mathlib stuff is sorted.
#define loam_expr inline

loam_expr Vector VectorHalfExtent( Vector mins, Vector maxs )
{
	return 0.5f * ( maxs - mins );
}

loam_expr Quaternion ToQuaternion( const QAngle& angles )
{
	Quaternion result;
	AngleQuaternion( angles, result );
	return result;
}

loam_expr QAngle ToQAngle( const Quaternion &q )
{
	QAngle result;
	QuaternionAngles( q, result );
	return result;
}

loam_expr QAngle ToQAngle( const matrix3x4_t& m )
{
	QAngle result;
	MatrixAngles( m, result );
	return result;
}

loam_expr Vector Abs( const Vector &v )
{
	Vector result;
	VectorAbs( v, result );
	return result;
}

loam_expr Vector Rotate( const Vector &vector, const Quaternion &angle )
{
	Vector out;
	VectorRotate( vector, angle, out );
	return out;
}

loam_expr Vector Rotate( const Vector& vector, const matrix3x4_t &matrix )
{
	Vector out;
	VectorRotate( vector, matrix, out );
	return out;
}

template < typename T >
constexpr T Cube( T x )
{
	return x * x * x;
}

namespace MatrixAxis
{
	enum JoltMatrixAxes
	{
		Forward = 0,
		Left = 1,
		Up = 2,

		X = 0,
		Y = 1,
		Z = 2,

		Origin = 3,
		Projective = 3,
	};
}
using JoltMatrixAxes = MatrixAxis::JoltMatrixAxes;

loam_expr Vector GetColumn( const matrix3x4_t& m, JoltMatrixAxes axis )
{
	Vector value;
	MatrixGetColumn( m, (int)axis, value );
	return value;
}

//
// SourceToJolt:
//
// Type conversions from:
//   - JPH::Vec3 -> Vector,
//   - JPH::Float3 -> Vector,
//   - JPH::Quat-> Quaternion,
//   - JPH::Quat -> QAngle,
// Unit conversions from:
//   - metres -> inches (distance, area, volume, energy)
//   - radians -> degrees
namespace JoltToSource
{
	inline constexpr float Factor	 = MetresToInches;
	inline constexpr float InvFactor = InchesToMetres;

	// This is for doing direct type conversions
	// ie. normals, directions, scale factors, coefficients and
	// certain dimensionless quantities.
	loam_expr float			Unitless( float value )					{ return value; }
	loam_expr Vector		Unitless( JPH::Vec3Arg value )			{ return Vector( value[0], value[1], value[2] ); }
	loam_expr Vector		Unitless( JPH::Float3 value )			{ return Vector( value[0], value[1], value[2] ); }

	// This is used for any unit that has a singular metre factor
	// ie. distance (m), velocity (m/s), acceleration (m/s^2), force (kg m/s^2).
	loam_expr float			Distance( float value )					{ return value * Factor; }
	loam_expr Vector		Distance( JPH::Vec3Arg value )			{ return Vector( Distance( value[0] ), Distance( value[1] ), Distance( value[2] ) ); } 
	loam_expr Vector		Distance( JPH::Float3 value )			{ return Vector( Distance( value[0] ), Distance( value[1] ), Distance( value[2] ) ); } 

	// m^2 -> in^2
	loam_expr float			Area( float value )						{ return value * Factor * Factor; }
	// m^3 -> in^3
	loam_expr float			Volume( float value )					{ return value * Factor * Factor * Factor; }
	
	// These handle converting Quaternions/QAngles -> JPH::Quat,
	// which is a direct type passthrough.
	// They also handles converting scalar angle units, which is rad -> deg.
	loam_expr Quaternion 	Quat( JPH::QuatArg value ) 		        { return Quaternion( value.GetX(), value.GetY(), value.GetZ(), value.GetW() ); }
	loam_expr float			Angle( float value )					{ return RAD2DEG( value ); }
	loam_expr QAngle		Angle( JPH::QuatArg value )				{ return ToQAngle( Quat( value ) ); }

	loam_expr float			Energy( float value )					{ return value / ( InvFactor * InvFactor ); }

	// Converts types and handles the angle (rad -> deg) unit conversion.
	loam_expr float			AngularImpulse( float value )			{ return Angle( value ); }
	loam_expr Vector		AngularImpulse( JPH::Vec3Arg value )	{ return Vector( AngularImpulse( value[0] ), AngularImpulse( value[1] ), AngularImpulse( value[2] ) ); }

	// Misc. AABB helpers.
    loam_expr Vector		AABBCenter( const JPH::AABox &aabox )		{ return Distance( aabox.mMin + aabox.GetExtent() ); }
    loam_expr Vector        AABBHalfExtent( const JPH::AABox &aabox )	{ return Distance( aabox.GetExtent() ); }
    loam_expr void          AABBBounds( const JPH::AABox &aabox, Vector &outMins, Vector &outMaxs )
    {
        outMins = AABBCenter( aabox ) - AABBHalfExtent( aabox );
        outMaxs = AABBCenter( aabox ) + AABBHalfExtent( aabox );
    }

	loam_expr matrix3x4_t	Matrix( JPH::Mat44Arg matrix )
	{
		return matrix3x4_t
		{
			Vector( matrix.GetAxisX().GetX(), matrix.GetAxisX().GetY(), matrix.GetAxisX().GetZ() ),
			Vector( matrix.GetAxisY().GetX(), matrix.GetAxisY().GetY(), matrix.GetAxisY().GetZ() ),
			Vector( matrix.GetAxisZ().GetX(), matrix.GetAxisZ().GetY(), matrix.GetAxisZ().GetZ() ),
			Distance( matrix.GetTranslation() ),
		};
	}

	loam_expr ::Color		Color( JPH::ColorArg color )
	{
		return ::Color( color.r, color.g, color.b, color.a );
	}
}

//
// SourceToJolt:
//
// Type conversions from:
//   - Vector -> JPH::Vec3,
//   - Vector -> JPH::Float3,
//   - Quaternion -> JPH::Quat,
//   - QAngle -> JPH::Quat,
// Unit conversions from:
//  - inches -> metres (distance, area, volume, energy)
//  - degrees -> radians
namespace SourceToJolt
{
	inline constexpr float Factor	 = InchesToMetres;
	inline constexpr float InvFactor = MetresToInches;

	// This is for doing direct type conversions
	// ie. normals, directions, scale factors, coefficients and
	// certain dimensionless quantities.
	loam_expr float			Unitless( float value )					{ return value; }
	loam_expr JPH::Vec3		Unitless( Vector value )				{ return JPH::Vec3( value[0], value[1], value[2] ); }
	loam_expr JPH::Float3	UnitlessFloat3( Vector value )			{ return JPH::Float3( value[0], value[1], value[2] ); }

	// This is used for any unit that has a singular Source Unit(tm) "inch" factor
	// ie. distance (in), velocity (in/s), acceleration (in/s^2), force (kg in/s^2).
	loam_expr float			Distance( float value )					{ return value * Factor; }
	loam_expr JPH::Vec3		Distance( Vector value )				{ return JPH::Vec3( Distance( value[0] ), Distance( value[1] ), Distance( value[2] ) ); } 
	loam_expr JPH::Float3	DistanceFloat3( Vector value )			{ return JPH::Float3( Distance( value[0] ), Distance( value[1] ), Distance( value[2] ) ); } 

	// in^2 -> m^2
	loam_expr float			Area( float value )						{ return value * Factor * Factor; }
	// in^3 -> m^3
	loam_expr float			Volume( float value )					{ return value * Factor * Factor * Factor; }
	
	// These handle converting JPH::Quat -> Quaternions/QAngles,
	// which is a direct type passthrough.
	// They also handles converting scalar angle units, which is deg -> rad.
	loam_expr JPH::Quat 	Quat( Quaternion value ) 		        { return JPH::Quat( value.x, value.y, value.z, value.w ); }
	loam_expr float			Angle( float value )					{ return DEG2RAD( value ); }
	loam_expr JPH::Quat		Angle( QAngle value )					{ return Quat( ToQuaternion( value ) ); }

	loam_expr float			Energy( float value )					{ return value / ( InvFactor * InvFactor ); }

	// Converts types and handles the angle (deg -> rad) unit conversion.
	loam_expr float			AngularImpulse( float value )			{ return Angle( value ); }
	loam_expr JPH::Vec3		AngularImpulse( Vector value )			{ return JPH::Vec3( AngularImpulse( value[0] ), AngularImpulse( value[1] ), AngularImpulse( value[2] ) ); }

	// Misc. AABB helpers.
	loam_expr JPH::Vec3		AABBCenter( Vector mins, Vector maxs )			{ return Distance( mins + VectorHalfExtent( mins, maxs ) ); }
	loam_expr JPH::Vec3		AABBHalfExtent( Vector mins, Vector maxs )		{ return Distance( VectorHalfExtent( mins, maxs ) ); }
	loam_expr JPH::AABox 	AABBBounds( Vector mins, Vector maxs )
	{
		return JPH::AABox
		{ 
			AABBCenter( mins, maxs ) - AABBHalfExtent( mins, maxs ),
			AABBCenter( mins, maxs ) + AABBHalfExtent( mins, maxs ),
		};
	}

	loam_expr JPH::Mat44    Matrix( const matrix3x4_t &m )
    {
        return JPH::Mat44
        {
            JPH::Vec4( GetColumn( m, MatrixAxis::X ).x, GetColumn( m, MatrixAxis::X ).y, GetColumn( m, MatrixAxis::X ).z, 0.0f ),
            JPH::Vec4( GetColumn( m, MatrixAxis::Y ).x, GetColumn( m, MatrixAxis::Y ).y, GetColumn( m, MatrixAxis::Y ).z, 0.0f ),
            JPH::Vec4( GetColumn( m, MatrixAxis::Z ).x, GetColumn( m, MatrixAxis::Z ).y, GetColumn( m, MatrixAxis::Z ).z, 0.0f ),
            JPH::Vec4( Distance( GetColumn( m, MatrixAxis::Origin ) ),                                                   1.0f ),
        };
    }
}

// Traces

// Same as CM_ClearTrace
inline void ClearTrace( trace_t *trace )
{
	memset( trace, 0, sizeof( *trace ) );
	trace->fraction = 1.0f;
	trace->fractionleftsolid = 0.0f;
	trace->surface.name = "**empty**";
}

// Converts a JoltPhysics smart ref-counted pointer to a raw pointer with a dangling
// reference that we can clean up explicitly explcitly later in eg. ConvexFree.
//
// The main reason is to avoids trailing class pointers that just have a single instance
// of this class that then get deleted, when we can just pass the raw pointer around ourselves
// and explicitly dereference on ourside when the game calls delete.
template < typename T >
T *ToDanglingRef( const JPH::Ref< T >& ref )
{
	T *pPtr = ref.GetPtr();
	pPtr->AddRef();
	return pPtr;
}

template < typename T >
bool VectorContains( const std::vector< T >& vector, const T &object )
{
	return std::find(vector.begin(), vector.end(), object) != vector.end();
}

inline const JPH::Shape* UndecorateShape( const JPH::Shape *pShape )
{
	if ( pShape->GetType() == JPH::EShapeType::Decorated )
		pShape = static_cast< const JPH::DecoratedShape * >( pShape )->GetInnerShape();

	return pShape;
}

inline const JPH::StaticCompoundShape *GetCompoundShape( const JPH::Shape *pShape )
{
	pShape = UndecorateShape( pShape );

	return pShape->GetType() == JPH::EShapeType::Compound
		? static_cast< const JPH::StaticCompoundShape * >( pShape )
		: nullptr;
}

template < typename T, typename ShapeType, typename Func >
T ActOnSubShape( const JPH::Shape *pShape, int nIndex, Func ShapeFunc )
{
	const JPH::StaticCompoundShape *pCompoundShape = GetCompoundShape( pShape );
	if ( pCompoundShape )
	{
		const JPH::CompoundShape::SubShape& subShape = pCompoundShape->GetSubShape( nIndex );
		return ShapeFunc( static_cast< const ShapeType * >( UndecorateShape( subShape.mShape.GetPtr() ) ) );
	}

	return ShapeFunc( static_cast< const ShapeType * >( UndecorateShape( pShape ) ) );
}

template < typename ShapeType, typename Func >
void ActOnSubShapes( const JPH::Shape *pShape, Func ShapeFunc )
{
	const JPH::StaticCompoundShape *pCompoundShape = GetCompoundShape( pShape );
	if ( pCompoundShape )
	{
		for ( const JPH::CompoundShape::SubShape& subShape : pCompoundShape->GetSubShapes() )
		{
			JPH::Mat44 matLocalTranslation = JPH::Mat44::sRotationTranslation( subShape.GetRotation(), subShape.GetPositionCOM() );
			ShapeFunc( static_cast< const ShapeType * >( UndecorateShape( subShape.mShape.GetPtr() ) ), matLocalTranslation );
		}
		return;
	}

	ShapeFunc( static_cast< const ShapeType * >( UndecorateShape( pShape ) ), JPH::Mat44::sIdentity() );
}

template< typename T, typename Value >
constexpr void Erase( T &c, const Value &value )
{
	auto it = std::remove( c.begin(), c.end(), value );
	c.erase( it, c.end() );
}

template< typename T, typename Pred >
constexpr void EraseIf( T &c, Pred pred )
{
	auto it = std::remove_if( c.begin(), c.end(), pred );
	c.erase( it, c.end() );
}

template< typename T, typename Value >
constexpr bool Contains( const T &c, const Value &value )
{
	return c.find( value ) != c.end();
}
