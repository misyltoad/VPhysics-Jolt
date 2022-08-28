
#pragma once

class JoltCollisionQuery final : public ICollisionQuery
{
public:
	JoltCollisionQuery( JPH::Shape *pShape );

	int				ConvexCount() override;
	int				TriangleCount( int convexIndex ) override;

	unsigned int	GetGameData( int convexIndex ) override;

	void			GetTriangleVerts( int convexIndex, int triangleIndex, Vector *verts ) override;
	void			SetTriangleVerts( int convexIndex, int triangleIndex, const Vector *verts ) override;
	
	int				GetTriangleMaterialIndex( int convexIndex, int triangleIndex ) override;
	void			SetTriangleMaterialIndex( int convexIndex, int triangleIndex, int index7bits ) override;

private:

	JPH::Ref<JPH::Shape> m_pShape;
};
