
#pragma once

class IMesh;

#ifdef JPH_DEBUG_RENDERER

class JoltPhysicsDebugRenderer final : public JPH::DebugRenderer
{
public:
	JoltPhysicsDebugRenderer();
	~JoltPhysicsDebugRenderer() override;

	///////////////////////////////////////////
	// JPH::DebugRenderer + Draw Implementation
	///////////////////////////////////////////

	void DrawLine( JPH::Vec3Arg inFrom, JPH::Vec3Arg inTo, JPH::ColorArg inColor ) override;

	void DrawTriangle( JPH::Vec3Arg inV1, JPH::Vec3Arg inV2, JPH::Vec3Arg inV3, JPH::ColorArg inColor, ECastShadow inCastShadow = ECastShadow::Off ) override;

	Batch CreateTriangleBatch( const Triangle *inTriangles, int inTriangleCount ) override;
	Batch CreateTriangleBatch( const Vertex *inVertices, int inVertexCount, const uint32 *inIndices, int inIndexCount ) override;

	// This parameter list sucks
	void DrawGeometry( JPH::Mat44Arg inModelMatrix, const JPH::AABox &inWorldSpaceBounds, float inLODScaleSq, JPH::ColorArg inModelColor, const GeometryRef &inGeometry, ECullMode inCullMode = ECullMode::CullBackFace, ECastShadow inCastShadow = ECastShadow::On, EDrawMode inDrawMode = EDrawMode::Solid ) override;

	void DrawText3D( JPH::Vec3Arg inPosition, const std::string_view &inString, JPH::ColorArg inColor = JPH::Color::sWhite, float inHeight = 0.5f ) override;

	///////////////////////////////////////////
	// Hehe
	///////////////////////////////////////////

	void DrawJoltTVText();

	///////////////////////////////////////////
	// Main Interface
	///////////////////////////////////////////

	void RenderPhysicsSystem( JPH::PhysicsSystem &physicsSystem );

	static JoltPhysicsDebugRenderer& GetInstance();

	static IVJoltDebugOverlay *GetDebugOverlay();

private:
	class BatchImpl final : public JPH::RefTargetVirtual, public JPH::RefTarget<BatchImpl>
	{
	public:
		BatchImpl() {}
		BatchImpl( IMesh *pMesh ) { AddMesh( pMesh ); }
		~BatchImpl();

		void AddRef() override { JPH::RefTarget<BatchImpl>::AddRef(); }
		void Release() override { JPH::RefTarget<BatchImpl>::Release(); }

		void AddMesh( IMesh* pMesh ) { m_Meshes.AddToTail( pMesh ); }

		int Count() const { return m_Meshes.Count(); }
		IMesh* GetMesh(int i) const { return m_Meshes[i]; }

	private:
		CUtlVector<IMesh*> m_Meshes;
	};

	bool m_bShouldClear = false;
};

#endif
