#pragma once

#include "MeshAdjacencyTable.h"
#include "UnsignedDistance.h"
#include "PointTriangleDistance.h"
#include "DataStructs.h"

namespace MeshSdf
{
	class SignedDistance
	{
		Mesh mesh;
		MeshAdjacencyTable meshadj;
		UnsignedDistance usdist;

		Vec3 PseudoNormal_Vertex(int vnearest) const;
		Vec3 PseudoNormal_Edge(NearestTriEntity ent, int tri) const;
		Vec3 PseudoNormal(NearestTriEntity nearestEntity, int nearestTri) const;

	public:

		explicit SignedDistance(Mesh mesh);
		double operator()(double x, double y, double z) const;
	};
}