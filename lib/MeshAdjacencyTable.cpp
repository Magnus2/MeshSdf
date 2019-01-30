#include "MeshAdjacencyTable.h"

using namespace std;
using namespace MeshSdf;

namespace
{
	pair<int,int> NormalizedEdge(int vert1, int vert2)
	{
		return minmax(vert1, vert2);
	}
}

MeshAdjacencyTable::MeshAdjacencyTable(vector<array<int, 3>> const& tris, int numVerts)
	: tris(tris), vert2tris(numVerts)
{
	for (auto itri = 0; itri < tris.size(); ++itri)
	{
		auto v0 = tris[itri][0];
		auto v1 = tris[itri][1];
		auto v2 = tris[itri][2];

		vert2tris[v0].push_back(itri);
		vert2tris[v1].push_back(itri);
		vert2tris[v2].push_back(itri);

		auto e0 = NormalizedEdge(v0, v1);
		auto e1 = NormalizedEdge(v1, v2);
		auto e2 = NormalizedEdge(v2, v0);

		edge2tris[e0].push_back(itri);
		edge2tris[e1].push_back(itri);
		edge2tris[e2].push_back(itri);
	}
}

std::vector<int> MeshAdjacencyTable::Edge2Tris(int edgeVert1, int edgeVert2) const
{
	auto e = NormalizedEdge(edgeVert1, edgeVert2);
	return edge2tris.at(e);
}

std::vector<int> MeshAdjacencyTable::Vert2Tris(int vert) const
{
	return vert2tris.at(vert);
}