#pragma once

#include <vector>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include "DataStructs.h"

namespace MeshSdf
{
	class MeshAdjacencyTable
	{
		using Edge = std::pair<int, int>;

		std::vector<Tri> tris;
		std::unordered_map<Edge, std::vector<int>, boost::hash<Edge>> edge2tris;
		std::vector<std::vector<int>> vert2tris;

	public:

		MeshAdjacencyTable(std::vector<Tri> const& tris, int numVerts);

		std::vector<int> Edge2Tris(int edgeVert1, int edgeVert2) const;
		std::vector<int> Vert2Tris(int vertex) const;
	};
}