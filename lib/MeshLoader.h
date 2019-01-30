#pragma once

#include "DataStructs.h"

namespace MeshSdf
{
	Mesh LoadObj(std::string const& file);

	std::vector<Vec3> TriangleNormals(
		std::vector<Vec3> const& verts,
		std::vector<Tri> const& tris);
}
