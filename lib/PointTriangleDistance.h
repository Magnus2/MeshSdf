#pragma once

#include <array>
#include "DataStructs.h"

namespace MeshSdf
{
	enum class NearestTriEntity
	{
		Vert0, Vert1, Vert2, Edge0, Edge1, Edge2, Face
	};

	std::pair<Vec3, NearestTriEntity> PtOnTri(
		Vec3 const& pquery,
		std::array<Vec3 const*, 3> const& vertstri);
}