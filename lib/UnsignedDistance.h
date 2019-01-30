#pragma once

#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include "PointTriangleDistance.h"
#include "DataStructs.h"

namespace MeshSdf
{
	namespace bg = boost::geometry;
	namespace bgi = boost::geometry::index;

	using Point_ = bg::model::point<double, 3, bg::cs::cartesian>;
	using Box_   = bg::model::box<Point_>;
	using Value_ = std::pair<Box_, int>;
	using RTree  = bgi::rtree<Value_, bgi::rstar<8>>;

	using Ring_ = bg::model::ring<Point_, true, false>;

	class UnsignedDistance
	{
		RTree rtree;
		std::vector<Vec3> verts;
		std::vector<Tri> tris;

	public:

		UnsignedDistance(std::vector<Vec3> verts, std::vector<Tri> tris);

		struct UDistInfo
		{
			double udist;
			int triNearest;
			Vec3 triPtNearest;
			NearestTriEntity triEntityNearest;
		};

		UDistInfo operator()(double x, double y, double z, int) const;
		double operator()(double x, double y, double z) const;

	};
}
