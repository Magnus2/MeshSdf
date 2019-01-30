#pragma once

#include <boost/gil/image.hpp>

namespace MeshSdf
{
	struct Mesh;
	struct Vec3;

	struct Box
	{
		double xmin, xmax, ymin, ymax, zmin, zmax;
	};

	Box BoundingBox(Mesh const& m);
	Box Scale(Box b, float scale);

	boost::gil::rgb8_image_t ColorCodedSlice(
		std::function<double(double, double, double)> const& f,
		int imHeight, int imWidth,
		Box const& boundBox,
		float depthPercent);
}