#define _SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING

#include "Visualize.h"
#include "DataStructs.h"
#include <boost/multi_array.hpp>

using namespace std;

MeshSdf::Box MeshSdf::BoundingBox(Mesh const& m)
{
	auto const Max = std::numeric_limits<double>::max();
	auto const Min = std::numeric_limits<double>::min();

	Box bb{ Max,Min,Max,Min,Max,Min };

	for (auto const& v : m.verts)
	{
		bb.xmin = min(bb.xmin, v.x);
		bb.xmax = max(bb.xmax, v.x);
		bb.ymin = min(bb.ymin, v.y);
		bb.ymax = max(bb.ymax, v.y);
		bb.zmin = min(bb.zmin, v.z);
		bb.zmax = max(bb.zmax, v.z);
	}

	return bb;
}

MeshSdf::Box MeshSdf::Scale(Box b, float scale)
{
	auto dx = b.xmax - b.xmin;
	auto dy = b.ymax - b.ymin;
	auto dz = b.zmax - b.zmin;

	auto dx_ = dx * scale;
	auto dy_ = dy * scale;
	auto dz_ = dz * scale;

	b.xmin -= (dx_ - dx) / 2;
	b.xmax += (dx_ - dx) / 2;
	b.ymin -= (dy_ - dy) / 2;
	b.ymax += (dy_ - dy) / 2;
	b.zmin -= (dz_ - dz) / 2;
	b.zmax += (dz_ - dz) / 2;

	return b;
}

boost::gil::rgb8_image_t MeshSdf::ColorCodedSlice(
	function<double(double, double, double)> const& f,
	int imHeight, int imWidth,
	Box const& bbox,
	float depthPercent)
{
	using namespace boost::gil;
	boost::multi_array<double, 2> sds{ boost::extents[imHeight][imWidth] };

	auto z = bbox.zmin + depthPercent*(bbox.zmax - bbox.zmin);

	#pragma omp parallel for schedule(dynamic,1)
	for (auto r = 0; r < imHeight; ++r)
	{
		auto y = bbox.ymax - (bbox.ymax - bbox.ymin)*(r / static_cast<double>(imHeight - 1));
		
		for (auto c = 0; c < imWidth; ++c)
		{			
			auto x = bbox.xmin + (bbox.xmax - bbox.xmin)*(c / static_cast<double>(imWidth - 1));
			sds[r][c] = f(x, y, z);
		}
	}

	auto [itminsd, itmaxsd] = minmax_element(sds.data(), sds.data() + sds.num_elements());
	auto minsd = *itminsd;
	auto maxsd = *itmaxsd;

	auto const Eps0 = (maxsd - minsd) / 512 * 2;

	rgb8_image_t img(imWidth, imHeight);

	for (auto r = 0; r < imHeight; ++r)
	{
		for (auto c = 0; c < imWidth; ++c)
		{
			if (abs(sds[r][c]) <= Eps0)
			{
				view(img)(c,r) = { 255,255,255 };
			}
			else if (sds[r][c] > Eps0)
			{
				auto i = static_cast<unsigned char>(clamp((1.0 - sds[r][c] / maxsd) * 255, 0.0, 255.0));
				view(img)(c,r) = { 0, 0, i };
			}
			else if (sds[r][c] < -Eps0)
			{
				auto i = static_cast<unsigned char>(clamp((1.0 - sds[r][c] / minsd) * 255, 0.0, 255.0));
				view(img)(c,r) = { 0, i, 0 };
			}
		}
	}

	return img;
}
