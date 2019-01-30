#pragma once

#include <cmath>
#include <vector>
#include <functional>
#include <array>
#include <boost/assert.hpp>

namespace MeshSdf
{
	struct Vec3
	{
		double x, y, z;

		void operator+=(Vec3 const& o)
		{
			x += o.x;
			y += o.y;
			z += o.z;
		}

		double Norm() const
		{
			return sqrt(x*x + y*y + z*z);
		}

		double NormSq() const
		{
			return x * x + y * y + z * z;
		}

		Vec3 Cross(Vec3 const& o) const
		{
			return {
				y*o.z - z*o.y,
				z*o.x - x*o.z,
				x*o.y - y*o.x };
		}

		double Dot(Vec3 const& o) const
		{
			return x*o.x + y*o.y + z*o.z;
		}

		Vec3 Normalized() const
		{
			BOOST_ASSERT_MSG(Norm() > 0.0, "Cannot normalize zero vector");

			auto n = Norm();
			return { x / n, y / n, z / n };
		}
	};

	inline Vec3 operator+(Vec3 const& a, Vec3 const& b)
	{
		return { a.x + b.x, a.y + b.y, a.z + b.z };
	}

	inline Vec3 operator-(Vec3 const& a, Vec3 const& b)
	{
		return { a.x - b.x, a.y - b.y, a.z - b.z };
	}

	inline Vec3 operator*(Vec3 const& v, double c)
	{
		return { c*v.x, c*v.y, c*v.z };
	}

	inline Vec3 operator*(double c, Vec3 const& v)
	{
		return v * c;
	}

	using Tri = std::array<int, 3>;

	using Fun3s = std::function<double(Vec3 const&)>;

	struct Mesh
	{
		std::vector<Vec3> verts;
		std::vector<Tri> tris;
		std::vector<Vec3> triNormals;
	};
}
