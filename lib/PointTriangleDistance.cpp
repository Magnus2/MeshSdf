#include "PointTriangleDistance.h"

std::pair<MeshSdf::Vec3, MeshSdf::NearestTriEntity> MeshSdf::PtOnTri(
	Vec3 const& pquery,
	std::array<Vec3 const*, 3> const& verttri)
{
	auto diff = *verttri[0] - pquery;
	auto edge0 = *verttri[1] - *verttri[0];
	auto edge1 = *verttri[2] - *verttri[0];
	auto a00 = edge0.Dot(edge0);
	auto a01 = edge0.Dot(edge1);
	auto a11 = edge1.Dot(edge1);
	auto b0 = diff.Dot(edge0);
	auto b1 = diff.Dot(edge1);
	auto det = std::abs(a00 * a11 - a01 * a01);
	auto s = a01 * b1 - a11 * b0;
	auto t = a01 * b0 - a00 * b1;

	NearestTriEntity ne;

	if (s + t <= det)
	{
		if (s < 0)
		{
			if (t < 0) // region 4
			{
				if (b0 < 0)
				{
					t = 0;
					if (-b0 >= a00)
					{
						// VN1
						ne = NearestTriEntity::Vert1;
						s = 1;
					}
					else
					{
						// EN0
						ne = NearestTriEntity::Edge0;
						s = -b0 / a00;
					}
				}
				else
				{
					s = 0;

					if (b1 >= 0)
					{
						// VN0
						ne = NearestTriEntity::Vert0;
						t = 0;
					}
					else if (-b1 >= a11)
					{
						// VN2
						ne = NearestTriEntity::Vert2;
						t = 1;
					}
					else
					{
						// EN2
						ne = NearestTriEntity::Edge2;
						t = -b1 / a11;
					}
				}
			}
			else // region 3
			{
				s = 0;

				if (b1 >= 0)
				{
					// VN0
					ne = NearestTriEntity::Vert0;
					t = 0;
				}
				else if (-b1 >= a11)
				{
					// VN2
					ne = NearestTriEntity::Vert2;
					t = 1;
				}
				else
				{
					// EN2
					ne = NearestTriEntity::Edge2;
					t = -b1 / a11;
				}
			}
		}
		else if (t < 0) // region 5
		{
			t = 0;

			if (b0 >= 0)
			{
				// VN0
				ne = NearestTriEntity::Vert0;
				s = 0;
			}
			else if (-b0 >= a00)
			{
				// VN1
				ne = NearestTriEntity::Vert1;
				s = 1;
			}
			else
			{
				// EN0
				ne = NearestTriEntity::Edge0;
				s = -b0 / a00;
			}
		}
		else // region 0
		{
			// FN
			ne = NearestTriEntity::Face;
			// minimum at interior point
			auto invDet = (1) / det;
			s *= invDet;
			t *= invDet;
		}
	}
	else
	{
		double tmp0, tmp1, numer, denom;

		if (s < 0) // region 2
		{
			tmp0 = a01 + b0;
			tmp1 = a11 + b1;

			if (tmp1 > tmp0)
			{
				numer = tmp1 - tmp0;
				denom = a00 - (2) * a01 + a11;

				if (numer >= denom)
				{
					// VN1
					ne = NearestTriEntity::Vert1;
					s = 1;
					t = 0;
				}
				else
				{
					// EN1
					ne = NearestTriEntity::Edge1;
					s = numer / denom;
					t = 1 - s;
				}
			}
			else
			{
				s = 0;

				if (tmp1 <= 0)
				{
					// VN2
					ne = NearestTriEntity::Vert2;
					t = 1;
				}
				else if (b1 >= 0)
				{
					// VN0
					ne = NearestTriEntity::Vert0;
					t = 0;
				}
				else
				{
					// EN2
					ne = NearestTriEntity::Edge2;
					t = -b1 / a11;
				}
			}
		}
		else if (t < 0) // region 6
		{
			tmp0 = a01 + b1;
			tmp1 = a00 + b0;

			if (tmp1 > tmp0)
			{
				numer = tmp1 - tmp0;
				denom = a00 - 2 * a01 + a11;

				if (numer >= denom)
				{
					// VN2
					ne = NearestTriEntity::Vert2;
					t = 1;
					s = 0;
				}
				else
				{
					// EN1
					ne = NearestTriEntity::Edge1;
					t = numer / denom;
					s = 1 - t;
				}
			}
			else
			{
				t = 0;

				if (tmp1 <= 0)
				{
					// VN1
					ne = NearestTriEntity::Vert1;
					s = 1;
				}
				else if (b0 >= 0)
				{
					// VN0
					ne = NearestTriEntity::Vert0;
					s = 0;
				}
				else
				{
					// EN0
					ne = NearestTriEntity::Edge0;
					s = -b0 / a00;
				}
			}
		}
		else // region 1
		{
			numer = a11 + b1 - a01 - b0;

			if (numer <= 0)
			{
				// VN2
				ne = NearestTriEntity::Vert2;
				s = 0;
				t = 1;
			}
			else
			{
				denom = a00 - (2) * a01 + a11;

				if (numer >= denom)
				{
					// VN1
					ne = NearestTriEntity::Vert1;
					s = 1;
					t = 0;
				}
				else
				{
					// EN1
					ne = NearestTriEntity::Edge1;
					s = numer / denom;
					t = 1 - s;
				}
			}
		}
	}

	auto pnearest = *verttri[0] + edge0 * s + edge1 * t;

	return { pnearest, ne };
}
