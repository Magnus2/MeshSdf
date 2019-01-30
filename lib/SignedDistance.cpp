#include "SignedDistance.h"
#include "PointTriangleDistance.h"
#include <boost/assert.hpp>

using namespace MeshSdf;

SignedDistance::SignedDistance(Mesh mesh)
	: mesh(std::forward<Mesh>(mesh))
	, meshadj(this->mesh.tris, static_cast<int>(this->mesh.verts.size()))
	, usdist(this->mesh.verts, this->mesh.tris)
{
}

double SignedDistance::operator()(double x, double y, double z) const
{
	auto udinfo = usdist(x, y, z, 0);

	auto pnormal = PseudoNormal(udinfo.triEntityNearest, udinfo.triNearest);
	auto qc = Vec3{ x,y,z } - udinfo.triPtNearest;
	auto sign = boost::math::sign(qc.Dot(pnormal));

	return sign * udinfo.udist;
}

Vec3 SignedDistance::PseudoNormal_Vertex(int vnearest) const
{
	Vec3 pnormal{ 0.0, 0.0, 0.0 };

	auto trisadj = meshadj.Vert2Tris(vnearest);
	BOOST_ASSERT_MSG(!trisadj.empty(), "no incident triangles");

	for (auto itriadj : trisadj)
	{
		auto const& triadj = mesh.tris.at(itriadj);

		// find other 2 vertices of tri
		auto edgevert0 = -1;
		auto edgevert1 = -1;

		if (vnearest == triadj[0])
		{
			edgevert0 = triadj[1];
			edgevert1 = triadj[2];
		}
		else if (vnearest == triadj[1])
		{
			edgevert0 = triadj[0];
			edgevert1 = triadj[2];
		}
		else if (vnearest == triadj[2])
		{
			edgevert0 = triadj[0];
			edgevert1 = triadj[1];
		}
		else
		{
			BOOST_ASSERT_MSG(false, "no nearest vertex");
		}

		// weigh triangle normal by incident angle at nearest vertex
		auto edge1 = mesh.verts.at(edgevert0) - mesh.verts.at(vnearest);
		auto edge2 = mesh.verts.at(edgevert1) - mesh.verts.at(vnearest);

		auto incidentAngle = acos(edge1.Dot(edge2) / (edge1.Norm()*edge2.Norm()));
		pnormal += incidentAngle * mesh.triNormals.at(itriadj);
	}

	return pnormal.Normalized();
}

Vec3 SignedDistance::PseudoNormal_Edge(NearestTriEntity ent, int tri) const
{
	auto const& triadj = mesh.tris.at(tri);

	// find edge vertices
	auto nearestvert0 = -1;
	auto nearestvert1 = -1;

	if (ent == NearestTriEntity::Edge0)
	{
		nearestvert0 = triadj[0];
		nearestvert1 = triadj[1];
	}
	else if (ent == NearestTriEntity::Edge1)
	{
		nearestvert0 = triadj[1];
		nearestvert1 = triadj[2];
	}
	else if (ent == NearestTriEntity::Edge2)
	{
		nearestvert0 = triadj[2];
		nearestvert1 = triadj[0];
	}

	// equal weight for triangle normal of both adjacent triangles
	auto adjtris = meshadj.Edge2Tris(nearestvert0, nearestvert1);

	BOOST_ASSERT_MSG(adjtris.size() == 1 || adjtris.size() == 2,
		"invalid number of incident triangles");

	Vec3 pnormal{ 0.0, 0.0 };

	for (auto adjtri : adjtris)
	{
		pnormal += mesh.triNormals.at(adjtri);
	}

	return pnormal.Normalized();
}

Vec3 SignedDistance::PseudoNormal(NearestTriEntity ent, int tri) const
{
	switch (ent)
	{
		case NearestTriEntity::Vert0: return PseudoNormal_Vertex(mesh.tris.at(tri)[0]);
		case NearestTriEntity::Vert1: return PseudoNormal_Vertex(mesh.tris.at(tri)[1]);
		case NearestTriEntity::Vert2: return PseudoNormal_Vertex(mesh.tris.at(tri)[2]);
		case NearestTriEntity::Edge0:
		case NearestTriEntity::Edge1:
		case NearestTriEntity::Edge2: return PseudoNormal_Edge(ent, tri);
		case NearestTriEntity::Face: return mesh.triNormals.at(tri);
		default: throw std::runtime_error{"unknown entity"};
	}

	return {};
}
