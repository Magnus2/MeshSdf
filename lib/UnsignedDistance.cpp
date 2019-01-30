#include "UnsignedDistance.h"

using namespace MeshSdf;
using namespace std;

namespace
{
	vector<pair<Box_, int>> BoundBoxesWithTris(vector<Vec3> const& verts, vector<Tri> const& tris)
	{
		vector<pair<Box_, int>> boundboxtris;

		for (auto itri = 0; itri < tris.size(); ++itri)
		{
			auto const& tri = tris[itri];

			auto const& v1 = verts[tri[0]];
			auto const& v2 = verts[tri[1]];
			auto const& v3 = verts[tri[2]];

			Point_ p1(v1.x, v1.y, v1.z);
			Point_ p2(v2.x, v2.y, v2.z);
			Point_ p3(v3.x, v3.y, v3.z);

			Ring_ triring;
			bg::append(triring, p1);
			bg::append(triring, p2);
			bg::append(triring, p3);

			auto boundbox = bg::return_envelope<Box_>(triring);
			auto bbtri = pair<Box_, int>(boundbox, itri);
			boundboxtris.push_back(bbtri);
		}

		return boundboxtris;
	}

	Box_ CreateBox(Vec3 const& p, double offset)
	{
		Vec3 off{ offset,offset,offset };
		auto a = p - off;
		auto b = p + off;

		return Box_({a.x, a.y, a.z}, {b.x, b.y, b.z});
	}

	UnsignedDistance::UDistInfo Dist2TriSq(Vec3 const& pquery, Vec3 const& triv0, Vec3 const& triv1, Vec3 const& triv2, int tri)
	{
		array<Vec3 const*, 3> vstri = { &triv0, &triv1, &triv2 };

		auto [ptri, ent] = PtOnTri(pquery, vstri);

		// use squared norm to avoid root
		auto dsq = (ptri - pquery).NormSq();

		return { dsq, tri, ptri, ent };
	}

	array<Vec3,3> VertsTri(vector<Vec3> const& verts, vector<Tri> const& tris, int tri)
	{
		return {
			verts[tris[tri][0]],
			verts[tris[tri][1]],
			verts[tris[tri][2]]
		};
	}
}

UnsignedDistance::UnsignedDistance(
	vector<Vec3> verts,
	vector<Tri> tris)
	: rtree(BoundBoxesWithTris(verts, tris))
	, verts(forward<vector<Vec3>>(verts))
	, tris(forward<vector<Tri>>(tris))
{
	// uses packing algorithm of rtree which improves tree structure leading to faster queries
}

UnsignedDistance::UDistInfo UnsignedDistance::operator()(double x, double y, double z, int) const
{
	// rtree query yields n nearest boxes
	// determine distance to n contained triangle
	// stop if first larger box distance encountered since results are ordered
	// if did not stop, increase size of result set to n*2 and repeat
	// skip boxes that have been processed as part of previous result set (but may be part of larger result set)
	// avoid sqrt
	// re-query w/ larger result set can use prune box to discard candidates that exceed current mindistsq

	const Point_ pquery(x, y, z);
	const Vec3 pquery_{ x, y, z };

	// special iterator faster than qend()
	const decltype(declval<RTree>().qbegin(bgi::nearest(declval<Point_>(), declval<unsigned>()))) end;

	const auto KMin = 2;
	const auto TreeSize = static_cast<int>(rtree.size());

	const auto Eps = 1e-6;

	UDistInfo udinfo{};
	udinfo.udist = numeric_limits<double>::max();

	set<int> trisseen;
	auto prunebox = CreateBox(pquery_, udinfo.udist);

	for (auto maxnumtris = min(KMin, TreeSize); maxnumtris <= TreeSize; maxnumtris *= 2)
	{
		auto newcandidates = false;

		auto pred = bgi::intersects(prunebox) && bgi::nearest(pquery, static_cast<unsigned>(maxnumtris));

		for (auto itbbtri = rtree.qbegin(pred); itbbtri != end; ++itbbtri)
		{
			auto tricandidate = itbbtri->second;

			if (trisseen.find(tricandidate) != trisseen.end())
			{
				continue;
			}

			newcandidates = true;

			auto boxdist = bg::distance(pquery, itbbtri->first);

			if (boxdist*boxdist > udinfo.udist || std::abs(udinfo.udist) <= Eps)
			{
				udinfo.udist = sqrt(udinfo.udist);
				return udinfo;
			}

			auto [v0, v1, v2] = VertsTri(verts, tris, tricandidate);
			auto udinfocandidate = Dist2TriSq(pquery_, v0, v1, v2, tricandidate);

			if (udinfocandidate.udist < udinfo.udist)
			{
				udinfo = udinfocandidate;
				prunebox = CreateBox(pquery_, std::sqrt(udinfo.udist));
			}

			trisseen.insert(tricandidate);
		}
		
		if (!newcandidates)
		{
			udinfo.udist = sqrt(udinfo.udist);
			return udinfo;
		}
	}

	throw std::runtime_error{ "no closest triangle found" };
}

double UnsignedDistance::operator()(double x, double y, double z) const
{
	return (*this)(x, y, z, 0).udist;
}
