#include "MeshLoader.h"
#include <vector>
#include <fstream>
#include <sstream>
#include "DataStructs.h"

using namespace std;
using namespace MeshSdf;

Mesh MeshSdf::LoadObj(std::string const& fileName)
{
	ifstream ifs{fileName};
	BOOST_ASSERT_MSG(ifs.is_open(), "Could not open file.");

	Mesh mesh;

	string line;
	while(getline(ifs, line))
	{
		stringstream ss(line);

		string prefix;
		ss >> prefix;

		if(prefix == "v")
		{
			Vec3 v;
			ss >> v.x >> v.y >> v.z;
			mesh.verts.push_back(move(v));
		}
		else if(prefix == "f")
		{
			array<int, 3> vs;
			string trash;
			
			ss >> vs[0];

			if (ss.peek() == '/') ss >> trash;
			ss >> vs[1];

			if (ss.peek() == '/') ss >> trash;
			ss >> vs[2];

			// 0-base
			--vs[0];
			--vs[1];
			--vs[2];

			mesh.tris.push_back(move(vs));
		}
	}

	mesh.triNormals = TriangleNormals(mesh.verts, mesh.tris);
	
	BOOST_ASSERT_MSG(mesh.triNormals.size() == mesh.tris.size(), "size mismatch");
	return mesh;
}

vector<Vec3> MeshSdf::TriangleNormals(
	vector<Vec3> const& verts,
	vector<Tri> const& tris)
{
	vector<Vec3> triNormals(tris.size(), Vec3{ 0.0, 0.0, 0.0 });

	for(auto itri = 0; itri < tris.size(); ++itri)
	{
		auto const& v0 = verts.at(tris[itri][0]);
		auto const& v1 = verts.at(tris[itri][1]);
		auto const& v2 = verts.at(tris[itri][2]);

		triNormals[itri] = (v2 - v1).Cross(v0 - v1).Normalized();
	}

	return triNormals;
}