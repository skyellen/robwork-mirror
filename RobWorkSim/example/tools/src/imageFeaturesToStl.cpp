#include <fstream>
#include <vector>
#include <string>

#include <rw/geometry/Box.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/math/Random.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/loaders/model3d/STLFile.hpp>

#include <boost/lexical_cast.hpp>

using namespace rw::geometry;
using rw::loaders::STLFile;
using namespace rw::math;

int main(int argc, char** argv)
{
    Random::seed(time(NULL));
    srand ( time(NULL) );

    if( argc < 4 ){
		std::cout << "------ Usage: " << std::endl;
	    std::cout << "- Arg 1 name of col data file" << std::endl;
	    std::cout << "- Arg 2 width of texels in meters\n" << std::endl;
	    std::cout << "- Arg 3 name of output stl file\n" << std::endl;
	    return 0;
	}

	std::string filename(argv[1]);
	double width = boost::lexical_cast<double>(argv[2]);
	std::string outfile(argv[3]);
	std::vector<Vector3D<> > texlets;
	char line[256];
	std::ifstream input(filename.c_str());
	// first line is a plane

	float vals[4];
	input.getline(line, 256);
    sscanf(line, "%f %f %f %f", &vals[0], &vals[1], &vals[2], &vals[3]);

    Vector3D<> n(vals[0], vals[1], vals[2]);


    Plane p( normalize(n), ((vals[3])/n.norm2())*-0.001 );

	// and then there are texlets
	while(!input.eof()){
	    input.getline(line, 256);
	    sscanf(line, "%f %f %f", &vals[0], &vals[1], &vals[2]);
	    Vector3D<> v(vals[0],vals[1],vals[2]);
	    v = v*0.001; // the values are in mm
	    texlets.push_back(v);
	}
	input.close();
	// now create the triangle mesh. Use boxes on the texlets
	Box texletBox(width,width,width);
	TriMesh::Ptr texletMesh = texletBox.getTriMesh();
	TriMesh::Ptr planeMesh = p.getTriMesh();
	size_t meshSize = planeMesh->size()+texletMesh->size()*texlets.size();
	PlainTriMesh<> stlMesh(meshSize);

	// and copy plane triangles into the mesh
	for(size_t i=0;i<planeMesh->size();i++)
	    stlMesh[i] = planeMesh->getTriangle(i);

	// and copy every point into the mesh
	for(size_t i=0;i<texlets.size();i++){
	    Transform3D<> transform(texlets[i], Rotation3D<>::identity());
	    for(size_t j=0;j<texletMesh->size();j++){
	        size_t idx = planeMesh->size()+i*texletMesh->size()+j;
	        if(idx>=stlMesh.size())
	            RW_THROW(idx<< ">=" << stlMesh.size() << "  i:" << i << " " << j);
	        stlMesh[idx] = texletMesh->getTriangle(j).transform( transform );
	    }
	}

	STLFile::save(stlMesh, outfile);
	return 0;
}
