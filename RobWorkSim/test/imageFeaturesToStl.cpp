#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <sys/stat.h>
#include <iostream>
#include <rw/rw.hpp>
#include <rwlibs/task.hpp>

#include <vector>

#include <rw/geometry/STLFile.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/GeometryFactory.hpp>

#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>

#include <rwsim/dynamics/ContactManifold.hpp>
#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>

#include <rwsim/dynamics/DynamicUtil.hpp>
#include <rw/rw.hpp>
#include <rwsim/dynamics/ContactManifold.hpp>
#include <rw/geometry/GeometryFactory.hpp>
#include <boost/lexical_cast.hpp>
USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace rwsim::dynamics;

using namespace boost::numeric::ublas;

int main(int argc, char** argv)
{
    Math::seed(time(NULL));
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
	Plane p( Q(4, vals[0], vals[1], vals[2], vals[3]) );
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
