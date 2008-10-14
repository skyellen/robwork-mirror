#ifndef STLFILE_HPP_
#define STLFILE_HPP_

#include <rw/geometry/Face.hpp>


#include <sandbox/trimesh/TriMesh.hpp>
#include <sandbox/trimesh/PlainTriMesh.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/math/Vector3D.hpp>

#include <vector>
#include <iostream>
#include <fstream>


class STLFile {
public:

	static void writeSTL(const std::vector<rw::geometry::Face<> >& faces,
						 const std::string& filename){
		using namespace rw::math;
		std::ofstream ostr(filename.c_str());
		ostr << "solid ascii" << std::endl;
		for(size_t i=0; i<faces.size(); i++){
			Vector3D<> v1(faces[i]._vertex1[0],faces[i]._vertex1[1],faces[i]._vertex1[2]);
			Vector3D<> v2(faces[i]._vertex2[0],faces[i]._vertex2[1],faces[i]._vertex2[2]);
			Vector3D<> v3(faces[i]._vertex3[0],faces[i]._vertex3[1],faces[i]._vertex3[2]);
			Vector3D<> n(faces[i]._normal[0],faces[i]._normal[1],faces[i]._normal[2]);

			writeFaceSTL<double>(v1,v2,v3,n,ostr);

		}
		ostr << "endsolid" << std::endl;
		ostr.flush();
		ostr.close();
	}

	//template <class T>
	static void writeSTL(rw::geometry::TriMesh& mesh,
						 const std::string& filename){
		using namespace rw::geometry;
		std::ofstream ostr;
		ostr.open( filename.c_str() );
		while( !ostr.is_open() ){
			std::cout << "Error state: " << ostr.rdstate() << std::endl;
			ostr.clear();
			ostr.open( filename.c_str() );
			rw::common::TimerUtil::sleepMs(1000);
		}
		RW_ASSERT( ostr.is_open() );

		ostr << "solid ascii" << std::endl;
		for(size_t i = 0; i<mesh.getSize(); i++){
			TriangleN0<double> tri = mesh.getTriangle(i);
			writeFaceSTL(tri[0],tri[1],tri[2],tri.calcFaceNormal(), ostr);
		}
		ostr << "endsolid" << std::endl;
		ostr.flush();
		ostr.close();
	}

    static rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<float> >*
        read(const std::string& filename);

private:



	template <class T>
	static void writeFaceSTL(const rw::math::Vector3D<T>& v1,
							 const rw::math::Vector3D<T>& v2,
							 const rw::math::Vector3D<T>& v3,
							 const rw::math::Vector3D<T>& n,
							 std::ostream& ostr){
		ostr << " facet normal "
			<< n[0] << " " << n[1] << " " << n[2] << std::endl;
		ostr << "  outer loop " << std::endl;
		ostr << "   vertex "
			<< v1[0] << " " << v1[1] << " " << v1[2] << std::endl;
		ostr << "   vertex "
			<< v2[0] << " " << v2[1] << " " << v2[2] << std::endl;
		ostr << "   vertex "
			<< v3[0] << " " << v3[1] << " " << v3[2] << std::endl;
		ostr << "  endloop " << std::endl;
		ostr << " endfacet" << std::endl;
	}

};

#endif /*STLFILE_HPP_*/
