/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "Model3D.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/GeometryData.hpp>
#include <rw/math/Transform3D.hpp>

#include <boost/foreach.hpp>

#include <list>
#include <stack>


using namespace rw::graphics;
using namespace rw::math;
using namespace rw::geometry;

Model3D::Model3D(const std::string& name):
        _name(name),
        _filePath(""),
        _mask(rw::geometry::Geometry::PhysicalGroup),
        _isDynamic(false)
{}

Model3D::~Model3D(){ }

int Model3D::addObject(Model3D::Object3D::Ptr obj){
    _objects.push_back(obj);
    return (int)_objects.size()-1;
}

int Model3D::addMaterial(const Model3D::Material& mat){
    _materials.push_back(mat);
    return (int)_materials.size()-1;
}

void Model3D::removeObject(const std::string& name){

}

Model3D::Material* Model3D::getMaterial(const std::string& matid){
    BOOST_FOREACH(Material& mat, _materials){
        if( mat.name == matid )
            return &mat;
    }
    return NULL;
}

bool Model3D::hasMaterial(const std::string& matid){
    BOOST_FOREACH(Material& mat, _materials){
        if( mat.name == matid )
            return true;
    }
    return false;
}

rw::geometry::GeometryData::Ptr Model3D::toGeometryData(){
	// Determine data type size for the vertices
	std::size_t totalVerts = 0;
	BOOST_FOREACH(Object3D::Ptr obj, _objects){
		const std::size_t vertices = obj->_vertices.size();
		// Check that
		if (totalVerts >=  std::numeric_limits<std::size_t>::max() - vertices)
			RW_THROW("Model3D can not be converted to GeometryData as the combined number of vertices must be less than " << std::numeric_limits<std::size_t>::max() << "!");
		totalVerts += vertices;
	}

	// 64 bit is currently not implemented to reduce compile time - such huge models will not be used in practice.
	IndexedTriMeshN0<float,uint8_t>::Ptr mesh8;
	IndexedTriMeshN0<float,uint16_t>::Ptr mesh16;
	IndexedTriMeshN0<float,uint32_t>::Ptr mesh32;
	IndexedTriMeshF::Ptr mesh;

	if (totalVerts < UINT8_MAX)
		mesh = mesh8 = rw::common::ownedPtr( new IndexedTriMeshN0<float,uint8_t>() );
	else if (totalVerts < UINT16_MAX)
		mesh = mesh16 = rw::common::ownedPtr( new IndexedTriMeshN0<float,uint16_t>() );
	else if (totalVerts < UINT32_MAX)
		mesh = mesh32 = rw::common::ownedPtr( new IndexedTriMeshN0<float,uint32_t>() );
	else
		RW_THROW("The Model3D has too many vertices to be converted to GeometryData (IndexedTriMesh) - should be less than " << UINT32_MAX << "!");

	std::stack<std::pair<Object3D::Ptr,Transform3D<float> > > objects;
	BOOST_FOREACH(Object3D::Ptr obj, _objects){
		objects.push( std::make_pair(obj, Transform3D<float>::identity()) );
	}

	while(!objects.empty()){
		Object3D::Ptr obj = objects.top().first;
		Transform3D<float> t3d = objects.top().second * obj->_transform;
		objects.pop();

		BOOST_FOREACH(Object3D::Ptr child, obj->_kids){
			objects.push( std::make_pair(child, t3d) );
		}

		size_t csize = mesh->getVertices().size();
		size_t tsize = 0;

		size_t vsize = obj->_vertices.size();
		size_t fsize = obj->_faces.size();

		mesh->getVertices().resize( csize+vsize );
		if (totalVerts < UINT8_MAX) {
			tsize = mesh8->getTriangles().size();
			mesh8->resize( tsize+fsize );
		} else if (totalVerts < UINT16_MAX) {
			tsize = mesh16->getTriangles().size();
			mesh16->resize( tsize+fsize );
		} else if (totalVerts < UINT32_MAX) {
			tsize = mesh32->getTriangles().size();
			mesh32->resize( tsize+fsize );
		}
		// copy all from obj to mesh
		for(size_t i=0;i<vsize;i++){
			mesh->getVertices()[csize+i] = /*obj->_transform*/ t3d * obj->_vertices[i];
		}

		if (totalVerts < UINT8_MAX) {
			for(size_t i=0;i<fsize;i++){
				mesh8->getTriangles()[tsize+i][0] = static_cast<uint8_t>(obj->_faces[i][0])+static_cast<uint8_t>(csize);
				mesh8->getTriangles()[tsize+i][1] = static_cast<uint8_t>(obj->_faces[i][1])+static_cast<uint8_t>(csize);
				mesh8->getTriangles()[tsize+i][2] = static_cast<uint8_t>(obj->_faces[i][2])+static_cast<uint8_t>(csize);
			}
		} else if (totalVerts < UINT16_MAX) {
			for(size_t i=0;i<fsize;i++){
				mesh16->getTriangles()[tsize+i][0] = static_cast<uint16_t>(obj->_faces[i][0])+static_cast<uint16_t>(csize);
				mesh16->getTriangles()[tsize+i][1] = static_cast<uint16_t>(obj->_faces[i][1])+static_cast<uint16_t>(csize);
				mesh16->getTriangles()[tsize+i][2] = static_cast<uint16_t>(obj->_faces[i][2])+static_cast<uint16_t>(csize);
			}
		} else if (totalVerts < UINT32_MAX) {
			for(size_t i=0;i<fsize;i++){
				mesh32->getTriangles()[tsize+i][0] = static_cast<uint32_t>(obj->_faces[i][0])+static_cast<uint32_t>(csize);
				mesh32->getTriangles()[tsize+i][1] = static_cast<uint32_t>(obj->_faces[i][1])+static_cast<uint32_t>(csize);
				mesh32->getTriangles()[tsize+i][2] = static_cast<uint32_t>(obj->_faces[i][2])+static_cast<uint32_t>(csize);
			}
		}
	}
	return mesh;
}

void Model3D::addGeometry(const Material& mat, rw::geometry::Geometry::Ptr geom){
	TriMesh::Ptr mesh = geom->getGeometryData()->getTriMesh(false);
	addTriMesh(mat, *mesh);
}

void Model3D::addTriMesh(const Material& mat, const TriMesh& mesh){
    const size_t maxMeshSize = 21845; // we use 16 bit indexing, so with normal array at size = 3*nrTriangles we get (2^16)/3 allowed
	size_t nrObjects = (size_t)std::floor(mesh.size()/(maxMeshSize*1.0))+1;
	int matId = addMaterial( mat );

	for(size_t objNr = 0; objNr<nrObjects; objNr++){

		size_t meshSize = mesh.size()-maxMeshSize*objNr;
		if( meshSize > maxMeshSize)
			meshSize = maxMeshSize;

		Object3D::Ptr obj = rw::common::ownedPtr( new Object3D("MeshObj") );
		obj->_vertices.resize(meshSize*3);
		obj->_normals.resize(meshSize*3);
		obj->_faces.resize(meshSize);
		Triangle<float> tri;
		for(size_t i=0;i<meshSize;i++){
			mesh.getTriangle(maxMeshSize*objNr + i , tri);
			Vector3D<float> normal = tri.calcFaceNormal();
			obj->_vertices[i*3+0] = tri[0];
			obj->_vertices[i*3+1] = tri[1];
			obj->_vertices[i*3+2] = tri[2];
			obj->_normals[i*3+0] = normal;
			obj->_normals[i*3+1] = normal;
			obj->_normals[i*3+2] = normal;
			obj->_faces[i] = IndexedTriangle<uint16_t>((uint16_t)i*3+0,(uint16_t)i*3+1,(uint16_t)i*3+2);
		}
		obj->_materialMap.push_back( Object3D::MaterialMapData((uint16_t)matId,0,(uint16_t)meshSize) );

		_objects.push_back(obj);
	}
}


namespace {

    rw::math::Vector3D<float> calcNormal(Model3D::Object3D::Ptr& obj, size_t face1){
        IndexedTriangle<uint16_t> &gtri = obj->_faces[face1];
        Vector3D<float> &v0 = obj->_vertices[ gtri[0] ];
        Vector3D<float> &v1 = obj->_vertices[ gtri[1] ];
        Vector3D<float> &v2 = obj->_vertices[ gtri[2] ];
        return cross(v1-v0,v2-v0);
    }

    rw::math::Vector3D<float> calcGroupNormal(Model3D::Object3D::Ptr& obj, std::list<size_t>& faces, Model3D::SmoothMethod method){
        Vector3D<float> normal(0,0,0);
        BOOST_FOREACH(size_t face, faces){
            if( method==Model3D::AVERAGED_NORMALS ){
                normal += normalize( calcNormal(obj, face) );
            } else {
                normal += calcNormal(obj, face);
            }
        }
        return normalize( normal );
    }
}

void Model3D::optimize(double smooth_angle, SmoothMethod method){
    std::stack<Object3D::Ptr> objects;
    BOOST_FOREACH(Object3D::Ptr obj, _objects){ objects.push(obj); }

    while(!objects.empty()){
        Object3D::Ptr obj = objects.top();
        objects.pop();

        // push all children on stack
        BOOST_FOREACH(Object3D::Ptr kid, obj->_kids){
            objects.push(kid);
        }

        // 1. Merge close vertices if choosen
        // we create an indexed triangle mesh that is used to calculate a new mesh
        //std::cout << "Nr of faces: " << obj->_faces.size() << std::endl;
        if( obj->_faces.size()==0 )
            continue;

        IndexedTriMeshN0<float, uint16_t> triMesh(&obj->_vertices, &obj->_faces );

        IndexedTriMeshN0<float, uint16_t>::Ptr nmesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<float, uint16_t> >( triMesh );

        obj->_vertices = nmesh->getVertices();
        //Vector3D<float> prev(0,0,0);
        //BOOST_FOREACH(Vector3D<float> v, obj->_vertices){
        //    std::cout << v << "   " << MetricUtil::dist2(v,prev) << "\n";
        //}

        obj->_normals.resize( obj->_vertices.size() );
        obj->_faces = nmesh->getTriangles();

        // 2 pre, create a material face map
        //std::vector<uint16_t> faceToMaterial(obj->_faces.size());
        //BOOST_FOREACH(Object3D::MaterialMapData &data, obj->_materialMap){
        //    for(uint16_t i=data.startIdx;i<data.startIdx+data.size(); i++){
        //        faceToMaterial[i] = data.matId;
        //    }
        //}

        // 2. recalculate vertice normals, if two face normals are further apart than
        //    smooth_angle then we need to create a new vertice
        std::vector< std::list<size_t> > verticeToFaces( obj->_vertices.size() );
        for(size_t i=0; i<obj->_faces.size(); i++){
            verticeToFaces[obj->_faces[i][0]].push_back(i);
            verticeToFaces[obj->_faces[i][1]].push_back(i);
            verticeToFaces[obj->_faces[i][2]].push_back(i);
        }

        // 2.2 now, for each vertice we determine its normal
        //std::vector<> ;
        for(size_t i=0;i<verticeToFaces.size();i++){
            std::list<size_t> &vertfaces = verticeToFaces[i];
            // we need to seperate the faces into groups that have normals closer than smooth_angle
            std::vector< std::list<size_t> > groups;

            BOOST_FOREACH(size_t face, vertfaces){
                if(groups.size()==0){
                    groups.push_back( std::list<size_t>(1,face) );
                    continue;
                }
                Vector3D<float> facenormal = normalize( calcNormal(obj, face) );
                // compare this face with all groups, add it to the first group where it fits
                bool ingroup = false;
                BOOST_FOREACH(std::list<size_t>& group, groups){
                    BOOST_FOREACH(size_t groupface, group){
                    	//double angle = angle(normalize(calcNormal(obj, groupface)), normalize(facenormal));
                        double ang = std::acos( dot( normalize(calcNormal(obj, groupface)), facenormal) );
                    	if( smooth_angle>std::fabs(ang)){
                        	// the face should be put into this group
                            ingroup = true;

                            break;
                        }
                    }
                    if(ingroup){
                        group.push_back(face);
                        break;
                    }
                }
                // if not in group add a new one
                if(!ingroup)
                    groups.push_back(std::list<size_t>(1,face));
            }

            // the first group use the original vertice
            obj->_normals[i] = calcGroupNormal(obj, groups[0], method);

            // the following groups each get a new vertice
            for(size_t j=1;j<groups.size();j++){
                //std::cout << "ADDING VERTICES" << std::endl;
                size_t nidx = obj->_vertices.size();
                obj->_vertices.push_back( obj->_vertices[i] );
                obj->_normals.push_back( calcGroupNormal(obj, groups[j] , method) );
                // change all references to the vertice
                BOOST_FOREACH(size_t changeFace, groups[j]){
                    if( obj->_faces[changeFace][0] == i)
                        obj->_faces[changeFace][0] = (uint16_t)nidx;
                    if( obj->_faces[changeFace][1] == i)
                        obj->_faces[changeFace][1] = (uint16_t)nidx;
                    if( obj->_faces[changeFace][2] == i)
                        obj->_faces[changeFace][2] = (uint16_t)nidx;
                }
            }
        }
    }

}
