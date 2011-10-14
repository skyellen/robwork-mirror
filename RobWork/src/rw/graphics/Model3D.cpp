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
#include <boost/foreach.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <list>
#include <stack>


using namespace rw::graphics;
using namespace rw::math;
using namespace rw::geometry;

Model3D::Model3D(){}

Model3D::~Model3D(){}

int Model3D::addObject(Model3D::Object3D::Ptr obj){
    _objects.push_back(obj);
    return _objects.size()-1;
}

int Model3D::addMaterial(const Model3D::Material& mat){
    _materials.push_back(mat);
    return _materials.size()-1;
}

void Model3D::removeObject(const std::string& name){

}

void Model3D::addTriMesh(const Material& mat, const TriMesh& mesh){
    const int maxMeshSize = 16000; // we use 16 bit indexing
    if(mesh.size()>maxMeshSize){
        RW_WARN("SPLITTING LARGE TRIANGLE MESH: " << mesh.size());
    }

    int nrObjects = std::floor(mesh.size()/(maxMeshSize*1.0))+1;
    std::cout << "NR OBJECTS: " << nrObjects << std::endl;
    int matId = addMaterial( mat );

    for(int objNr = 0; objNr<nrObjects; objNr++){

        int meshSize = mesh.size();
        if( (mesh.size()-maxMeshSize*objNr) > maxMeshSize)
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
            obj->_faces[i] = IndexedTriangle<uint16_t>(i*3+0,i*3+1,i*3+2);
        }
        obj->_materialMap.push_back( Object3D::MaterialMapData(matId,0,meshSize) );

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
    BOOST_FOREACH(Object3D::Ptr obj, _objects){ objects.push(obj);}

    while(!objects.empty()){
    //BOOST_FOREACH(Object3D::Ptr obj, _objects){
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
                Vector3D<float> facenormal = calcNormal(obj, face);
                // compare this face with all groups, add it to the first group where it fits
                bool ingroup = false;
                BOOST_FOREACH(std::list<size_t>& group, groups){
                    BOOST_FOREACH(size_t groupface, group){
                        if( smooth_angle>angle(calcNormal(obj, groupface), facenormal)){
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
                size_t nidx = obj->_vertices.size();
                obj->_vertices.push_back( obj->_vertices[i] );
                obj->_normals.push_back( calcGroupNormal(obj, groups[j] , method) );
                // change all references to the vertice
                BOOST_FOREACH(size_t changeFace, groups[j]){
                    if( obj->_faces[changeFace][0] == i)
                        obj->_faces[changeFace][0] = nidx;
                    if( obj->_faces[changeFace][1] == i)
                        obj->_faces[changeFace][1] = nidx;
                    if( obj->_faces[changeFace][2] == i)
                        obj->_faces[changeFace][2] = nidx;
                }
            }
        }
    }
}
