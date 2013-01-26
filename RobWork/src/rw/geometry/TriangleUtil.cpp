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


#include "TriangleUtil.hpp"

using namespace rw::geometry;
using namespace rw::math;

/*
rw::common::Ptr<TriMesh> TriangleUtil::offsetSurface(const TriMesh& triMesh, double dist)
{
    return nmesh;
}
*/
/*
rw::common::Ptr<TriMesh> TriangleUtil::recalcNormals(const TriMesh& triMesh)
{
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
                    obj->_faces[changeFace][0] = nidx;
                if( obj->_faces[changeFace][1] == i)
                    obj->_faces[changeFace][1] = nidx;
                if( obj->_faces[changeFace][2] == i)
                    obj->_faces[changeFace][2] = nidx;
            }
        }
    }


}
*/
