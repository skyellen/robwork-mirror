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

#include "RenderSmoothSkin.hpp"

#include <rw/kinematics/Kinematics.hpp>

using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::math;
using namespace rwlibs::opengl;

/*
RenderSmoothSkin::RenderSmoothSkin(rw::geometry::IndexedTriMesh<>::Ptr mesh,
                                   rw::kinematics::Frame* base,
                                   std::vector<rw::kinematics::Frame*>& bones,
                                   std::vector<BoneWeights>& weights)
{
    init(mesh, base, bones, weights);
}

RenderSmoothSkin::~RenderSmoothSkin(){

}

void RenderSmoothSkin::init(IndexedTriMesh<>::Ptr mesh,
                       rw::kinematics::Frame* base,
                       std::vector<rw::kinematics::Frame*>& bones,
                       std::vector<BoneWeights>& weights)
{
    RW_ASSERT( bones.size()==weights.size() );
    // per vertex
    _mesh = mesh;
    _vertices = mesh->getVertices();

    std::vector<int> vertsRef(_vertices.size(), 0);
    std::vector<int> vertsIndex(_vertices.size(), -1); // index into the weights array

    // create the weighting structure
    // first run through the weights to estimate the size of the array
    size_t sum = 0;
    BOOST_FOREACH(BoneWeights& weight, weights){
        sum += weigth.size();
        BOOST_FOREACH(VerticeWeight& vw, weight){
            vertsRef[vw.first]++;
        }
    }
    sum *= sizeof(VerticeWeightINL);
    sum += _vertices.size();

    _weights.resize(sum);

    // now calculate the position of each weight in the weight array
    uint32_t idx=0, vidx=0;
    BOOST_FOREACH(int nrWeights, vertsRef){
        vidx++;
        //if(nrWeights>0){
            vertsIndex[vidx] = idx;
            idx++;
            idx += nrWeights*sizeof(VerticeWeightINL);
        //}
    }
    std::cout << sum << "=="<< idx << std::endl;
    RW_ASSERT( sum==idx );

    // now add the relevant entries to the array
    uint32_t idx = 0, vidx = 0;

    while( idx<_weights.size() ){
        uint8_t wcount = _weights[idx];
        const Vector3D<> v = _vertices[vidx];
        Vector3D<> vnew(0,0,0);
        idx++;
        for(size_t i=0; i<wcount; i++){
            VerticeWeightINL *vw = ((VerticeWeightINL*)&_weights[idx]);
            vnew += vw->weight * (_transforms[vw->boneIdx]*v);
            idx += sizeof(VerticeWeightINL);
        }
        // set the newly computed vertice in the tri mesh
        _mesh->getVertices()[vidx] = vnew;
        vidx++;
    }


}

void RenderSmoothSkin::update(const rw::kinematics::State& state){
    // first update all transforms
    for(size_t i=0; i<_bones.size(); i++){
        _transforms[i] = Kinematics::frameTframe( _base, _bones[i], state);
    }

    // for each vertice we apply the transforms according to the weight
    uint32_t idx = 0, vidx = 0;

    while( idx<_weights.size() ){
        uint8_t wcount = _weights[idx];
        const Vector3D<> v = _vertices[vidx];
        Vector3D<> vnew(0,0,0);
        idx++;
        for(size_t i=0; i<wcount; i++){
            VerticeWeightINL *vw = ((VerticeWeightINL*)&_weights[idx]);
            vnew += vw->weight * (_transforms[vw->boneIdx]*v);
            idx += sizeof(VerticeWeightINL);
        }
        // set the newly computed vertice in the tri mesh
        _mesh->getVertices()[vidx] = vnew;
        vidx++;
    }
}

void RenderSmoothSkin::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const{
    // render the mesh

}

*/