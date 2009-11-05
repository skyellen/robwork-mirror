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


#include "ProximityStrategyYaobi.hpp"

#include <cfloat>
#include <vector>

#include <rw/geometry/Face.hpp>
#include <rw/geometry/FaceArrayFactory.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Exception.hpp>
#include <rw/models/Accessor.hpp>

#include <boost/foreach.hpp>

#include <yaobi/yaobi_mesh_interface.h>
#include <yaobi/yaobi_tree_builder.h>

using namespace rw::common;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

using namespace rwlibs::proximitystrategies;

//----------------------------------------------------------------------
// Utilities

namespace
{
    Ptr<yaobi::CollModel> makeModelFromSoup(
        const std::vector<Face<float> > &faceList)
    {
        unsigned char tri_stride(3);
        unsigned num_tris(faceList.size());
        unsigned num_verts(num_tris*3);
        yaobi::AppRealT *vertices = new yaobi::AppRealT[num_verts*3];
        int *tris = new int[num_tris*3];

        for(unsigned triIdx = 0, vertIdx = 0;
            triIdx < num_tris;
            triIdx++, vertIdx += 3)
        {
            const Face<float>& face = faceList[triIdx];
            for (size_t j=0; j < 3; j++) {
                vertices[vertIdx*3+0+j] = face._vertex1[j];
                vertices[vertIdx*3+3+j] = face._vertex2[j];
                vertices[vertIdx*3+6+j] = face._vertex3[j];
            }
            tris[vertIdx+0] = vertIdx+0;
            tris[vertIdx+1] = vertIdx+1;
            tris[vertIdx+2] = vertIdx+2;
        }

        yaobi::TriMeshInterface* tri = new yaobi::TriMeshInterface(
            num_verts, vertices,
            num_tris, tris,
            tri_stride,
            yaobi::OWN_DATA);

        Ptr<yaobi::CollModel> model = ownedPtr(new yaobi::CollModel(tri, yaobi::OWN_DATA));
											   yaobi::build_obb_tree( *model, yaobi::OWN_DATA );

        //model->ShrinkToFit();
        return model;
    }

    // Convert Transform3D to Yaobi representation.
    void toTransform(
        const Transform3D<>& tr,
        yaobi::Real T[3][4])
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 4; j++)
                T[i][j] = static_cast<yaobi::Real>(tr(i, j));
    }

    void collide(
        const yaobi::CollModel& ma, const Transform3D<>& wTa,
        const yaobi::CollModel& mb, const Transform3D<>& wTb,
        yaobi::CollideResult& result)
    {
        yaobi::Real ta[3][4];
        yaobi::Real tb[3][4];

        toTransform(wTa, ta);
        toTransform(wTb, tb);

        Collide(result, ta, ma, tb, mb, yaobi::FIRST_CONTACT_ONLY);
    }
}

//----------------------------------------------------------------------
// ProximityStrategyYaobi

ProximityStrategyYaobi::ProximityStrategyYaobi()
{}


rw::proximity::ProximityModelPtr ProximityStrategyYaobi::createModel()
{
    YaobiProximityModel *model = new YaobiProximityModel(this);
    return ownedPtr(model);
}

void ProximityStrategyYaobi::destroyModel(rw::proximity::ProximityModelPtr model){
    RW_ASSERT(model!=NULL);
    //YaobiProximityModel *pmodel = (YaobiProximityModel*) model.get();

}

bool ProximityStrategyYaobi::addGeometry(rw::proximity::ProximityModelPtr model, const rw::geometry::Geometry& geom){
    RW_ASSERT(model!=NULL);
    YaobiProximityModel *pmodel = (YaobiProximityModel*) model.get();
    YaobiModelPtr yaobimodel;
    // first check if model is in cache
    if( _modelCache.has(geom.getId()) ){
        yaobimodel = _modelCache.get(geom.getId());
    } else {
        const std::vector<rw::geometry::Face<float> > &faceList = geom.getFaces();
        if(faceList.size()==0)
            return false;
        yaobimodel = makeModelFromSoup( faceList );
    }
    pmodel->models.push_back( RWYaobiModel(geom.getTransform(), yaobimodel) );

    _allmodels.push_back(pmodel->models.back());
    _geoIdToModelIdx[geom.getId()].push_back(_allmodels.size()-1);
    return true;
}

bool ProximityStrategyYaobi::removeGeometry(rw::proximity::ProximityModelPtr model, const std::string& geomId){
	return false;
}

bool ProximityStrategyYaobi::collides(
    ProximityModelPtr aModel,
    const Transform3D<>& wTa,
    ProximityModelPtr bModel,
    const Transform3D<>& wTb)
{
    YaobiProximityModel *a = (YaobiProximityModel*)aModel.get();
    YaobiProximityModel *b = (YaobiProximityModel*)bModel.get();

    BOOST_FOREACH(const RWYaobiModel& ma, a->models) {
        BOOST_FOREACH(const RWYaobiModel& mb, b->models) {
            yaobi::CollideResult result;
            collide(
                *ma.second, wTa * ma.first,
                *mb.second, wTb * mb.first,
                result);

            if (result.IsColliding()) return true;
        }
    }

    return false;
}

void ProximityStrategyYaobi::clear()
{
	// TODO: also clear cache
    //_frameModelMap.clear();
    _allmodels.clear();
    _modelCache.clear();
    clearFrames();
}

void ProximityStrategyYaobi::setFirstContact(bool b)
{
    _firstContact = b;
}

CollisionStrategyPtr ProximityStrategyYaobi::make()
{
    return ownedPtr(new ProximityStrategyYaobi);
}
