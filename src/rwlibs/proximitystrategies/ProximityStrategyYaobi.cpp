/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

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
    std::auto_ptr<yaobi::CollModel> makeModelFromSoup(
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

        std::auto_ptr<yaobi::CollModel> model(
            new yaobi::CollModel(tri, yaobi::OWN_DATA));
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

    std::auto_ptr<yaobi::CollModel> makeModel(const CollisionModelInfo& info)
    {
        typedef std::auto_ptr<yaobi::CollModel> T;
        if (info.getId() == "")
            return T(NULL);

        std::vector<Face<float> > faceList;

        try {
            if (FaceArrayFactory::getFaceArray(info.getId(), faceList)) {
            	return makeModelFromSoup(faceList);
            } else {
                RW_WARN(
                    "Can not obtain triangles from: " <<
                    StringUtil::quote(info.getId()));
            }
        }
        catch (const Exception& exp) {
            RW_WARN(
                "Failed constructing collision model with message: "
                << exp.getMessage().getText());
        }
        return T(NULL);;
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

ProximityStrategyYaobi::ProximityStrategyYaobi() :
    _frameModelMap(200)
{}

bool ProximityStrategyYaobi::hasModel(const Frame* frame)
{
    if (!_frameModelMap.has(*frame)) {
        const std::vector<CollisionModelInfo>* infos =
            Accessor::collisionModelInfo().getPtr(*frame);
        return infos && !infos->empty();
    } else
        return true;
}

bool ProximityStrategyYaobi::addModel(
    const Frame *frame,
    const std::vector< Face<float> > &faces)
{
	// Construct the new model and add it to the model list
    yaobi::CollModel *model = makeModelFromSoup(faces).release();
    if (!model) return false;

    // add it to the list so it will be cleaned up later
    SharedModel sharedModel = ownedPtr(model);

    // update the modelMap
    _frameModelMap[*frame].push_back(
        ColModel(
            Transform3D<>::identity(),
            sharedModel));
    return true;
}

bool ProximityStrategyYaobi::addModel(const Frame* frame)
{
	const std::vector<CollisionModelInfo>* infos =
        Accessor::collisionModelInfo().getPtr(*frame);

    if (!infos) return false;

	BOOST_FOREACH(const CollisionModelInfo &info, *infos) {
        ModelList& seq = _frameModelMap[*frame];

		if (_modelCache.isInCache(info.getId())) {
			const SharedModel sharedModel = _modelCache.get(info.getId());
			seq.push_back(ColModel(info.getTransform(), sharedModel));
			continue;
		}

		yaobi::CollModel* yaobiModel = makeModel(info).release();
		if (!yaobiModel) continue;

		_modelCache.add(info.getId(), yaobiModel);
		SharedModel sharedModel = _modelCache.get(info.getId());
		seq.push_back(ColModel(info.getTransform(), sharedModel));
	}

	return true;
}

const ProximityStrategyYaobi::ModelList&
ProximityStrategyYaobi::getModels(const Frame* frame)
{
    const bool hasList = _frameModelMap.has(*frame);
    ModelList& result = _frameModelMap[*frame];
    if (!hasList) addModel(frame);
    return result;
}

bool ProximityStrategyYaobi::inCollision(
    const Frame* a,
    const Transform3D<>& wTa,
    const Frame* b,
    const Transform3D<>& wTb)
{
    const ModelList& modelsA = getModels(a);
    const ModelList& modelsB = getModels(b);

    BOOST_FOREACH(const ColModel& ma, modelsA) {
        BOOST_FOREACH(const ColModel& mb, modelsB) {

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
    _frameModelMap.clear();
}

void ProximityStrategyYaobi::clearFrame(const Frame* frame)
{
    _frameModelMap[*frame].clear();
}

void ProximityStrategyYaobi::setFirstContact(bool b)
{
    _firstContact = b;
}

CollisionStrategyPtr ProximityStrategyYaobi::make()
{
    return ownedPtr(new ProximityStrategyYaobi);
}
