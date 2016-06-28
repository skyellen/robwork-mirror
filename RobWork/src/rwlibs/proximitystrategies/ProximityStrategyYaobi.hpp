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


#ifndef RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYYAOBI_HPP
#define RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYYAOBI_HPP

/**
 * @file ProximityStrategyYaobi.hpp
 */

#include <map>
#include <vector>
#include <list>

#include <yaobi/yaobi.h>
#include <rw/common/Ptr.hpp>
#include <rw/common/Cache.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
//#include <rw/proximity/ProximityStrategyFactory.hpp>

namespace rwlibs { namespace proximitystrategies {
    /** @addtogroup proximitystrategies */
    /*@{*/

    /**
       @brief This is a strategy wrapper for the collision library Yaobi.

       Yaobi use Oriented Bounding Boxes (OBB) and hierachical bounding trees
       for fast collision detection between triangulated objects.

       For further information check out http://sourceforge.net/projects/yaobi
    */
    class ProximityStrategyYaobi:
        public rw::proximity::CollisionStrategy
    {
        typedef rw::common::Ptr<yaobi::CollModel> YaobiModelPtr;
        typedef std::pair<rw::math::Transform3D<>, YaobiModelPtr> RWYaobiModel;
        typedef std::vector<RWYaobiModel> RWYaobiModelList;
        typedef std::pair<RWYaobiModel, RWYaobiModel> RWYaobiModelPair;

        struct YaobiProximityModel : public rw::proximity::ProximityModel {
            YaobiProximityModel(ProximityStrategy *owner):
                ProximityModel(owner)
            {
            }
            RWYaobiModelList models;
        };


    private:
        rw::common::Cache<std::string, yaobi::CollModel> _modelCache;
        std::vector<RWYaobiModel> _allmodels;
        std::map<std::string, std::vector<int> > _geoIdToModelIdx;

    public:
        /**
         * @brief Constructor
         */
        ProximityStrategyYaobi();

        /**
         * @copydoc rw::proximity::ProximityStrategy::createModel
         */
		virtual rw::proximity::ProximityModel::Ptr createModel();

        /**
         * @copydoc rw::proximity::ProximityStrategy::destroyModel
         */
        void destroyModel(rw::proximity::ProximityModel* model);

        /**
         * @copydoc rw::proximity::ProximityStrategy::addGeometry
         */
        bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom);

        //! @copydoc rw::proximity::ProximityStrategy::addGeometry(ProximityModel* model, rw::common::Ptr<rw::geometry::Geometry> geom, bool forceCopy=false)
        bool addGeometry(rw::proximity::ProximityModel* model, rw::common::Ptr<rw::geometry::Geometry> geom, bool forceCopy=false);

        /**
         * @copydoc rw::proximity::ProximityStrategy::removeGeometry
         */
        bool removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId);

        /**
         * @copydoc rw::proximity::ProximityStrategy::getGeometryIDs
         */
        std::vector<std::string> getGeometryIDs(rw::proximity::ProximityModel* model);

        /**
         *  @copydoc rw::proximity::ProximityStrategy::clear
         */
        void clear();

        //! @copydoc rw::proximity::CollisionStrategy::getCollisionContacts
		void getCollisionContacts(std::vector<CollisionStrategy::Contact>& contacts,
											  rw::proximity::ProximityStrategyData& data);


        /**
           @brief Makes a Yaobi based collision strategy.
        */
		static rw::proximity::CollisionStrategy::Ptr make();

    protected:
        /**
         * @copydoc rw::proximity::CollisionStrategy::doInCollision
         */
        bool doInCollision(
            rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
            rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            rw::proximity::ProximityStrategyData& data);

    };

    //static const bool YaobiCollisionStrategyRegistrered = rw::proximity::ProximityStrategyFactory::addCollisionStrategy<ProximityStrategyYaobi>("YAOBI");

}} // end namespaces

#endif // end include guard
