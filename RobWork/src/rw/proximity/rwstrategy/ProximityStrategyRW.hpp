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

#ifndef RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYRW_HPP
#define RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYRW_HPP

/**
 * @file ProximityStrategyRW.hpp
 */

#include <vector>

#include <rw/common/Cache.hpp>

//#include <rw/proximity/CollisionData.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionToleranceStrategy.hpp>
#include <rw/proximity/DistanceStrategy.hpp>

#include <rw/proximity/ProximityCache.hpp>

#include "BinaryBVTree.hpp"
#include "BVTreeCollider.hpp"

#include <rw/geometry/OBBToleranceCollider.hpp>

//#include "RSSDistanceCalc.hpp"


namespace rw { namespace proximity {
    /** @addtogroup proximity */
    /*@{*/
/*
	class CollisionCache: public rw::proximity::ProximityCache {
	public:
		CollisionCache(rw::proximity::ProximityStrategy *owner):
				rw::proximity::ProximityCache(owner){};

		size_t size() const{ return result.num_pairs_alloced;}
		void clear() { result.FreePairsList(); };

		PQP::PQP_CollideResult result;
	};
*/
    /**
     * @brief This is a strategy wrapper for the distance library
     * PQP (Proximity Query Package).
     *
     * PQP use Oriented Bounding Boxes (OBB) and hierarchical bounding trees for
     * fast distance calculation.
     *
     * For further information check out http://www.cs.unc.edu/~geom/SSV/
     */
    class ProximityStrategyRW :
        public rw::proximity::CollisionStrategy,
        public rw::proximity::CollisionToleranceStrategy,
        public rw::proximity::DistanceStrategy
        //public rw::proximity::DistanceToleranceStrategy
        //public rw::proximity::DistanceThresholdStrategy

    {
    public:
        typedef rw::common::Ptr<ProximityStrategyRW> Ptr;

        //! @brief cache key
        typedef std::pair<std::string, double> CacheKey;

        //! @brief cache for any of the queries possible on this strategy
        struct PCache: public rw::proximity::ProximityCache{
            PCache(void *owner):ProximityCache(owner){}
            virtual size_t size() const{ return 0;};
            virtual void clear(){};

            // TODO: reuse stuff from the collision test
            rw::common::Ptr<rw::proximity::BVTreeCollider<rw::proximity::BinaryOBBPtrTreeD > > tcollider;
            rw::common::Ptr<rw::proximity::BVTreeCollider<rw::proximity::BinaryOBBPtrTreeD > > tolcollider;
            rw::geometry::OBBToleranceCollider<> *tolCollider;

        };

        //! @brief

        struct Model {
            typedef rw::common::Ptr<Model > Ptr;

            Model(std::string id, rw::math::Transform3D<> trans, rw::proximity::BinaryOBBPtrTreeD::Ptr obbtree):
                geoid(id),t3d(trans),tree(obbtree){}

            std::string geoid;
            double scale;
            rw::math::Transform3D<> t3d;
            rw::proximity::BinaryOBBPtrTreeD::Ptr tree;
            CacheKey ckey;
        };

        //typedef std::vector<RWPQPModel> RWPQPModelList;
        //typedef std::pair<RWPQPModel, RWPQPModel> RWPQPModelPair;
        struct RWProximityModel : public rw::proximity::ProximityModel {
            RWProximityModel(ProximityStrategy *owner):
                ProximityModel(owner)
            {
            }
            std::vector<Model::Ptr> models;
        };

    private:
        rw::common::Cache<CacheKey, Model> _modelCache;

    public:
        /**
         * @brief Constructor
         */
        ProximityStrategyRW();

        //// interface of ProximityStrategy

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

        bool addGeometry(rw::proximity::ProximityModel* model, rw::common::Ptr<rw::geometry::Geometry> geom, bool);

        /**
         * @copydoc rw::proximity::ProximityStrategy::removeGeometry
         */
        bool removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId);

        /**
         * @copydoc rw::proximity::ProximityStrategy::getGeometryIDs
         */
        std::vector<std::string> getGeometryIDs(rw::proximity::ProximityModel* model);

        /**
         * @copydoc rw::proximity::CollisionStrategy::setFirstContact
         */
        void setFirstContact(bool b);

        /**
         * @copydoc rw::proximity::CollisionStrategy::collision
         */
        bool doInCollision(
			rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            rw::proximity::ProximityStrategyData &data);

        /**
         * @copydoc rw::proximity::CollisionToleranceStrategy::doIsWithinDistance
         */
        bool doIsWithinDistance(
            rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
            rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            double tolerance,
            rw::proximity::ProximityStrategyData &data);

        /**
         * @copydoc rw::proximity::DistanceStrategy::doDistance
         */
        DistanceStrategy::Result& doDistance(
            rw::proximity::ProximityModel::Ptr a,
            const math::Transform3D<>& wTa,
            rw::proximity::ProximityModel::Ptr b,
            const math::Transform3D<>& wTb,
            rw::proximity::ProximityStrategyData& data);


        /**
         *  @copydoc rw::proximity::ProximityStrategy::clear
         */
        void clear();

        /**
         * @brief returns the number of bounding volume tests performed
         * since the last call to clearStats
         */
        int getNrOfBVTests(){return _numBVTests;};

        /**
         * @brief returns the number of ptriangle tests performed
         * since the last call to clearStats
         */
        int getNrOfTriTests(){return _numTriTests;};

        /**
         * @brief clears the bounding volume and triangle test counters.
         */
        void clearStats(){ _numBVTests = 0; _numTriTests = 0;};

		void getCollisionContacts(std::vector<CollisionStrategy::Contact>& contacts,
											  ProximityStrategyData& data);

    private:

        struct QueryData {
           PCache *cache;
           RWProximityModel *a, *b;
        };

        QueryData initQuery(rw::proximity::ProximityModel::Ptr& aModel,
                            rw::proximity::ProximityModel::Ptr& bModel,
                            rw::proximity::ProximityStrategyData &data);
    private:

    	int _numBVTests,_numTriTests;

    	rw::proximity::BVTreeCollider<rw::proximity::BinaryOBBPtrTreeD>::Ptr _tcollider;
    	std::vector<Model::Ptr> _allModels;
    };

}} // end namespaces

#endif // end include guard
