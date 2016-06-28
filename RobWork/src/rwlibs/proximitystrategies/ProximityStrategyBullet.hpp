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


#ifndef RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYBULLET_HPP
#define RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYBULLET_HPP

/**
 * @file ProximityStrategyBullet.hpp
 */
#include <rw/common/Cache.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionToleranceStrategy.hpp>
#include <rw/proximity/DistanceStrategy.hpp>
#include <rw/proximity/DistanceToleranceStrategy.hpp>
#include <rw/proximity/DistanceThresholdStrategy.hpp>
#include <rw/proximity/ProximityCache.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>
#include <rw/geometry/GeometryData.hpp>

#include <btBulletCollisionCommon.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

#include <map>
#include <vector>
#include <list>


namespace rwlibs { namespace proximitystrategies {
    /** @addtogroup proximitystrategies */
    /*@{*/

    /**
     * @brief a cache for the PQP proximity strategy
     */
/*
	class BulletCollisionCache: public rw::proximity::ProximityCache {
	public:
		BulletCollisionCache(rw::proximity::ProximityStrategy *owner):
				rw::proximity::ProximityCache(owner){};

		//! @copydoc rw::proximity::ProximityCache::size()
		size_t size() const{ return 0;}

		//! @copydoc rw::proximity::ProximityCache::clear()
		void clear() { };

		//PQP::PQP_CollideResult result;
	};
*/
    /**
     * @brief This is a strategy wrapper for the distance and collision library
     * Gimpact from the Bullet physicsengine
     *
     */
    class ProximityStrategyBullet :
        public rw::proximity::CollisionStrategy,
        public rw::proximity::CollisionToleranceStrategy,
        public rw::proximity::DistanceStrategy,
        public rw::proximity::DistanceToleranceStrategy
    {
    public:

        typedef rw::common::Ptr<ProximityStrategyBullet> Ptr;
        //! @brief cache key
        typedef std::pair<rw::geometry::GeometryData*,double> CacheKey;

        //! @brief cache for any of the queries possible on this PQPStrategy
        struct ProximityCacheBullet: public rw::proximity::ProximityCache{
            ProximityCacheBullet(void *owner):ProximityCache(owner){}
            virtual size_t size() const{ return 0;};
            virtual void clear(){};


            btCollisionDispatcher *dispatcher;
            btGImpactCollisionAlgorithm *gimpactColAlg;
            //PQP::PQP_ToleranceResult _toleranceResult;
            //PQP::PQP_CollideResult _collideResult;
            //PQP::PQP_DistanceResult _distResult;
            //PQP::PQP_MultiDistanceResult _multiDistResult;
        };

        //! @brief
        struct BulletModel {
            BulletModel(std::string id, rw::math::Transform3D<> trans, rw::common::Ptr<btCollisionObject> model):
                geoid(id),t3d(trans),model(model){}
            std::string geoid;
            rw::math::Transform3D<> t3d;
            rw::common::Ptr<btCollisionObject> model;
            CacheKey ckey;
        };

        struct ProximityModelBullet : public rw::proximity::ProximityModel {
            ProximityModelBullet(ProximityStrategy *owner):
                ProximityModel(owner)
            {
            }
            std::vector< BulletModel >  models;
        };

    private:
        //bool _firstContact;
        rw::common::Cache<CacheKey, btCollisionShape> _modelCache;

    public:
        /**
         * @brief Constructor
         */
        ProximityStrategyBullet();

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
         * @copydoc rw::proximity::ProximityStrategy::addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom)
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
         * @copydoc rw::proximity::CollisionStrategy::collision
         */
        bool doInCollision(
			rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            rw::proximity::ProximityStrategyData &data);

        /**
         * @copydoc rw::proximity::CollisionStrategy::collision
         */
/*
        bool collides(
			rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            rw::proximity::CollisionData& data);
*/
        /**
         * @copydoc rw::proximity::CollisionToleranceStrategy::inCollision
         */
        bool doInCollisionTolerance(
			rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            double tolerance,
            rw::proximity::ProximityStrategyData &data);

        /**
         * @copydoc rw::proximity::DistanceStrategy::distance
         */
        rw::proximity::DistanceStrategy::Result& doDistance(
			rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            rw::proximity::ProximityStrategyData &data);

        /**
         * @copydoc rw::proximity::DistanceThresholdStrategy::distance
         */
        rw::proximity::DistanceStrategy::Result& doDistanceThreshold(
            rw::proximity::ProximityModel::Ptr aModel,
            const rw::math::Transform3D<>& wTa,
            rw::proximity::ProximityModel::Ptr bModel,
            const rw::math::Transform3D<>& wTb,
            double threshold,
            rw::proximity::ProximityStrategyData &data);

        /**
         * @copydoc rw::proximity::DistanceToleranceStrategy::getDistances
         */
        rw::proximity::DistanceToleranceStrategy::Result& distances(
			rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            double tolerance,
            rw::proximity::ProximityStrategyData &data);

        /**
         *  @copydoc rw::proximity::ProximityStrategy::clear
         */
        void clear();

        /**
           @brief A PQP based collision strategy.
        */
		static rw::proximity::CollisionStrategy::Ptr make();

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


        //! @copydoc rw::proximity::CollisionStrategy::getCollisionContacts
		void getCollisionContacts(std::vector<CollisionStrategy::Contact>& contacts,
											  rw::proximity::ProximityStrategyData& data);



		void setThreshold(double threshold);

		std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> >
			getSurfaceNormals( rw::proximity::MultiDistanceResult& res, int idx);

    private:

        struct QueryData {
            ProximityCacheBullet *cache;
            ProximityModelBullet *a, *b;
        };

        QueryData initQuery(rw::proximity::ProximityModel::Ptr& aModel,
                            rw::proximity::ProximityModel::Ptr& bModel,
                            rw::proximity::ProximityStrategyData &data);
    private:
    	int _numBVTests,_numTriTests;

    	btCollisionWorld *_cworld;
    	btCollisionDispatcher *_dispatcher;
    	btGImpactCollisionAlgorithm *_gimpactColAlg;
    	btCollisionConfiguration *_collisionConfiguration;

    	std::vector< BulletModel > _allmodels;
    	std::map<std::string, std::vector<int> > _geoIdToModelIdx;
		double _threshold;

	};

    //static const bool PQPCollisionStrategyRegistrered = rw::proximity::ProximityStrategyFactory::addCollisionStrategy<ProximityStrategyPQP>("PQP");
    //static const bool PQPDistanceStrategyRegistrered = rw::proximity::ProximityStrategyFactory::addDistanceStrategy<ProximityStrategyPQP>("PQP");

}} // end namespaces

#endif // end include guard
