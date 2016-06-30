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


#ifndef RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYPQP_HPP
#define RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYPQP_HPP

/**
 * @file ProximityStrategyPQP.hpp
 */

#include <map>
#include <vector>

#include <rw/common/Cache.hpp>

//#include <rw/proximity/CollisionData.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionToleranceStrategy.hpp>
#include <rw/proximity/DistanceStrategy.hpp>
#include <rw/proximity/DistanceMultiStrategy.hpp>
#include <rw/proximity/ProximityCache.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>

#include <PQP/PQP.h>

namespace rw { namespace geometry { class GeometryData; } }

namespace PQP { class PQP_Model; }

namespace rwlibs { namespace proximitystrategies {
    /** @addtogroup proximitystrategies */
    /*@{*/

    /**
     * @brief a cache for the PQP proximity strategy
     */
	class PQPCollisionCache: public rw::proximity::ProximityCache {
	public:
		PQPCollisionCache(rw::proximity::ProximityStrategy *owner):
				rw::proximity::ProximityCache(owner){};

		//! @copydoc rw::proximity::ProximityCache::size()
		size_t size() const{ return result.num_pairs_alloced;}

		//! @copydoc rw::proximity::ProximityCache::clear()
		void clear() { result.FreePairsList(); };

		PQP::PQP_CollideResult result;
	};

    /**
     * @brief This is a strategy wrapper for the distance library
     * PQP (Proximity Query Package).
     *
     * PQP use Oriented Bounding Boxes (OBB) and hierarchical bounding trees for
     * fast distance calculation.
     *
     * For further information check out http://www.cs.unc.edu/~geom/SSV/
     */
    class ProximityStrategyPQP :
        public rw::proximity::CollisionStrategy,
        public rw::proximity::CollisionToleranceStrategy,
        public rw::proximity::DistanceStrategy,
        public rw::proximity::DistanceMultiStrategy
    {
    public:

        typedef rw::common::Ptr<ProximityStrategyPQP> Ptr;
        //! @brief cache key
        typedef std::pair<rw::geometry::GeometryData*,double> CacheKey;
        //! @brief smart pointer to PQP model
        typedef rw::common::Ptr<PQP::PQP_Model> PQPModelPtr;

        //! @brief cache for any of the queries possible on this PQPStrategy
        struct PQPProximityCache: public rw::proximity::ProximityCache {
            PQPProximityCache(void *owner):ProximityCache(owner){}
            virtual size_t size() const{ return 0;};
            virtual void clear(){};

            PQP::PQP_ToleranceResult _toleranceResult;
            PQP::PQP_CollideResult _collideResult;
            PQP::PQP_DistanceResult _distResult;
            PQP::PQP_MultiDistanceResult _multiDistResult;
        };

        //! @brief
        struct RWPQPModel {
            RWPQPModel(std::string id, rw::math::Transform3D<> trans, PQPModelPtr model):
                geoid(id),t3d(trans),pqpmodel(model){}
            std::string geoid;
            rw::math::Transform3D<> t3d;
            PQPModelPtr pqpmodel;
            CacheKey ckey;
        };

        typedef std::vector<RWPQPModel> RWPQPModelList;
        typedef std::pair<RWPQPModel, RWPQPModel> RWPQPModelPair;

        struct PQPProximityModel : public rw::proximity::ProximityModel {
            PQPProximityModel(ProximityStrategy *owner):
                ProximityModel(owner)
            {
            }
            RWPQPModelList models;
        };

    private:
        //bool _firstContact;
        rw::common::Cache<CacheKey, PQP::PQP_Model> _modelCache;

    public:
        /**
         * @brief Constructor
         */
        ProximityStrategyPQP();

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
         * @copydoc rw::proximity::CollisionStrategy::setFirstContact
         */
        void setFirstContact(bool b);

        /**
         * @copydoc rw::proximity::CollisionStrategy::inCollision
         */
        bool doInCollision(
			rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            rw::proximity::ProximityStrategyData &data);

        /**
         * @copydoc rw::proximity::CollisionStrategy::getCollisionContacts
         */
		void getCollisionContacts(
				std::vector<CollisionStrategy::Contact>& contacts,
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
        rw::proximity::DistanceStrategy::Result& doDistance(
			rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            rw::proximity::ProximityStrategyData &data);

        /**
         * @copydoc rw::proximity::DistanceThresholdStrategy::doDistanceThreshold
         */
        rw::proximity::DistanceStrategy::Result& doDistanceThreshold(
            rw::proximity::ProximityModel::Ptr aModel,
            const rw::math::Transform3D<>& wTa,
            rw::proximity::ProximityModel::Ptr bModel,
            const rw::math::Transform3D<>& wTb,
            double threshold,
            rw::proximity::ProximityStrategyData &data);

        /**
         * @copydoc rw::proximity::DistanceMultiStrategy::doDistances
         */
        rw::proximity::DistanceMultiStrategy::Result& doDistances(
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

		void setThreshold(double threshold);

		std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> >
			getSurfaceNormals( rw::proximity::MultiDistanceResult& res, int idx);

    private:

        struct QueryData {
            PQPProximityCache *cache;
            PQPProximityModel *a, *b;
        };

        QueryData initQuery(rw::proximity::ProximityModel::Ptr& aModel,
                            rw::proximity::ProximityModel::Ptr& bModel,
                            rw::proximity::ProximityStrategyData &data);
    private:
    	int _numBVTests,_numTriTests;

    	std::vector<RWPQPModel> _allmodels;
    	std::map<std::string, std::vector<int> > _geoIdToModelIdx;
		double _threshold;

	};

    //static const bool PQPCollisionStrategyRegistrered = rw::proximity::ProximityStrategyFactory::addCollisionStrategy<ProximityStrategyPQP>("PQP");
    //static const bool PQPDistanceStrategyRegistrered = rw::proximity::ProximityStrategyFactory::addDistanceStrategy<ProximityStrategyPQP>("PQP");

}} // end namespaces

#endif // end include guard
