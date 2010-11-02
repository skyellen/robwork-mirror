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
#include <list>

#include <boost/shared_ptr.hpp>

#include <rw/common/Cache.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/proximity/CollisionData.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionToleranceStrategy.hpp>
#include <rw/proximity/DistanceStrategy.hpp>
#include <rw/proximity/DistanceToleranceStrategy.hpp>
#include <rw/proximity/DistanceThresholdStrategy.hpp>

#include <rw/proximity/ProximityCache.hpp>

#include <PQP/PQP.h>

namespace PQP { class PQP_Model; }

namespace rwlibs { namespace proximitystrategies {
    /** @addtogroup proximitystrategies */
    /*@{*/

	class PQPCollisionCache: public rw::proximity::ProximityCache {
	public:
		PQPCollisionCache(rw::proximity::ProximityStrategy *owner):
				rw::proximity::ProximityCache(owner){};

		size_t size() const{ return result.num_pairs_alloced;}
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
        public rw::proximity::DistanceToleranceStrategy
    {
    public:
        //! @brief cache key
        typedef std::pair<rw::geometry::GeometryData*,double> CacheKey;
        //! @brief smart pointer to PQP model
        typedef rw::common::Ptr<PQP::PQP_Model> PQPModelPtr;


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
        bool _firstContact;
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
         * @copydoc rw::proximity::ProximityStrategy::addGeometry
         */
        bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom);

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
        bool collides(
			rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb);

        /**
         * @copydoc rw::proximity::CollisionStrategy::collision
         */
        bool collides(
			rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            rw::proximity::CollisionData& data);

        /**
         * @copydoc rw::proximity::CollisionToleranceStrategy::inCollision
         */
        bool collides(
			rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            double tolerance);

        /**
         * @copydoc rw::proximity::DistanceStrategy::distance
         */
        bool calcDistance(
            rw::proximity::DistanceResult &result,
			rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            double rel_err = 0.0,
            double abs_err = 0.0);

        /**
         * @copydoc rw::proximity::DistanceToleranceStrategy::getDistances
         */
        bool calcDistances(
            rw::proximity::MultiDistanceResult &result,
			rw::proximity::ProximityModel::Ptr a,
            const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr b,
            const rw::math::Transform3D<>& wTb,
            double tolerance,
            double rel_err = 0.0,
            double abs_err = 0.0);

        /**
		 * @copydoc rw::proximity::DistanceThresholdStrategy::getDistanceThreshold
		 */
        bool calcDistanceThreshold(
			rw::proximity::DistanceResult &rwresult,
			rw::proximity::ProximityModel::Ptr aModel,
			const rw::math::Transform3D<>& wTa,
			rw::proximity::ProximityModel::Ptr bModel,
			const rw::math::Transform3D<>& wTb,
			double threshold,
			double rel_err,
			double abs_err);

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
    private:
    	int _numBVTests,_numTriTests;

    	std::vector<RWPQPModel> _allmodels;
    	std::map<std::string, std::vector<int> > _geoIdToModelIdx;
    };

}} // end namespaces

#endif // end include guard
