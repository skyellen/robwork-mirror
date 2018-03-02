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


#ifndef RW_PROXIMITY_DISTANCESTRATEGY_HPP
#define RW_PROXIMITY_DISTANCESTRATEGY_HPP
/**
 * @file DistanceStrategy.hpp
 */

#include <rw/common/ExtensionPoint.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/math/Transform3D.hpp>

#include "ProximityStrategy.hpp"

namespace rw { namespace kinematics { class Frame; } }

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/

    /**
     * @brief This is an interface that defines methods for computing the minimum distance
     * between geometric objects. If geometry objects has been related to frames (see ProximityStrategy)
     * then distance functions computing the distance between the geometry attached to frames can also be used.
     */
    class DistanceStrategy : public virtual ProximityStrategy {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<DistanceStrategy> Ptr;

        /**
         * @brief DistanceResult contains basic information about the distance
         * result between two sets of geometries. These geometry sets
         */
        struct Result {
			Result(): f1(NULL), f2(NULL), distance(0), geoIdxA(-1), geoIdxB(-1), idx1(0), idx2(0) {}

             //! @brief reference to the first frame
            const kinematics::Frame* f1;

            //! @brief reference to the second frame
            const kinematics::Frame* f2;
            
            /**
             * @brief pointer to the ProximityModel containing the geometries for the first frame
             **/
            ProximityModel::Ptr a;
            
            /**
             * @brief pointer to the ProximityModel containing the geometries for the second frame
             **/
            ProximityModel::Ptr b;

            //! Closest point on f1 to f2, described in f1 reference frame
            math::Vector3D<double> p1;

            //! Closest point on f2 to f1, described in >>>> \b f1 <<<<< reference frame
            math::Vector3D<double> p2;

            //! @brief distance between frame f1 and frame f1
            double distance;

            //! @brief geometry index to triangle mesh A
            int geoIdxA;
            
            //! @brief geometry index to triangle mesh B
            int geoIdxB;

            //! @brief index to the first face/triangle that is the closest feature
            unsigned int idx1;
            
            //! @brief index to the second face/triangle that is the closest feature
            unsigned int idx2;

            void clear(){ }
            
           /**
           @brief Streaming operator.
            */
            friend std::ostream& operator<<(std::ostream& out, const DistanceStrategy::Result& o)
            {
                return out
                    << "DistanceStrategy::Result("
                    << "o.f1: " << o.f1
                    << ", o.f2: " << o.f2 
                    << ", o.a: " << o.a 
                    << ", o.b: " << o.b
                    << ", o.p1: " << o.p1
                    << ", o.p2: " << o.p2
                    << ", o.distance: " << o.distance 
                    << ", o.geoIdxA: " << o.geoIdxA
                    << ", o.geoIdxB: " << o.geoIdxB 
                    << ", o.idx1: " << o.idx1 
                    << ", o.idx2: " << o.idx2 
                    << ")";
            }
        };

        /**
         * @brief Destroys object
         */
        virtual ~DistanceStrategy();

        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$
         * @param result [out] DistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        Result distance(const kinematics::Frame* a,
        					  const math::Transform3D<>& wTa,
        		              const kinematics::Frame* b,
        		              const math::Transform3D<>& wTb);

        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$
         * @param result [out] DistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        Result& distance(
                              const kinematics::Frame* a,
                              const math::Transform3D<>& wTa,
                              const kinematics::Frame* b,
                              const math::Transform3D<>& wTb,
                              class ProximityStrategyData& data);

        /**
         * @brief Calculates the distance between two proximity models @f$ \mathcal{a} @f$ and
         * @f$ \mathcal{b} @f$
         * @param result [out] DistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        Result& distance(
			ProximityModel::Ptr a,
            const math::Transform3D<>& wTa,
			ProximityModel::Ptr b,
			const math::Transform3D<>& wTb,
			class ProximityStrategyData& data){ return doDistance(a,wTa,b,wTb,data); }


        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ if the distance are within threshold. If the distance
         * between the frames are larger than the threshold, the result will be inaccurate.
         * @param result [out] DistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param threshold [in] threshold for distance calculations
         * @param rel_err [in] relative acceptable error
         * @param abs_err [in] absolute acceptable error
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        Result distance(const kinematics::Frame* a,
                              const math::Transform3D<>& wTa,
                              const kinematics::Frame* b,
                              const math::Transform3D<>& wTb,
                              double threshold);

        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ if the distance are within threshold. If the distance
         * between the frames are larger than the threshold, the result will be inaccurate.
         * @param result [out] DistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param threshold [in] threshold for distance calculations
         * @param rel_err [in] relative acceptable error
         * @param abs_err [in] absolute acceptable error
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        DistanceStrategy::Result& distance(const kinematics::Frame* a,
                              const math::Transform3D<>& wTa,
                              const kinematics::Frame* b,
                              const math::Transform3D<>& wTb,
                              double threshold,
                              ProximityStrategyData& data);

        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ if the distance are within threshold. If the distance
         * between the frames are larger than the threshold, the result will be inaccurate.
         * @param result [out] DistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param threshold [in] threshold for distance calculations
         * @param rel_err [in] relative acceptable error
         * @param abs_err [in] absolute acceptable error
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        DistanceStrategy::Result& distance(
                ProximityModel::Ptr a,
                const math::Transform3D<>& wTa,
                ProximityModel::Ptr b,
                const math::Transform3D<>& wTb,
                double threshold,
                ProximityStrategyData& data){
            return doDistanceThreshold(a, wTa, b, wTb, threshold, data);
        }

    	/**
    	 * @addtogroup extensionpoints
    	 * @extensionpoint{rw::proximity::DistanceStrategy::Factory,rw::proximity::DistanceStrategy,rw.proximity.DistanceStrategy}
    	 */

    	/**
    	 * @brief A factory for a DistanceStrategy. This factory also defines an ExtensionPoint.
    	 *
    	 * Extensions providing a DistanceStrategy implementation can extend this factory by registering
    	 * the extension using the id "rw.proximity.DistanceStrategy".
    	 *
    	 * Typically one or more of the following DistanceStrategy types will be available:
    	 *  - RW - rw::proximity::ProximityStrategyRW - Internal RobWork proximity strategy
    	 *  - Bullet - rwlibs::proximitystrategies::ProximityStrategyBullet - Bullet Physics
    	 *  - PQP - rwlibs::proximitystrategies::ProximityStrategyPQP - Proximity Query Package
    	 *  - FCL - rwlibs::proximitystrategies::ProximityStrategyFCL - Flexible Collision Library
    	 */
    	class Factory: public rw::common::ExtensionPoint<DistanceStrategy> {
    	public:
    		/**
    		 * @brief Get the available strategies.
    		 * @return a vector of identifiers for strategies.
    		 */
    		static std::vector<std::string> getStrategies();

    		/**
    		 * @brief Check if strategy is available.
    		 * @param strategy [in] the name of the strategy.
    		 * @return true if available, false otherwise.
    		 */
    		static bool hasStrategy(const std::string& strategy);

    		/**
    		 * @brief Create a new strategy.
    		 * @param strategy [in] the name of the strategy.
    		 * @return a pointer to a new DistanceStrategy.
    		 */
    		static DistanceStrategy::Ptr makeStrategy(const std::string& strategy);

    	private:
    		Factory();
    	};

    protected:

        /**
         * @brief Calculates the distance between two proximity models @f$ \mathcal{a} @f$ and
         * @f$ \mathcal{b} @f$
         * @param result [out] DistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        virtual Result& doDistance(
            ProximityModel::Ptr a,
            const math::Transform3D<>& wTa,
            ProximityModel::Ptr b,
            const math::Transform3D<>& wTb,
            class ProximityStrategyData& data) = 0;


        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ if the distance are within threshold. If the distance
         * between the frames are larger than the threshold, the result will be inaccurate.
         * @param result [out] DistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param threshold [in] threshold for distance calculations
         * @param rel_err [in] relative acceptable error
         * @param abs_err [in] absolute acceptable error
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        virtual DistanceStrategy::Result& doDistanceThreshold(
                ProximityModel::Ptr a,
                const math::Transform3D<>& wTa,
                ProximityModel::Ptr b,
                const math::Transform3D<>& wTb,
                double threshold,
                ProximityStrategyData& data)
        {
            return doDistance(a,wTa,b,wTb,data);
        }

    private:
        DistanceStrategy(const DistanceStrategy&);
        DistanceStrategy& operator=(const DistanceStrategy&);

    protected:
        /**
         * @brief Creates object
         */
        DistanceStrategy();
    };

    /*@}*/
}} // end namespaces

#endif /* RW_PROXIMITY_DISTANCESTRATEGY_HPP */
