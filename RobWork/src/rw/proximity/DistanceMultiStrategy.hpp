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


#ifndef RW_PROXIMITY_distancesSTRATEGY_HPP
#define RW_PROXIMITY_distancesSTRATEGY_HPP
/**
 * @file DistanceStrategy.hpp
 */

#include <rw/math/Transform3D.hpp>

#include "ProximityStrategy.hpp"

namespace rw { namespace kinematics { class State; } }

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/

    /**
     * @brief This interface allows the definition of computing all points between two
     * geometric objects that are closer than a specified tolerance. See ProxmityStrategy on
     * how to add geometry to the strategy.
     */
    class DistanceMultiStrategy: public virtual ProximityStrategy {

    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<DistanceMultiStrategy> Ptr;


        /**
         * @brief DistanceResult contains basic information about the distance
         * result between two frames.
         */
        struct Result {
             //! @brief reference to the first proximity model
            ProximityModel::Ptr a;

            //! @brief reference to the second proximity model
            ProximityModel::Ptr b;

            //! Closest point on f1 to f2, described in f1 reference frame
            math::Vector3D<double> p1;

            //! Closest point on f2 to f1, described in f2 reference frame
            math::Vector3D<double> p2;

            //! @brief distance between frame f1 and frame f2
            double distance;

            //! Closest points on f1 to f2, described in f1 reference frame
            std::vector< math::Vector3D<> > p1s;

            /**
             * @brief Closest point on f2 to f1, IMPORTANT! NOTICE! described in
             * >>>> \b f1 <<<<< reference frame
             */
            std::vector< math::Vector3D<> > p2s;
            
            /**
             * @brief indices to the primitives which are the closest points on the first proximity model
             **/
            std::vector< int > p1prims;

            /**
             * @brief indices to the primitives which are the closest points on the second proximity model
             **/
            std::vector< int > p2prims;

            //! distances between contact points
            std::vector< double > distances;

            void clear(){
                p1s.clear();
                p2s.clear();
                p1prims.clear();
                p2prims.clear();
                distances.clear();
            }
        };

        /**
         * @brief Destroys object
         */
        virtual ~DistanceMultiStrategy();

        /**
         * @brief Calculates all distances between geometry of two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$
         * @param result [out] MultiDistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param tolerance [in] point pairs that are closer than tolerance will
         * be included in the result.
         * @param rel_err [in] relative acceptable error
         * @param abs_err [in] absolute acceptable error
         *
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        Result distances(
            const kinematics::Frame* a,
            const math::Transform3D<>& wTa,
            const kinematics::Frame* b,
            const math::Transform3D<>& wTb,
            double tolerance);

        /**
         * @brief Calculates all distances between geometry of two  given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$
         * @param result [out] MultiDistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param tolerance [in] point pairs that are closer than tolerance will
         * be included in the result.
         * @param rel_err [in] relative acceptable error
         * @param abs_err [in] absolute acceptable error
         *
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        Result& distances(
            const kinematics::Frame* a,
            const math::Transform3D<>& wTa,
            const kinematics::Frame* b,
            const math::Transform3D<>& wTb,
            double tolerance,
            class ProximityStrategyData& data);

        /**
         * @copydoc dodistances
         */
        Result& distances(
			ProximityModel::Ptr a,
            const math::Transform3D<>& wTa,
			ProximityModel::Ptr b,
            const math::Transform3D<>& wTb,
            double tolerance,
            class ProximityStrategyData& data)
        {
            return doDistances(a,wTa,b,wTb,tolerance,data);
        }
    protected:

        /**
         * @brief Calculates all distances between two given objects @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ where the distances are below a certain threshold.
         * @param result [out] MultiDistanceResult to copy result into
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param tolerance [in] point pairs that are closer than tolerance will
         * be included in the result.
         * @param rel_err [in] relative acceptable error
         * @param abs_err [in] absolute acceptable error
         *
         * @return list of distances between all points that are closer than threshold.
         */
        virtual Result& doDistances(
            ProximityModel::Ptr a,
            const math::Transform3D<>& wTa,
            ProximityModel::Ptr b,
            const math::Transform3D<>& wTb,
            double tolerance,
            class ProximityStrategyData& data) = 0;

    private:
        DistanceMultiStrategy(const DistanceMultiStrategy&);
        DistanceMultiStrategy& operator=(const DistanceMultiStrategy&);

    protected:
        /**
         * @brief Creates object
         */
        DistanceMultiStrategy();
    };

    /*@}*/
}} // end namespaces

#endif /* RW_PROXIMITY_distancesSTRATEGY_HPP */
