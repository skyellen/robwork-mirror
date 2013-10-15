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


#ifndef RW_PROXIMITY_COLLISIONSTRATEGY_HPP
#define RW_PROXIMITY_COLLISIONSTRATEGY_HPP

/**
 * @file rw/proximity/CollisionStrategy.hpp
 */

#include <string>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/common/ExtensionPoint.hpp>
#include <rw/kinematics/FrameMap.hpp>



#include "ProximityStrategy.hpp"
#include "CollisionToleranceStrategy.hpp"
//#include "ProximityStrategyData.hpp"

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/
#ifdef RW_USE_DEPRECATED
    class CollisionStrategy;

    //! A pointer to a CollisionStrategy.
    typedef rw::common::Ptr<CollisionStrategy> CollisionStrategyPtr;
#endif
    /**
     * @brief The CDStrategy interface is used to abstract away
     * specific collision detection algorithms or strategies.
     */
    class CollisionStrategy : public virtual ProximityStrategy {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<CollisionStrategy> Ptr;

		//! the type of query that is to be performed
        typedef enum{FirstContact, AllContacts} QueryType;


        /**
         * @brief result of a single collision pair
         *
         * A collision result is one or all colliding triangles between two objects which may have
         * several geometries attached.
         * The collision result does not have access to the actual triangle meshes of the geometries
         * so to extract the actual contact location the user has to supply the triangles meshes of
         * the geometries himself.
         *
         */
        struct Result
        {
            //! @brief reference to the first model
            ProximityModel::Ptr a;

            //! @brief reference to the second model
            ProximityModel::Ptr b;

            //! @brief a collision pair of
            struct CollisionPair {
                //! @brief geometry index
                int geoIdxA, geoIdxB;
                /**
                 *  @brief indices into the geomPrimIds array, which means that inidicies [_geomPrimIds[startIdx];_geomPrimIds[startIdx+size]]
                 *  are the colliding primitives between geometries geoIdxA and geoIdxB
                 */
                int startIdx, size;
            };

            //! @breif transformation from a to b
            rw::math::Transform3D<> _aTb;

            //! @brief the collision pairs
            std::vector<CollisionPair> _collisionPairs;

            /**
             * @brief indices of triangles/primitives in geometry a and b that are colliding
             * all colliding triangle indices are in this array also those that are from different geometries
             */
            std::vector<std::pair<int, int> > _geomPrimIds;

            int _nrBVTests, _nrPrimTests;

            int getNrPrimTests(){ return _nrPrimTests; }
            int getNrBVTests(){ return _nrBVTests; }

            /**
             * @brief clear all result values
             */
            void clear(){
                _collisionPairs.clear();
                _geomPrimIds.clear();
                _nrBVTests = 0;
                _nrPrimTests = 0;
            }
        };



        /**
         * @brief Destroys object
         */
        virtual ~CollisionStrategy();

        /**
         * @brief Checks to see if two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are in collision
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param type [in] collision query type
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        virtual bool inCollision(
            const kinematics::Frame* a,
            const math::Transform3D<>& wTa,
            const kinematics::Frame *b,
            const math::Transform3D<>& wTb,
            QueryType type = FirstContact);

        /**
         * @brief Checks to see if two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are in collision
         * @param a [in] @f$ \mathcal{F}_a @f$
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         * @param b [in] @f$ \mathcal{F}_b @f$
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * @param data [in/out] caching and result container
         * @param type [in] collision query type
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        virtual bool inCollision(
            const kinematics::Frame* a,
            const math::Transform3D<>& wTa,
            const kinematics::Frame *b,
            const math::Transform3D<>& wTb,
            class ProximityStrategyData& data,
            QueryType type = FirstContact);

        /**
         * @brief Checks to see if two proximity models are in collision
         * @param a [in] model 1
         * @param wTa [in] transform of model a
         * @param b [in] model 2
         * @param wTb [in] transform of model b
         * @param data [in/out] caching and result container
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        virtual bool inCollision(
			ProximityModel::Ptr a,
            const math::Transform3D<>& wTa,
			ProximityModel::Ptr b,
            const math::Transform3D<>& wTb,
            ProximityStrategyData& data) = 0;

        /**
         * @brief describes a simple collision contact data structure
         */
		struct Contact {

			// point described in object A frame
        	rw::math::Vector3D<> point;

			// surface normal on object B described in object A coordinates
			rw::math::Vector3D<> normalA;
			rw::math::Vector3D<> normalB;
		};

		/**
		 * @brief this method interprets the collision query result and calculates a list of
		 * contacts to represent the collision geometry between the colliding geometries.
		 *
		 * Please note that for most collisions
		 * a single point and normal is not sufficient to describe the complete collision area. However,
		 * it is typically a reasonable approximation.
		 * The approximation can hence be implementation specific.
		 * @param contacts [out] list of contacts that can be calculated from data
		 * @param data [in] the result from the collision query
		 */
		virtual void getCollisionContacts(std::vector<CollisionStrategy::Contact>& contacts,
											  ProximityStrategyData& data) = 0;

        /**
           @brief A collision strategy constructed from a collision tolerance
           strategy and a resolution.

           The constructed collision strategy considers a pair of geometries to
           be in collision if \b strategy claim they are in collision for a
           tolerance of \b tolerance.
        */
		static CollisionStrategy::Ptr make(CollisionToleranceStrategy::Ptr strategy,
                         double tolerance);

        /**
           @brief A collision strategy constructed from a collision tolerance
           strategy and a resolution.

           The constructed collision strategy considers a pair of geometries to
           be in collision if \b strategy claim they are in collision for a
           tolerance of \b tolerance.
        */
        static CollisionStrategy::Ptr make(CollisionToleranceStrategy::Ptr strategy,
                         const rw::kinematics::FrameMap<double>& frameToTolerance,
                         double defaultTolerance);




    	/**
    	 * @addtogroup extensionpoints
    	 * @extensionpoint{rw::loaders::WorkCellSaver::Factory, rw::loaders::WorkCellSaver, rw.loaders.WorkCellSaver}
 	 	 */

		/**
		 * @brief a factory for WorkCellSaver. This factory also defines an
		 * extension point for workcell savers.
		 */

        class Factory: public rw::common::ExtensionPoint<CollisionStrategy> {
	    public:
	    	//! constructor
	        Factory():rw::common::ExtensionPoint<CollisionStrategy>("rw.proximity.CollisionStrategy",
	        														 "Extensions to create collision strategies"){};

	        /**
	         * @brief create
	         * @return
	         */
	        //static rw::common::Ptr<CollisionStrategy> getCollisionStrategy(const std::string& strategy_hint);

	    };



    private:
        CollisionStrategy(const CollisionStrategy&);
        CollisionStrategy& operator=(const CollisionStrategy&);

    protected:
        /**
         * @brief Creates object
         */
        CollisionStrategy();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
