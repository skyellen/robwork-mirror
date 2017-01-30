/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYFCL_HPP
#define RWLIBS_PROXIMITYSTRATEGIES_PROXIMITYSTRATEGYFCL_HPP

/**
 * @file ProximityStrategyFCL.hpp
 */

#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/DistanceStrategy.hpp>

#include <string>
#include <vector>
#include <utility>

namespace fcl { class CollisionGeometry; }
namespace fcl { class CollisionRequest; }
namespace fcl { class CollisionResult; }
namespace fcl { class DistanceRequest; }
namespace fcl { class DistanceResult; }

namespace rwlibs { namespace proximitystrategies {
        /** @addtogroup proximitystrategies */
        /*@{*/

        /**
         * @brief This is a strategy wrapper for the Flexible Collision Library (FCL)
         *
         * For further information check out https://github.com/flexible-collision-library/fcl
         */
        class ProximityStrategyFCL :
            public rw::proximity::CollisionStrategy,
            public rw::proximity::DistanceStrategy
        {
        public:
        	//! @brief Smart pointer type for FCL Proximity strategy.
            typedef rw::common::Ptr<ProximityStrategyFCL> Ptr;

            //! @brief Type of internal collision geometry.
        	typedef rw::common::Ptr<fcl::CollisionGeometry> FCLBVHModelPtr;

			//! @brief Datatype to hold the FCL bounding volume and related geometrical data.
            struct FCLModel {
        		/**
        		 * @brief Create new holder for internal collision geometry information.
        		 * @param geoId [in] id of the geometry.
        		 * @param transform [in] transform of the geometry.
        		 * @param model [in] the internal model of the collision geometry.
        		 */
                FCLModel(const std::string& geoId, const rw::math::Transform3D<>& transform, FCLBVHModelPtr model): geoId(geoId), t3d(transform), model(model) { /* Empty */ }
                //! @brief Identifier for the geometry.
                std::string geoId;
                //! @brief Location of the geometry.
                rw::math::Transform3D<> t3d;
                //! @brief Using fcl::CollisionGeometry as the type of the model, to allow holding all the different fcl::BVHModel{bv type} types.
                FCLBVHModelPtr model;
            };

            //! @brief Type for list of proximity models.
            typedef std::vector<FCLModel> FCLModelList;

            //! @brief Datatype to hold the proximity models
            struct FCLProximityModel : public rw::proximity::ProximityModel {
            	/**
            	 * @brief Constructor.
            	 * @param owner [in] the strategy owning this model.
            	 */
                FCLProximityModel(ProximityStrategy *owner) : ProximityModel(owner) {
                    /* Nothing specific */
                }
                //! @brief Models holding the internal collision geometry.
                FCLModelList models;
            };

            //! @brief Supported bounding volumes
            enum class BV {
            	AABB,   //!< Axis-Aligned Bounding Boxes
				OBB,    //!< Oriented Bounding Boxes
				RSS,    //!< Rectangle Swept Spheres
				OBBRSS, //!< Mix of OBB and RSS
				kIOS,   //!< Bounding volume as the intersection of a set of spheres
				KDOP16, //!< Discrete Oriented Polytope
				KDOP18, //!< Discrete Oriented Polytope
				KDOP24  //!< Discrete Oriented Polytope
            };

            /**
             * @brief Constructor
             * @param bv [in] the bounding volume type to use.
             */
            ProximityStrategyFCL(BV bv = BV::AABB);

            /**
             * @brief Destructor
             */
            virtual ~ProximityStrategyFCL();

            //// interface of ProximityStrategy
            //! @copydoc rw::proximity::ProximityStrategy::createModel
            virtual rw::proximity::ProximityModel::Ptr createModel();

            //! @copydoc rw::proximity::ProximityStrategy::destroyModel
            void destroyModel(rw::proximity::ProximityModel* model);

            /**
             * @copydoc rw::proximity::ProximityStrategy::addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom)
             *
             * @throws Exception when a bounding volume type has been chosen that is not supported.
             */
            bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom);


            /**
             * @copydoc rw::proximity::ProximityStrategy::addGeometry(ProximityModel* model, rw::common::Ptr<rw::geometry::Geometry> geom, bool forceCopy=false)
             *
             * @throws Exception when a bounding volume type has been chosen that is not supported.
             */
            bool addGeometry(rw::proximity::ProximityModel* model, rw::common::Ptr<rw::geometry::Geometry> geom, bool forceCopy=false);

            //! @copydoc rw::proximity::ProximityStrategy::removeGeometry
            bool removeGeometry(rw::proximity::ProximityModel* model, const std::string& geomId);

            //! @copydoc rw::proximity::ProximityStrategy::getGeometryIDs
            std::vector<std::string> getGeometryIDs(rw::proximity::ProximityModel* model);

            //! @copydoc rw::proximity::ProximityStrategy::clear
            void clear();

            //// Interface of CollisionStrategy
            //! @copydoc rw::proximity::CollisionStrategy::inCollision
            bool doInCollision(
                rw::proximity::ProximityModel::Ptr a,
                const rw::math::Transform3D<>& wTa,
                rw::proximity::ProximityModel::Ptr b,
                const rw::math::Transform3D<>& wTb,
                rw::proximity::ProximityStrategyData& data);

            /**
             * @copydoc rw::proximity::CollisionStrategy::getCollisionContacts
             *
             * @note Not implemented as nothing appears to be using this functionality
             */
            void getCollisionContacts(
                std::vector<CollisionStrategy::Contact>& contacts,
                rw::proximity::ProximityStrategyData& data);

            //// Interface of DistanceStrategy
            //! @copydoc rw::proximity::DistanceStrategy::doDistance
            rw::proximity::DistanceStrategy::Result& doDistance(
                rw::proximity::ProximityModel::Ptr a,
                const rw::math::Transform3D<>& wTa,
                rw::proximity::ProximityModel::Ptr b,
                const rw::math::Transform3D<>& wTb,
                class rw::proximity::ProximityStrategyData& data);

            //// End of interfaces

            /**
             * @brief Set the bounding volume
             * @param bv [in] new bounding volume type.
             */
            void setBV(const BV& bv) {
                _bv = bv;
            }

            /**
             * @brief Get the bounding volume
             * @return the type of bounding volume used.
             */
            BV getBV() {
                return _bv;
            }

            /**
             * @brief Get access to the CollisionRequest that is used with FCL collision query
             *
             * See the fcl/collision_data.h for the data structure, and look through their documentation for specifics on what solvers that can be chosen.
             */
            fcl::CollisionRequest& getCollisionRequest();

            /**
             * @brief Get access to the DistanceRequest that is used with FCL distance query
             *
             * See the fcl/collision_data.h for the data structure, and look through their documentation for specifics on what solvers that can be chosen.
             *
             * @note The rel_err and abs_err fields will be overwritten with the values that are supplied in the rw::proximity::ProximityStrategyData
             */
            fcl::DistanceRequest& getDistanceRequest();

            /**
             * @brief Specify if FCL results should be collected
             */
            void setCollectFCLResults(bool enable) {
                _collectFCLResults = enable;
            }

            /**
             * @brief Get whether FCL results should be collected or not
             */
            bool getCollectFCLResults() {
                return _collectFCLResults;
            }

            /**
             * @brief Get access to the collected FCL collision results
             */
            std::vector<fcl::CollisionResult>& getCollisionResults() {
                return _fclCollisionResults;
            }

            /**
             * @brief Get access to the collected FCL collision result
             *
             * @throws std::out_of_range exception if index is not within the bounds.
             */
            fcl::CollisionResult& getCollisionResult(std::size_t index);

            /**
             * @brief Get access to the collected FCL distance result
             */
            fcl::DistanceResult& getDistanceResult();
            
        private:
            template<typename BV> bool addGeometry(rw::proximity::ProximityModel* model, const rw::geometry::Geometry& geom);

        private:
            BV _bv;
            fcl::CollisionRequest* const _fclCollisionRequest;
            fcl::DistanceRequest* const _fclDistanceRequest;

            bool _collectFCLResults;
            std::vector<fcl::CollisionResult> _fclCollisionResults;
            fcl::DistanceResult* _fclDistanceResult;
        };
    }} // end namespaces

#endif // include guard
