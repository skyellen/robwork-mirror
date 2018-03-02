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


#ifndef RW_PROXIMITY_DISTANCECALCULATOR_HPP
#define RW_PROXIMITY_DISTANCECALCULATOR_HPP

#include "DistanceStrategy.hpp"
#include <rw/common/Timer.hpp>
#include <rw/proximity/CollisionSetup.hpp>

/**
 * @file DistanceCalculator.hpp
 */

namespace rw { namespace kinematics { class State; } }
namespace rw { namespace models { class WorkCell; } }

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/

    /**
     * @brief The DistanceCalculator implements an efficient way of calculating
     * different distances between two objects, each represented by a frame
     *
     * A list of frame pairs is contained within the distance calculater,
     * that specifies which frames are to be checked against each other.
     * The method of used for distance calculation relies on the DistanceStrategy
     * chosen.
     *
     * The DistanceCalculator supports switching between multiple strategies
     */
    class DistanceCalculator {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<DistanceCalculator> Ptr;
        //! @brief smart pointer type to this const class
		typedef rw::common::Ptr< const DistanceCalculator > CPtr;
        /**
         * @brief Distance calculations for a given tree, collision setup and
         * primitive distance calculator. Uses proximity strategy given by the workcell.
         *
         * \b strategy must be non-NULL.
         *
         * \b root must be non-NULL.
         *
         * Ownership of \b root is not taken.
         *
         * @param root [in] - the root of the Frame tree.
         *
         * @param workcell [in] - the workcell to do the distance calculations in.
         *
         * @param strategy [in] - the primitive strategy of distance calculations.
         *
         * @param initial_state [in] - the work cell state to use for the
         * initial traversal of the tree.
         */
        DistanceCalculator(rw::kinematics::Frame *root,
                           rw::common::Ptr<rw::models::WorkCell> workcell,
						   DistanceStrategy::Ptr strategy,
        				   const rw::kinematics::State& initial_state);

        /**
         * @brief Construct distance calculator for a WorkCell with an associated
         * distance calculator strategy.
         *
         * The DistanceCalculator extracts information about the tree and the
         * CollisionSetup from workcell.
         *
         * @param workcell [in] the workcell to check
         * @param strategy [in] the distance calculation strategy to use
         */
		DistanceCalculator(rw::common::Ptr<rw::models::WorkCell> workcell,
			DistanceStrategy::Ptr strategy);


        /**
         * @brief Constructs distance calculator for a selected set of frames
         *
         * The list \b pairs specifies which frame-pairs to be used for distance checking.
         *
         * \b strategy must be non-NULL.
         *
         * Ownership of \b root is not taken.
         *
         * @param pairs [in] Pairs of frame to check
         * @param strategy [in] the distance calculation strategy to use
         */
        DistanceCalculator(kinematics::FramePairList pairs,
			DistanceStrategy::Ptr strategy);

		/**
		 * @brief Destructor
		 */
        virtual ~DistanceCalculator();


        /**
         * @brief Calculates the distances between frames in the tree
         *
         * @param state [in] The state for which to calculate distances.
         *
         * @param result [out] If non-NULL, the distance results are written
         * to \b result.
         *
         * @return the shortest distance between frame and frame tree
         */
        DistanceStrategy::Result distance(const kinematics::State& state,
                                std::vector<DistanceStrategy::Result>* result = 0) const;

        DistanceStrategy::Result distanceOMP(const kinematics::State& state,
					  			   std::vector<DistanceStrategy::Result>* result = 0) const;

        /**
         * @brief Calculates the distance between frame and the rest of the tree
         *
         * @param state [in] The state for which to calculate distances.
         *
         * @param frame [in] The frame for which distances are to be calculated
         *
         * @param result [out] If non-NULL, the distance results are written
         * to \b result.
         *
         * @return the shortest distance between frame and frame tree
         */
        DistanceStrategy::Result distance(const kinematics::State& state,
                                const kinematics::Frame* frame,
                                std::vector<DistanceStrategy::Result>* result = 0) const;

        /**
         * @brief Set the primitive distance calculator to \b strategy.
         *
         * \b strategy must be non-NULL.
         *
         * Ownership of the strategy is not taken.
         *
         * @param strategy [in] - the primitive distance calculator to use.
         */
		void setDistanceStrategy(DistanceStrategy::Ptr strategy);

        /**
         * @brief Toggle whether the distance calculator should calculate the
         * distance along the nearest objects or all nearest points between every
         * other frame in the tree and the given frame in the distance calculation.
         *
         * By default the value of shortest distance is true.
         *
         * @param b [in] - if true the shortest distance will return after the
         * shortest distance has been found. This might be faster (depending on the
         * distance calculator)
         */
        void setShortestDistance(bool b) { _shortestDistance = b; }


        /**
         * @brief Adds distance model to frame
         *
         * The distance model is constructed based on the list of faces given.
         *
         * @param frame [in] frame to which the distance model should associate
         * @param faces [in] list of faces from which to construct the model
         * @return true if a distance model was succesfully created and linked
         * with the frame; false otherwise.
         */
        bool addDistanceModel(const rw::kinematics::Frame* frame, const rw::geometry::Geometry& faces);

        /**
         * @brief Clears the cache of the distance models
         */
        void clearCache();

		double getComputationTime() {
			return _timer.getTime();
		}

		int getCount() {
			return _cnt;
		}

		void resetComputationTimeAndCount() {
			_timer.resetAndPause();
			_cnt = 0;
		}


		void setDistanceThresholdStrategy(DistanceStrategy::Ptr strategy);
    private:
		mutable rw::common::Timer _timer;
		mutable int _cnt;

    	bool _shortestDistance;

    	rw::kinematics::Frame* _root;
    	rw::proximity::CollisionSetup _setup;
		DistanceStrategy::Ptr _strategy;
		DistanceStrategy::Ptr _thresholdStrategy;

        rw::kinematics::State _state;

        // The pairs of frames to check for distances.
        kinematics::FramePairList _distancePairs;


        DistanceCalculator(const DistanceCalculator&);
        DistanceCalculator& operator=(const DistanceCalculator&);

        /**
         * @brief Initializes the geometry of the workcell. The geometry is added to the used strategy
         * @param wc [in] Pointer to workcell to import geometry from.
         */
        void initializeGeometry(rw::common::Ptr<const rw::models::WorkCell> wc);
        /**
         * @brief Initializes the distance pairs to collision check based on
         * collision setup of the workcell.
         */
        void initializeDistancePairs();
    };

} } // End of namespace

#endif /*RW_PROXIMITY_DISTANCECALCULATOR_HPP*/
