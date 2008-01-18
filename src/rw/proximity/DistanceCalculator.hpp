#ifndef rw_distance_DistanceCalculator_HPP
#define rw_distance_DistanceCalculator_HPP

#include "DistanceStrategy.hpp"
#include <rw/proximity/CollisionSetup.hpp>
#include <rw/proximity/CollisionDetector.hpp>

#include <rw/kinematics/State.hpp>
/**
 * @file DistanceCalculator.hpp
 */

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
        /**
         * @brief Distance calculations for a given tree, collision setup and
         * primitive distance calculator.
         *
         * \b strategy must be non-NULL.
         *
         * The Distance Calculator takes the ownership of \b strategy.
         *
         * \b root must be non-NULL.
         *
         * Ownership of \b root is not taken.
         *
         * @param root [in] - the root of the Frame tree.
         *
         * @param setup [in] - the setup of the distance calculations (CollisionSetup).
         *
         * @param strategy [in] - the primitive strategy of distance calculations.
         *
         * @param initial_state [in] - the work cell state to use for the
         * initial traversal of the tree.
         */
        DistanceCalculator(rw::kinematics::Frame *root,
        				   const CollisionSetup& setup,
        				   DistanceStrategy* strategy,
        				   const rw::kinematics::State& initial_state);
        
        /**
         * @brief Construct distance calculator for a WorkCell with an associated
         * distance calculator strategy.
         *
         * The DistanceCalculator extracts information about the tree and the
         * CollisionSetup from workcell.
         *
         * The DistanceCalculator does not take ownership of the workcell
         *
         * The DistanceCalculator takes ownership of the DistanceStrategy
         *
         * @param workcell [in] the workcell to check
         * @param strategy [in] the collision checker strategy to use
         */
        DistanceCalculator(rw::models::WorkCell* workcell,
        				   DistanceStrategy* strategy);
        
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
        DistanceResult distance(const kinematics::State& state, 
                                std::vector<DistanceResult>* result = 0) const;
        
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
        DistanceResult distance(const kinematics::State& state, 
                                const kinematics::Frame* frame, 
                                std::vector<DistanceResult>* result = 0) const;

        /**
         * @brief Set the primitive distance calculator to \b strategy.
         *
         * \b strategy must be non-NULL.
         *
         * Ownership of the strategy is not taken.
         *
         * @param strategy [in] - the primitive distance calculator to use.
         */
        void setDistanceStrategy(DistanceStrategy* strategy);

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
        bool addDistanceModel(const rw::kinematics::Frame* frame, const std::vector<rw::geometry::Face<float> >& faces);

        /**
         * @brief Clears the cache of the distance models      
         */
        void clearCache();
        
    private:
    	bool _shortestDistance;
    	
    	rw::kinematics::Frame* _root;
    	rw::proximity::CollisionSetup _setup;
        boost::shared_ptr<DistanceStrategy> _strategy;
        rw::kinematics::State _state;

        // The pairs of frames to check for distances.
        FramePairList _distancePairs;
        
        
        DistanceCalculator(const DistanceCalculator&);
        DistanceCalculator& operator=(const DistanceCalculator&);

        void initialize();
    };

    
} } // End of namespace

#endif /*rw_distance_DistanceCalculator_HPP*/
