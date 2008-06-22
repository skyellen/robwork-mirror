/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rw_collision_CollisionDetector_HPP
#define rw_collision_CollisionDetector_HPP

/**
 * @file CollisionDetector.hpp
 *
 * @brief Class rw::proximity::CollisionDetector
 */

#include "ProximityCommon.hpp"
#include "CollisionSetup.hpp"

#include <rw/math/Transform3D.hpp>
#include <rw/geometry/Face.hpp>
#include <rw/kinematics/State.hpp>

#include <vector>
#include <boost/shared_ptr.hpp>

namespace rw { namespace models { class WorkCell; }}
namespace rw { namespace kinematics { class Frame; }}

namespace rw { namespace proximity {

    class CollisionStrategy;

    /** @addtogroup proximity */
    /*@{*/

    /**
     * @brief The CollisionDetector implements an efficient way of checking a
     * complete frame tree for collisions.
     *
     * It contain a set of pairs of frames that are not to be checked against
     * each other. The collision detector does not dictate a specific detection
     * strategy or algorithm, instead it relies on the CollisionStrategy interface for
     * the actual collision checking between two frames.
     *
     * The CollisionDetector supports switching between multiple strategies.
     */
    class CollisionDetector {
    public:
        /**
         * @brief Collision detection for a given tree, collision setup and
         * primitive collision checker.
         *
         * \b strategy must be non-NULL.
         *
         * The CollisionDetector takes the ownership of \b strategy.
         *
         * \b root must be non-NULL.
         *
         * Ownership of \b root is not taken.
         *
         * @param root [in] - the root of the Frame tree.
         *
         * @param setup [in] - the setup of the collision checking.
         *
         * @param strategy [in] - the primitive collision checker.
         *
         * @param initial_state [in] - the work cell state to use for the
         * initial traversal of the tree.
         */
        CollisionDetector(
            kinematics::Frame *root,
            const CollisionSetup& setup,
            CollisionStrategy* strategy,
            const kinematics::State& initial_state);

        /**
         * @brief Construct collision detector for a WorkCell with an associated
         * collision checker strategy.
         *
         * The CollisionDetector extracts information about the tree and the
         * CollisionSetup from workcell.
         *
         * The CollisionDetector does not take ownership of the workcell
         *
         * The CollisionDetector takes ownership of the CollisionStrategy
         *
         * @param workcell [in] the workcell to check
         * @param strategy [in] the collision checker strategy to use
         */
        CollisionDetector(
            models::WorkCell* workcell,
            CollisionStrategy* strategy);

        /**
         * @brief checks the frame tree for collision
         *
         * @param state [in] The state for which to check for collisions.
         *
         * @param result [out] If non-NULL, the pairs of colliding frames are
         * written to \b result.
         *
         * @return true if a collision is detected; false otherwise.
         */
        bool inCollision(
            const kinematics::State& state,
            FramePairList* result = 0) const;

#ifndef RW_REMOVE_DEPRECATED
        /**
           @brief DEPRECATED. Use setCollisionStrategy().
        */
        void setCDStrategy(CollisionStrategy* strategy)
        { setCollisionStrategy(strategy); }
#endif /* RW_REMOVE_DEPRECATED */

        /**
         * @brief Set the primitive collision strategy to \b strategy.
         *
         * \b strategy must be non-NULL.
         *
         * The CollisionDetector takes the ownership of \b strategy.
         *
         * @param strategy [in] - the primitive collision checker to use.
         */
        void setCollisionStrategy(CollisionStrategy* strategy);

        /**
           @brief The collision strategy of the collision checker.
        */
        CollisionStrategy& getCollisionStrategy() const { return *_strategy; }

        /**
         * @brief Toggle wether the collision detector should stop checking
         * after first found collision.
         *
         * By default the value of first contact is true.
         *
         * @param b [in] - if true the collision detector will return after the
         * first found collision. If false all colliding pairs of frames will be
         * found.
         */
        void setFirstContact(bool b) { _firstContact = b; }

        /**
         * @brief Adds collision model to frame
         *
         * The collision model is constructed based on the list of faces given.
         *
         * @param frame [in] frame to which the collision model should associate
         * @param faces [in] list of faces from which to construct the model
         * @return true if a collision model was succesfully created and linked
         * with the frame; false otherwise.
         */
        bool addCollisionModel(
            const rw::kinematics::Frame* frame,
            const std::vector<rw::geometry::Face<float> >& faces);

        /**
         * @brief Clears the cache of the collision models
         */
        void clearCache();

    private:
        bool _firstContact;
        rw::kinematics::Frame* _root;
        rw::proximity::CollisionSetup _setup;
        boost::shared_ptr<CollisionStrategy> _strategy;
        rw::kinematics::State _state;

        // The pairs of frames to check for collisions.
        std::set<FramePair> _collisionPairs;

    private:
        CollisionDetector(const CollisionDetector&);
        CollisionDetector& operator=(const CollisionDetector&);

        void initialize();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
