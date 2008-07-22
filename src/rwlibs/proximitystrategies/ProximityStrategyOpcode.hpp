/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
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

#ifndef rwlibs_collisionstrategies_CDStrategyOpcode_HPP
#define rwlibs_collisionstrategies_CDStrategyOpcode_HPP

/**
 * @file ProximityStrategyOpcode.hpp
 */

#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/math/Transform3D.hpp>

#include <map>

namespace Opcode{
    class AABBTreeCollider;
    class BVTCache;
    class Model;
};

namespace rwlibs { namespace proximitystrategies {

    /** @addtogroup proximitystrategies */
    /*@{*/

    /**
     * @brief Wrapper for the OpCode collision detection library.
     *
     * OpCode uses Axis Aligned Bounding Boxes (AABB) and hierachical bounding trees
     * to perform fast collision checking. For further information check out
     * http://www.codercorner.com/Opcode.htm
     */
    class ProximityStrategyOpcode : public rw::proximity::CollisionStrategy {
    private:
        typedef std::map<const rw::kinematics::Frame*, Opcode::Model*> FrameModelMap;
        FrameModelMap _frameModelMap;
        Opcode::AABBTreeCollider* _AABBTC;

    public:
        /**
         * @brief Creates object
         */
        ProximityStrategyOpcode();

        /**
         * @brief Destroys object
         */
        virtual ~ProximityStrategyOpcode();

        /**
         * @copydoc rw::proximity::ProximityStrategy::addModel(const rw::kinematics::Frame*)
         */
        bool addModel(const rw::kinematics::Frame *frame);

        /**
         * @copydoc rw::proximity::ProximityStrategy::addModel(const rw::kinematics::Frame*, const std::vector<rw::geometry::Face<float> >&)
         */
        bool addModel(const rw::kinematics::Frame* frame,
                      const std::vector<rw::geometry::Face<float> >& faces);

        /**
         * @copydoc rw::proximity::ProximityStrategy::hasModel
         */
        bool hasModel(const rw::kinematics::Frame* frame);
        
        /**
         * @copydoc rw::proximity::CollisionStrategy::setFirstContact
         */
        void setFirstContact(bool b);

        /**
         * @copydoc rw::proximity::CollisionStrategy::inCollision
         */
        bool inCollision(
            const rw::kinematics::Frame* a,
            const rw::math::Transform3D<>& wTa,
            const rw::kinematics::Frame *b,
            const rw::math::Transform3D<>& wTb);

        /**
         * @copydoc rw::proximity::ProximityStrategy::clear
         */
        void clear();

        /**
           @copydoc rw::proximity::ProximityStrategy::clearFrame
         */
        void clearFrame(const rw::kinematics::Frame* frame);

        /**
           @brief An Opcode based collision strategy.
        */
        static std::auto_ptr<rw::proximity::CollisionStrategy> make();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
