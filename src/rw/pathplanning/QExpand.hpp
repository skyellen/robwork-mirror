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

#ifndef rw_pathplanning_QExpand_HPP
#define rw_pathplanning_QExpand_HPP

/**
   @file QExpand.hpp
*/

#include "QConstraint.hpp"
#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/

    class QExpand;

    //! A pointer to a QExpand.
    typedef rw::common::Ptr<QExpand> QExpandPtr;

    /**
       @brief Interface for sampling a configuration in the vicinity of some
       other configuration.

       QExpand is a primitive for planners in the SBL family. The primitive
       takes a configuration \b q as parameter and returns another configuration
       somewhere in the vicinity of \b q.

       Different implementations can have different policies with respect to
       what constraints are satisfied by the configurations returned.
    */
    class QExpand
    {
    public:
        /**
           @brief A configuration sampled from the vicinity of \b q.

           Implementation dependant, the sampler may return the empty
           configuration if no configurations can be sampled near \b q.
        */
        rw::math::Q expand(const rw::math::Q& q) { return doExpand(q); }

        /**
           @brief A configuration space in the shape of a box.

           The box is given by a lower and upper corner.
        */
        typedef std::pair<rw::math::Q, rw::math::Q> QBounds;

        /**
           @brief Expansion within the overlap of an inner and outer box.

           Given a configuration \b q, the expand() method returns a configuration
           sampled uniformly at random from the intersection between
\code
    q + inner
\endcode
           and
\code
    outer
\endcode

           Given a \b device, you typically use \b device.getBounds() as the box
           for the outer configuration space.

           If the overlap between the boxes is empty, expand() returns the empty
           configuration.
        */
        static std::auto_ptr<QExpand> makeUniformBox(
            const QBounds& outer,
            const QBounds& inner);

        /**
           @brief Expansion within a scaled down box of the configuration space.

           Given a configuration \b q, the expand() method samples a
           configuration uniformly at random from the intersection between
\code
    q + inner
\endcode
           and
\code
    outer
\endcode
           where \b inner equals \b outer scaled by a factor of \b ratio and
           centered at origo.

           This is a form of expansion you will use in a standard implementation
           of an SBL planner.

           \b ratio must be positive.

           If \b outer is non-empty, the expand() method will always return a
           non-empty configuration.
        */
        static std::auto_ptr<QExpand> makeUniformBox(
            const QBounds& outer,
            double ratio);

        /**
           @brief Sample within a box of decreasing size until a collision free
           configuration is found.

           The size of the inner box decreases as 1, 1/2, 1/3, ...

           This form of expansion is typical for SBL planners.

           The inner and outer box are specified as explained for
           makeUniformBox().
        */
        static std::auto_ptr<QExpand> makeShrinkingUniformBox(
            QConstraintPtr constraint,
            const QBounds& outer,
            const QBounds& inner);

        /**
           @brief Sample within a box of shrinking size until a collision free
           configuration is found.

           The inner box shrinks in size as 1, 1/2, 1/3, ...

           This form of expansion is typical for SBL planners.

           The inner and outer box are specified as explained for
           makeUniformBox().
        */
        static std::auto_ptr<QExpand> makeShrinkingUniformBox(
            QConstraintPtr constraint,
            const QBounds& outer,
            double ratio);

    protected:
        /**
           @brief Constructor
        */
        QExpand() {}

        /**
           @brief Subclass implementation of the expand() method.
        */
        virtual rw::math::Q doExpand(const rw::math::Q& q) = 0;

    private:
        // These are private as long as noone needs them or has provided a
        // QExpand implementation for which they make sense.

        /**
           @brief Let sample() expand near \b q.
        */
        void setSeed(const rw::math::Q& q);

        /**
           @brief Sample a configuration near the seed assigned by setSeed().
        */
        rw::math::Q sample();

        /**
           @brief Subclass implementation of the setSeed() method.

           The seed configuration has been assigned before doSetSeed() is called
           and can be retrieved with getSeet().

           By default the method does nothing.
        */
        virtual void doSetSeed(const rw::math::Q& q);

        /**
           @brief Subclass implementation of the sample() method.

           By default the method forwards to expand().
        */
        virtual rw::math::Q doSample();

        /**
           @brief The configuration assigned by \b getSeed() or the empty
           configuration if no configuration has been assigned.
        */
        const rw::math::Q& getSeed() const { return _seed; }

    private:
        QExpand(const QExpand&);
        QExpand& operator=(const QExpand&);

    private:
        rw::math::Q _seed;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
