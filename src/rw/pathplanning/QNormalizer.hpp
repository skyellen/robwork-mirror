/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_PATHPLANNING_QNORMALIZER_HPP
#define RW_PATHPLANNING_QNORMALIZER_HPP

/**
   @file QNormalizer.hpp
*/

#include <rw/math/Q.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/

    /**
       @brief Normalization of configurations.

       QNormalizer linearly maps configurations of a rectangular configuration
       space into a square configuration space with lower corner (0, 0, ..., 0)
       and upper corner (1, 1, ..., 1).
    */
    class QNormalizer
    {
    public:
        /**
           @brief Convert from a normalized configuration to a real
           configuration.
        */
        rw::math::Q fromNormalized(const rw::math::Q& q) const;

        /**
           @brief Convert a real configuration to a normalized configuration.
        */
        rw::math::Q toNormalized(const rw::math::Q& q) const;

        /**
           @brief Convert from a normalized configuration to a real
           configuration and assign the real configuration to \b q.
        */
        void setFromNormalized(rw::math::Q& q) const;

        /**
           @brief Convert a real configuration to a normalized configuration and
           write the normalized configuration to \b q.
        */
        void setToNormalized(rw::math::Q& q) const;

        /**
           @brief The box of the configuration space with respect to which
           normalization is done.
        */
        const std::pair<rw::math::Q, rw::math::Q>& getBounds() const
        { return _bounds; }

        /**
           @brief Normalizer for the configuration space box given by \b bounds.
        */
        explicit QNormalizer(const std::pair<rw::math::Q, rw::math::Q>& bounds)
            : _bounds(bounds)
        {}

        /**
           @brief Normalizer for the already normalized configuration space box.
        */
        static QNormalizer identity() { return QNormalizer(); }

    private:
        QNormalizer() {}

    private:
        std::pair<rw::math::Q, rw::math::Q> _bounds;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
