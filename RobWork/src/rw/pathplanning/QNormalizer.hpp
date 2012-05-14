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
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<QNormalizer> Ptr;

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
