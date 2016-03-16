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


#ifndef RW_MATH_RPY_HPP
#define RW_MATH_RPY_HPP

/**
 * @file RPY.hpp
 */

#include <rw/common/Serializable.hpp>

#include "Rotation3DVector.hpp"
#include "Rotation3D.hpp"
#include "Vector3D.hpp"

namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
     * @brief A class for representing Roll-Pitch-Yaw Euler angle rotations.
     */
    template<class T = double>
    class RPY: public Rotation3DVector<T> {
    public:

        /**
         * @brief Constructs rotation in which all elements are initialized to 0
         */
        RPY() : _rpy(0.0, 0.0, 0.0) {}

        /**
         * @brief Constructs an initialized roll-pitch-yaw euler angle vector
         * @param roll Rotation around z
         * @param pitch Rotation around y
         * @param yaw Rotation around x
         */
        RPY(T roll, T pitch, T yaw) : _rpy(roll, pitch, yaw) {}

        /**
         * @brief Constructs an RPY object initialized according to the specified Rotation3D
         *
         * \f$ \beta = arctan2(-r_{31},\sqrt{r_{11}^2+r_{21}^2}) \f$
         * \f$ \alpha = arctan2(r_{21}/cos(\beta), r_{11}/cos(\beta)) \f$
         * \f$ \beta = arctan2(r_{32}/cos(\beta), r_{33}/cos(\beta))) \f$
         *
         * @param R [in] A 3x3 rotation matrix \f$ \mathbf{R} \f$
         *
         * @param epsilon [in] Value specifying the value for which \f$
         * cos(\beta)\f$ is assumed 0 and the special case solution assuming
         * \f$\alpha=0, \beta=\pi/2 and \gamma = arctan2(r_{21}, r_{22})\f$ is
         * to be used.
         */
        explicit RPY(const Rotation3D<T>& R, T epsilon = 1e-5);

        /**
         * @copydoc Rotation3DVector::toRotation3D
         */
        const Rotation3D<T> toRotation3D() const;

        /**
         * @brief Returns reference to the element
         *
         * @param index [in] index of element
         *
         * @return reference to the element
         */
        T& operator()(size_t index){
            return _rpy(index);
        }

        /**
         * @brief Returns a const reference the an element
         *
         * @param index [in] index of element
         *
         * @return const reference to the element
         */
        const T& operator()(size_t index) const {
            return _rpy(index);
        }


        /**
         * @brief Returns a const reference the an element
         *
         * @param index [in] index of element
         *
         * @return const reference to the element
         */
        const T& operator[](size_t i) const { return (*this)(i); }

        /**
          * @brief Returns reference to the element
          *
          * @param index [in] index of element
          *
          * @return reference to the element
          */
        T& operator[](size_t i) { return (*this)(i); }

        /**
         * @brief Comparison operator.
         *
         * The comparison operator makes a element wise comparison.
         * Returns true only if all elements are equal.
         *
         * @param rhs [in] RPY to compare with
         * @return True if equal.
         */
        bool operator==(const RPY<T> &rhs) const {
          return (_rpy(0) == rhs(0) && _rpy(1) == rhs(1) && _rpy(2) == rhs(2));
        }

        /**
         * @brief Comparison operator.
         *
         * The comparison operator makes a element wise comparison.
         * Returns true if any of the elements are different.
         *
         * @param rhs [in] RPY to compare with
         * @return True if not equal.
         */
        bool operator!=(const RPY<T> &rhs) const {
            return !(*this == rhs);
        }

        /**
         * @brief size of this RPY.
         * @return the value 3
         */
        size_t size() const { return 3; }

        /**
         * @brief Ouputs RPY to stream
         *
         * @param os [in/out] stream to use
         *
         * @param rpy [in] rpy rotation
         *
         * @return the resulting stream
         */
        friend std::ostream& operator<<(std::ostream& os, const RPY<T>& rpy){
            return os <<"RPY {"<<rpy(0)<<", "<<rpy(1)<<", "<<rpy(2)<<"}";
            //return os << rpy._rpy;
        }

        /**
         * @brief Casts RPY<T> to RPY<Q>
         *
         * @param rpy [in] RPY with type T
         *
         * @return RPY with type Q
         */
        template<class Q>
        friend const RPY<Q> cast(const RPY<T>& rpy) {
            return RPY<Q>(
                static_cast<Q>(rpy(0)),
                static_cast<Q>(rpy(1)),
                static_cast<Q>(rpy(2)));
        }

    private:
        Vector3D<T> _rpy;
    };

    /*@}*/
}} // end namespaces


namespace rw{ namespace common {
    class OutputArchive; class InputArchive;
namespace serialization {
    template<>
    void write(const rw::math::RPY<double>& tmp, rw::common::OutputArchive& oar, const std::string& id);
    template<>
    void read(rw::math::RPY<double>& tmp, rw::common::InputArchive& iar, const std::string& id);
    template<>
    void write(const rw::math::RPY<float>& tmp, rw::common::OutputArchive& oar, const std::string& id);
    template<>
    void read(rw::math::RPY<float>& tmp, rw::common::InputArchive& iar, const std::string& id);
}}} // end namespaces


#endif // end include guard
