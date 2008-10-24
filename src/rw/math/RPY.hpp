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

#ifndef RW_MATH_RPY_HPP
#define RW_MATH_RPY_HPP

/**
 * @file RPY.hpp
 */

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
        Rotation3D<T> toRotation3D() const;

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
         * @brief Ouputs RPY to stream
         *
         * @param os [in/out] stream to use
         *
         * @param rpy [in] rpy rotation
         *
         * @return the resulting stream
         */
        friend std::ostream& operator<<(std::ostream& os, const RPY<T>& rpy){
            return os << rpy._rpy;
        }

        /**
         * @brief Casts RPY<T> to RPY<Q>
         *
         * @param rpy [in] RPY with type T
         *
         * @return RPY with type Q
         */
        template<class Q>
        friend RPY<Q> cast(const RPY<T>& rpy) {
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

#endif // end include guard
