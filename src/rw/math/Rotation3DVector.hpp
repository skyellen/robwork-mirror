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

#ifndef rw_math_Rotation3DVector_HPP
#define rw_math_Rotation3DVector_HPP

/**
 * @file Rotation3DVector.hpp
 */

// introduce cyclic dependancy when deprecated is on...
#include "Rotation3D.hpp"

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{*/

    /**
     * @brief An abstract base class for Rotation3D parameterisations
     *
     * Classes that represents a parametrisation of a 3D rotation may inherit
     * from this class
     */
    template<class T = double>
    class Rotation3DVector{
    public:
        /**
         * @brief Virtual destructor
         */
        virtual ~Rotation3DVector(){}

        /**
         * @brief Returns the corresponding @f$ 3\times 3 @f$ Rotation matrix
         * @return The rotation matrix
         */
        virtual Rotation3D<T> toRotation3D() const = 0;

    protected:
        /**
         * @brief Copy Constructor
         *
         * We allow subclasses of this class to be copied.
         */
        Rotation3DVector(const Rotation3DVector&) {}

        /**
         * @brief Assignment operator is protected to force subclasses to
         * implement it by themself.
         */
        Rotation3DVector& operator=(const Rotation3DVector&) { return *this;}

    protected:
        /**
         * @brief Default Constructor
         */
        Rotation3DVector() {}
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
