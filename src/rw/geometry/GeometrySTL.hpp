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

#ifndef rw_geometry_GeometrySTL_HPP
#define rw_geometry_GeometrySTL_HPP

/**
 * @file GeometrySTL.hpp
 */

#include "Face.hpp"

namespace rw { namespace geometry {
    /** @addtogroup geometry */
    /*@{*/

    /**
     * @brief This class loads in STL geometry objects
     */

    class GeometrySTL {
    public:
        /**
         * @brief loads a stl file. Automaticly detects if the STL format is
         * ASCII or Binary
         *
         * @param filename [in] path and name of file to load
         *
         * @param result [out] vector into which the result is stored
         */
        static void load(
            const std::string &filename,
            std::vector<Face<float> >& result);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
