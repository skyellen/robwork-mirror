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

#ifndef rwlibs_drawable_Drawable3DS_HPP
#define rwlibs_drawable_Drawable3DS_HPP

/**
 * @file Drawable3DS.hpp
 */

#include "Model_3DS.h"
#include "Drawable.hpp"

#include <rwlibs/os/rwgl.hpp>

#include <cstring>
#include <iostream>

namespace rwlibs { namespace drawable {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief This class loads 3d scenes or objects from a 3ds file
     * format.
     *
     */
    class Drawable3DS : public Drawable {
    private:
        Model_3DS _model;

    public:
        /* Functions inherited from Drawable */
        /**
         * @copydoc Drawable::draw
         */
        //       void draw() const;

        /**
         * @brief creates a Drawable3DS given a 3DS file.
         * @param filename [in] - the path and name of the 3DS file
         */
        Drawable3DS(const std::string &filename);

        virtual ~Drawable3DS(){};

    private:
        /**
         * @copydoc Drawable::update
         */
        void update(UpdateType type);


    };

    /*@}*/
}} // end namespaces

#endif // end include guard
