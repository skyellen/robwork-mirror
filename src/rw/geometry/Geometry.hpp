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

#ifndef RW_GEOMETRY_GEOMETRY_HPP
#define RW_GEOMETRY_GEOMETRY_HPP


#include "Face.hpp"
#include <list>

namespace rw { namespace geometry {

    /**
     * @brief Geometry provides an interface for geometries which
     * can be used for visualization and collision detection
     */
    class Geometry {
    	std::string _id;
    public:

    	/**
    	 * @brief constructor
    	 * @param id [in] Unique identifier of the geometry instance
    	 */
    	Geometry(const std::string& id):
    		_id(id)
    	{}

    	/**
    	 * @brief destructor
    	 */
        virtual ~Geometry() {}

        /**
         * @brief Returns reference to vector of faces
         *
         * When the Geometry object is delete the reference becomes invalid.
         * If the faces are needed afterwards, then copy the list.
         *
         * @return Reference to vector of faces
         */
        virtual const std::vector<Face<float> >& getFaces() const = 0;

        /**
         * @brief get the unique id of this geometry instance
         */
        virtual const std::string& getId() const {
        	return _id;
        }

    protected:
        Geometry(const Geometry& other);
        Geometry& operator=(const Geometry& other);
    };

}} // end namespaces

#endif // end include guard
