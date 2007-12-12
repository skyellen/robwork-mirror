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
 * for detailed Actionrmation about these packages.
 *********************************************************************/
#ifndef RW_TASK_TARGET_HPP 
#define RW_TASK_TARGET_HPP 

/**
 * @file Target.hpp
 */

//#include "Task.hpp"

#include "Property.hpp"

#include "Link.hpp"

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

#include <boost/variant.hpp>




namespace rw { namespace task {
	class Link;
	
	/** @addtogroup task */
    /*@{*/

    /**
     * @brief brief Data structure for target specifications in task trajectories.
     *
	 * TODO: Longer description
     */


	class Target
	{	
		friend class Trajectory;
	public:
		typedef std::pair<rw::math::Transform3D<>, std::string> ToolLocation;
		typedef rw::math::Q JointLocation;


		Target(rw::math::Transform3D<> T, std::string frame="");

		Target(rw::math::Q q);

		~Target();

		bool isJoint() { return _value.type() == typeid(JointLocation); }

		bool isToolFrame() { return _value.type() == typeid(ToolLocation); }

		rw::math::Transform3D<> Transform3D();

		rw::math::Vector3D<> P();

		std::string Frame();

		rw::math::Q Joint();

		void SetName(std::string name) { _name = name; }
		std::string Name() { return _name; }

		Property &Properties() { return _properties; };
		
		Link *Next() { return _next; }
		Link *Prev() { return _prev; }



	private:

		void setNext(Link *next) { _next = next; }
		void setPrev(Link *prev) { _prev = prev; } 


		boost::variant<JointLocation, ToolLocation > _value;

		std::string _name;

		std::string _frame;
	
		Property _properties;

		Link *_prev, *_next;


	};


}// end task namespace
}// end rw namespace

#endif

