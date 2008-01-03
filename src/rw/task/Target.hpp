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
#include <rw/kinematics/Frame.hpp>

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


	class ToolLocation
	{
	public:
		ToolLocation(const rw::math::Transform3D<> &T, rw::kinematics::Frame *frame) : _T(T), _frame(frame)
		{
		}
	
		rw::math::Transform3D<> &getTransform() { return _T; }
		rw::kinematics::Frame *getFrame() { return _frame; }


	private:
		rw::math::Transform3D<> _T;
		rw::kinematics::Frame *_frame;
	};


	class Target
	{	
		friend class Trajectory;
	public:

		Target::Target(const boost::variant<rw::math::Q, ToolLocation > &value, const std::string &name="");

		~Target();

		bool isQ() { return _value.type() == typeid(rw::math::Q); }

		bool isToolLocation() { return _value.type() == typeid(ToolLocation); }

		ToolLocation &getToolLocation();

		rw::math::Q &getQ();

		std::string getName() { return _name; }

		Property &Properties() { return _properties; };
		
		Link *next() { return _next; }
		Link *nrev() { return _prev; }



	private:

		void setNext(Link *next) { _next = next; }
		void setPrev(Link *prev) { _prev = prev; } 


		boost::variant<rw::math::Q, ToolLocation > _value;

		std::string _name;

		std::string _frame;
	
		Property _properties;

		Link *_prev, *_next;


	};


}// end task namespace
}// end rw namespace

#endif

