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

#ifndef RW_TASK_PROPERTY_HPP 
#define RW_TASK_PROPERTY_HPP 

/**
 * @file TaskProperty.hpp
 */
#include <rw/common/PropertyMap.hpp>

#include <iostream>
#include <string.h>

namespace rw { namespace task {


	/** @addtogroup task */
    /*@{*/

    /**
     * @brief Data structure for task action specifications.
     *
	 * TODO: Longer description
     */


	class TaskProperty
	{	
	public:
		TaskProperty();
		~TaskProperty();

		void addProperty(std::string key, double val);
		void addProperty(std::string key, int val);
		void addProperty(std::string key, std::string val);

		bool getProperty(std::string key, double &val);
		bool getProperty(std::string key, int &val);
		bool getProperty(std::string key, std::string &val);

		void setSpecial(bool special) { special = _special; }
		bool getSpecial() { return _special; }


	private:
		bool _special;

		common::PropertyMap *_properties;


	};


}// end task namespace
}// end rw namespace

#endif