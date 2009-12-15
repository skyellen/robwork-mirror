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


#ifndef RWLIBS_TASK_XMLTASKLOADER_HPP
#define RWLIBS_TASK_XMLTASKLOADER_HPP



#include "../Task.hpp"
#include "../Entity.hpp"

#include <xercesc/dom/DOMElement.hpp>

#include <string>


namespace rwlibs {
namespace task {


/** @addtogroup task */
/*@{*/


class XMLTaskLoader {
public:
	XMLTaskLoader();
	virtual ~XMLTaskLoader();


	void load(const std::string& filename, const std::string& schemaFileName = "");
	void load(std::istream& instream, const std::string& schemaFileName);

	rwlibs::task::TaskBasePtr readTask(xercesc::DOMElement* element);

//	static rw::task3::TaskBasePtr readTargets(xercesc::DOMElement* element);



	void readEntityData(xercesc::DOMElement* element, rw::common::Ptr<rwlibs::task::Entity> entity);


	rwlibs::task::ActionPtr readAction(xercesc::DOMElement* element);

	template <class T>
	rw::common::Ptr<rwlibs::task::Motion<T> > readMotion(xercesc::DOMElement* element);

	template <class T>
	rw::common::Ptr<rwlibs::task::Target<T> > readTarget(xercesc::DOMElement* element);

	template <class T>
	void readTargets(xercesc::DOMElement* element, rw::common::Ptr<rwlibs::task::Task<T> > task);

	template <class T>
	void readEntities(xercesc::DOMElement* element, rw::common::Ptr<rwlibs::task::Task<T> > task);



	void readAugmentations(xercesc::DOMElement* element, rwlibs::task::TaskBasePtr task);


	template <class T>
	rw::common::Ptr<rwlibs::task::Task<T> > readTemplateTask(xercesc::DOMElement* element);

	rwlibs::task::QTaskPtr getQTask();

	rwlibs::task::CartesianTaskPtr getCartesianTask();

	rwlibs::task::TaskBasePtr getTask();

private:
	typedef std::map<std::string, rwlibs::task::TargetPtr> TargetMap;
	TargetMap _targetMap;



	rwlibs::task::QTaskPtr _qTask;
	rwlibs::task::CartesianTaskPtr _cartTask;
	rwlibs::task::TaskBasePtr _task;
};

/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //end include guard
