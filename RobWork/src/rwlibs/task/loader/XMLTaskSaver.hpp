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



#ifndef RWLIBS_TASK_XMLTASKSAVER_HPP
#define RWLIBS_TASK_XMLTASKSAVER_HPP

#include "../Task.hpp"

#include <xercesc/util/XercesDefs.hpp>

XERCES_CPP_NAMESPACE_BEGIN
class DOMDocument;
class DOMElement;
class XMLFormatTarget;
XERCES_CPP_NAMESPACE_END

namespace rwlibs {
namespace task {


/** @addtogroup task */
/*@{*/


class XMLTaskSaver {
public:
	XMLTaskSaver();
	virtual ~XMLTaskSaver();


	static bool save(rwlibs::task::QTask::Ptr task, std::ostream& outstream);

	static bool save(rwlibs::task::CartesianTask::Ptr task, std::ostream& outstream);

	static bool save(rwlibs::task::QTask::Ptr task, const std::string& filename);

	static bool save(rwlibs::task::CartesianTask::Ptr task, const std::string& filename);

	void writeTask(rwlibs::task::TaskBase::Ptr task, xercesc::DOMElement* parent, xercesc::DOMDocument* doc);

private:
	template <class T>
	bool saveImpl(typename rwlibs::task::Task<T>::Ptr task, xercesc::XMLFormatTarget* target);

	void writeEntityInfo(rwlibs::task::Entity::Ptr entity, xercesc::DOMElement* element, xercesc::DOMDocument* doc);



	template <class T>
	void writeTargets(typename rwlibs::task::Task<T>::Ptr task, xercesc::DOMElement* element, xercesc::DOMDocument* document);

	template <class T>
	void writeMotion(typename rwlibs::task::Motion<T>::Ptr motion, xercesc::DOMElement* element, xercesc::DOMDocument* doc);


	template <class T>
	void writeEntities(typename rwlibs::task::Task<T>::Ptr task, xercesc::DOMElement* element, xercesc::DOMDocument* doc);

	void writeAction(rwlibs::task::Action::Ptr action, xercesc::DOMElement* element, xercesc::DOMDocument* doc);



	template <class T>
	void writeTaskToElement(typename rwlibs::task::Task<T>::Ptr task, xercesc::DOMElement* element, xercesc::DOMDocument* doc);

	template <class T>
	void writeTaskImpl(typename rwlibs::task::Task<T>::Ptr task, xercesc::DOMElement* element, xercesc::DOMDocument* doc);


	std::map<rwlibs::task::TargetBase::Ptr, std::string> _targetMap;

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the XMLTaskSaver is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if XMLTaskSaver is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;
};

/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //end include guard
