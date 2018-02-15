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

#include "TaskLoader.hpp"
#include "../Task.hpp"
#include "../Entity.hpp"

#include <xercesc/util/XercesDefs.hpp>

#include <string>

XERCES_CPP_NAMESPACE_BEGIN
class DOMElement;
XERCES_CPP_NAMESPACE_END

namespace rwlibs {
namespace task {


/** @addtogroup task */
/*@{*/
/**
 * @brief Loader for the XML task format, based on Xerces.
 *
 * Please consider using the newer DOMTaskLoader instead.
 */
class XMLTaskLoader: public TaskLoader {
public:
	//! @brief Constructor.
	XMLTaskLoader() {}

	//! @brief Destructor.
	virtual ~XMLTaskLoader() {}

	//! @copydoc TaskLoader::load(const std::string&, const std::string&)
	void load(const std::string& filename, const std::string& schemaFileName = "");

	//! @copydoc TaskLoader::load(std::istream&, const std::string&)
	void load(std::istream& instream, const std::string& schemaFileName = "");

	//! @copydoc TaskLoader::getQTask
	rwlibs::task::QTask::Ptr getQTask();

	//! @copydoc TaskLoader::getCartesianTask
	rwlibs::task::CartesianTask::Ptr getCartesianTask();

	//! @copydoc TaskLoader::getTask
	rwlibs::task::TaskBase::Ptr getTask();

	//! @copydoc TaskLoader::clone
	TaskLoader::Ptr clone() const;

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the XMLTaskLoader is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if XMLTaskLoader is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	rwlibs::task::TaskBasePtr readTask(xercesc::DOMElement* element);

	void readEntityData(xercesc::DOMElement* element, rw::common::Ptr<rwlibs::task::Entity> entity);


	rwlibs::task::Action::Ptr readAction(xercesc::DOMElement* element);

	template <class T>
	typename rwlibs::task::Motion<T>::Ptr readMotion(xercesc::DOMElement* element);

	template <class T>
	typename rwlibs::task::Target<T>::Ptr readTarget(xercesc::DOMElement* element);

	template <class T>
	void readTargets(xercesc::DOMElement* element, typename rwlibs::task::Task<T>::Ptr task);

	template <class T>
	void readEntities(xercesc::DOMElement* element, typename rwlibs::task::Task<T>::Ptr task);



	void readAugmentations(xercesc::DOMElement* element, rwlibs::task::TaskBase::Ptr task);


	template <class T>
	typename rwlibs::task::Task<T>::Ptr readTemplateTask(xercesc::DOMElement* element);

private:
	static const Initializer initializer;

	typedef std::map<std::string, rwlibs::task::TargetBase::Ptr> TargetMap;
	TargetMap _targetMap;

	rwlibs::task::QTask::Ptr _qTask;
	rwlibs::task::CartesianTask::Ptr _cartTask;
	rwlibs::task::TaskBase::Ptr _task;
};

/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //end include guard
