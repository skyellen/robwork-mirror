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

#include <xercesc/dom/DOM.hpp>

namespace rwlibs {
namespace task {


/** @addtogroup task */
/*@{*/


class XMLTaskSaver {
public:
	XMLTaskSaver();
	virtual ~XMLTaskSaver();


    static bool save(rwlibs::task::QTaskPtr task, std::ostream& outstream);

    static bool save(rwlibs::task::CartesianTaskPtr task, std::ostream& outstream);

	static bool save(rwlibs::task::QTaskPtr task, const std::string& filename);

	static bool save(rwlibs::task::CartesianTaskPtr task, const std::string& filename);

	void writeTask(rwlibs::task::TaskBasePtr task, xercesc::DOMElement* parent, xercesc::DOMDocument* doc);

private:
	template <class T>
	bool saveImpl(rw::common::Ptr<rwlibs::task::Task<T> > task, xercesc::XMLFormatTarget* target);

	void writeEntityInfo(rwlibs::task::EntityPtr entity, xercesc::DOMElement* element, xercesc::DOMDocument* doc);

	template <class T>
	void writeTargets(rw::common::Ptr<rwlibs::task::Task<T> > task, xercesc::DOMElement* element, xercesc::DOMDocument* document);

	template <class T>
	void writeMotion(rw::common::Ptr<rwlibs::task::Motion<T> > motion, xercesc::DOMElement* element, xercesc::DOMDocument* doc);


	template <class T>
	void writeEntities(rw::common::Ptr<rwlibs::task::Task<T> > task, xercesc::DOMElement* element, xercesc::DOMDocument* doc);

	void writeAction(rwlibs::task::ActionPtr action, xercesc::DOMElement* element, xercesc::DOMDocument* doc);



	template <class T>
	void writeTaskToElement(rw::common::Ptr<rwlibs::task::Task<T> > task, xercesc::DOMElement* element, xercesc::DOMDocument* doc);

	template <class T>
	void writeTaskImpl(rw::common::Ptr<rwlibs::task::Task<T> > task, xercesc::DOMElement* element, xercesc::DOMDocument* doc);


	std::map<rwlibs::task::TargetPtr, std::string> _targetMap;
};

/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //end include guard
