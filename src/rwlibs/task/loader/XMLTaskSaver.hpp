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

	static bool save(rwlibs::task::QTaskPtr task, const std::string& filename);

	static bool save(rwlibs::task::CartesianTaskPtr task, const std::string& filename);

	void writeTask(rwlibs::task::TaskBasePtr task, xercesc::DOMElement* parent, xercesc::DOMDocument* doc);

private:
	template <class T>
	bool saveImpl(rw::common::Ptr<rwlibs::task::Task<T> > task, const std::string& filename);

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
