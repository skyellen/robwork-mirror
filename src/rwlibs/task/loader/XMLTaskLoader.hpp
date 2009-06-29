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
