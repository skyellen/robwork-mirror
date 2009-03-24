/*
 * XMLTaskLoader.hpp
 *
 *  Created on: Mar 17, 2009
 *      Author: lpe
 */

#ifndef RW_LOADERS_XMLTASKLOADER_HPP
#define RW_LOADERS_XMLTASKLOADER_HPP



#include <sandbox/task3/Task.hpp>
#include <sandbox/task3/Entity.hpp>

#include <xercesc/dom/DOMElement.hpp>

#include <string>


namespace rw {
namespace loaders {


class XMLTaskLoader {
public:
	XMLTaskLoader();
	virtual ~XMLTaskLoader();

	void load(const std::string& filename, const std::string& schemaFileName = "");

	rw::task3::TaskBasePtr readTask(xercesc::DOMElement* element);

//	static rw::task3::TaskBasePtr readTargets(xercesc::DOMElement* element);



	void readEntityData(xercesc::DOMElement* element, rw::common::Ptr<rw::task3::Entity> entity);


	rw::task3::ActionPtr readAction(xercesc::DOMElement* element);

	template <class T>
	rw::common::Ptr<rw::task3::Motion<T> > readMotion(xercesc::DOMElement* element);

	template <class T>
	rw::common::Ptr<rw::task3::Target<T> > readTarget(xercesc::DOMElement* element);

	template <class T>
	void readTargets(xercesc::DOMElement* element, rw::common::Ptr<rw::task3::Task<T> > task);

	template <class T>
	void readEntities(xercesc::DOMElement* element, rw::common::Ptr<rw::task3::Task<T> > task);



	void readAugmentations(xercesc::DOMElement* element, rw::task3::TaskBasePtr task);


	template <class T>
	rw::common::Ptr<rw::task3::Task<T> > readTemplateTask(xercesc::DOMElement* element);

	rw::task3::QTaskPtr getQTask();

	rw::task3::CartesianTaskPtr getCartesianTask();

	rw::task3::TaskBasePtr getTask();

private:
	typedef std::map<std::string, rw::task3::TargetPtr> TargetMap;
	TargetMap _targetMap;



	rw::task3::QTaskPtr _qTask;
	rw::task3::CartesianTaskPtr _cartTask;
	rw::task3::TaskBasePtr _task;
};

} //end namespace loaders
} //end namespace rw

#endif //end include guard
