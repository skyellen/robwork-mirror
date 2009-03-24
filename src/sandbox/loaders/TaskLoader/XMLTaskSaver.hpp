/*
 * XMLTaskSaver.hpp
 *
 *
 *  Created on: Mar 23, 2009
 *      Author: lpe
 */

#ifndef RW_LOADERS_XMLTASKSAVER_HPP
#define RW_LOADERS_XMLTASKSAVER_HPP

#include <sandbox/task3/Task.hpp>

#include <xercesc/dom/DOM.hpp>

namespace rw {
namespace loaders {


class XMLTaskSaver {
public:
	XMLTaskSaver();
	virtual ~XMLTaskSaver();

	static bool save(rw::task3::QTaskPtr task, const std::string& filename);

	static bool save(rw::task3::CartesianTaskPtr task, const std::string& filename);

	void writeTaskBase(rw::task3::TaskBasePtr task, xercesc::DOMElement* parent, xercesc::DOMDocument* doc);

private:
	template <class T>
	bool saveImpl(rw::common::Ptr<rw::task3::Task<T> > task, const std::string& filename);

	void writeEntityInfo(rw::task3::EntityPtr entity, xercesc::DOMElement* element, xercesc::DOMDocument* doc);

	template <class T>
	void writeTargets(rw::common::Ptr<rw::task3::Task<T> > task, xercesc::DOMElement* element, xercesc::DOMDocument* document);

	template <class T>
	void writeMotion(rw::common::Ptr<rw::task3::Motion<T> > motion, xercesc::DOMElement* element, xercesc::DOMDocument* doc);


	template <class T>
	void writeEntities(rw::common::Ptr<rw::task3::Task<T> > task, xercesc::DOMElement* element, xercesc::DOMDocument* doc);

	void writeAction(rw::task3::ActionPtr action, xercesc::DOMElement* element, xercesc::DOMDocument* doc);



	template <class T>
	void writeTaskToElement(rw::common::Ptr<rw::task3::Task<T> > task, xercesc::DOMElement* element, xercesc::DOMDocument* doc);

	template <class T>
	void writeTask(rw::common::Ptr<rw::task3::Task<T> > task, xercesc::DOMElement* element, xercesc::DOMDocument* doc);



	std::map<rw::task3::TargetPtr, std::string> _targetMap;
};

} //end namespace loaders
} //end namespace rw

#endif //end include guard
