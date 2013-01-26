#ifndef RW_DOMPARSER_HPP
#define RW_DOMPARSER_HPP

#include <string>
#include <vector>
#include <list>

#include "DOMElem.hpp"
#include <rw/common/Ptr.hpp>

namespace rw {
namespace common {

	/**
	 * @brief interface for parsing documents in a DOM fasion.
	 */
	class DOMParser {
	protected:
		DOMParser(){};

	public:

		//! smart pointer type
		typedef rw::common::Ptr<DOMParser> Ptr;

		//! destructor
		virtual ~DOMParser(){}

		// loading and saving
		virtual void load(const std::string& filename) = 0;
		virtual void load(std::istream& input) = 0;
		virtual void save(const std::string& filename) = 0;
		virtual void save(std::ostream& input) = 0;

		virtual void setSchema(const std::string& filename){ _schemaFile = filename; }

		virtual DOMElem::Ptr getRootElement() = 0;

		// extra stuff that can be added in top of document
		static rw::common::Ptr<DOMParser> make();

	protected:
		std::string _schemaFile;
	};

}} //namespace

#endif
