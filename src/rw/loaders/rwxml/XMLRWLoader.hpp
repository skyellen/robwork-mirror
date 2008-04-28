#ifndef XMLRWLOADER_HPP_
#define XMLRWLOADER_HPP_

#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

namespace rw { namespace loaders {
	/** @addtogroup loaders */
	/*@{*/

	/**
	 * @brief this class loads a workcell in xml format from a filename.
	 *  
	 */
    class XMLRWLoader
    {
    public:
        /**
         * @brief Loads/imports robwork workcell in XML file format
         *
         * An exception is thrown if the file can't be loaded.
         *
         * @param filename [in] filename of XML file
         */
        static std::auto_ptr<rw::models::WorkCell> loadWorkCell(
            const std::string& filename);
    };

	/*@}*/
}} // end namespaces

#endif // end include guard
