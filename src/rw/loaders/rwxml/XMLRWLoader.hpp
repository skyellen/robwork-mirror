#ifndef XMLRWLOADER_HPP_
#define XMLRWLOADER_HPP_

#include <rw/models/WorkCell.hpp>
#include <rw/models/DeviceModel.hpp>

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
        static std::auto_ptr<rw::models::WorkCell> LoadWorkCell(
            const std::string& filename);

        /**
         * brief Loads/imports robwork device in XML file format
         *
         * An exception is thrown if the file can't be loaded.
         *
         * param filename [in] filename of XML file
         */
        //static std::auto_ptr<rw::models::DeviceModel> LoadDeviceModel(
        //    const std::string& filename);

    };
	/*@}*/
}}
#endif /*XMLRWLOADER_HPP_*/
