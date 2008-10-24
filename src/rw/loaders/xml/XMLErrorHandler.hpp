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

#ifndef RW_LOADERS_XMLERRORHANDLER_HPP
#define RW_LOADERS_XMLERRORHANDLER_HPP

#include <boost/spirit.hpp>
#include <boost/spirit/core.hpp>
#include <boost/spirit/error_handling/exceptions.hpp>
#include <boost/spirit/iterator/position_iterator.hpp>

namespace rw{
namespace loaders{
	/** @addtogroup loaders */
	/*@{*/

	/**
	 * @brief Error handler for XML parsing using boost spirit.
	 *
	 * starttag >> *element >> endtag
	 *
	 * example:
	 * <elemname>
	 *  <elemname2></elemname2>
	 * </elemname>
	 *
	 *
	 */
    class XMLErrorHandler
    {
    public:
        /*
         * @brief XML Error codes
         */
        typedef enum {
            xml_att_unexpected,
            xml_att_expected,
            xml_children_expected,
            xml_end_expected,
            xml_bad_end_elem,
            xml_missing_brack,
            xml_starttag_end_exp,
            xml_bad_child
        } XMLErrors;

        /**
         * @brief XML error object
         */
        struct XMLError {
            std::string _elemname;
            XMLErrors _err;
            XMLError(std::string elemname, XMLErrors err):
                _elemname(elemname),_err(err)
            {}
        };

        /**
         * @brief a static instance of a spirit guard, guarding trown XLErrors
         */
        static boost::spirit::guard<XMLError> XMLErrorGuard;

        /**
         * @brief Asserts if the element name of the endtag does not match the element
         * name of the starttag
         * @param elemname [in] name of the element that is being parsed
         * @return spirit assertion object of XMLError type
         */
        static boost::spirit::assertion<XMLError> BadEndElem(const std::string& elemname){
            return boost::spirit::assertion<XMLError>
                    ( XMLError(elemname,xml_bad_end_elem) );
        }

        /**
         * @brief
         * @param elemname [in] name of the element that is being parsed
         * @return spirit assertion object of XMLError type
         */
        static boost::spirit::assertion<XMLError> MissingBrac(const std::string& elemname){
            return boost::spirit::assertion<XMLError>
                    ( XMLError(elemname,xml_missing_brack) );
        }

        /**
         * @brief
         * @param elemname [in] name of the element that is being parsed
         * @return spirit assertion object of XMLError type
         */
        static boost::spirit::assertion<XMLError> AttExpected(const std::string& elemname){
            return boost::spirit::assertion<XMLError>
                    ( XMLError(elemname,xml_att_expected) );
        }

        /**
         * @brief
         * @param elemname [in] name of the element that is being parsed
         * @return spirit assertion object of XMLError type
         */
        static boost::spirit::assertion<XMLError> StartEndExpected(const std::string& elemname){
            return boost::spirit::assertion<XMLError>
                    ( XMLError(elemname, xml_starttag_end_exp) );
        }

        /**
         * @brief
         * @param elemname [in] name of the element that is being parsed
         * @return spirit assertion object of XMLError type
         */
        static boost::spirit::assertion<XMLError> BadChild(const std::string& elemname){
            return boost::spirit::assertion<XMLError>
                    ( XMLError(elemname, xml_bad_child) );
        }

        template <typename ScannerT, typename ErrorT>
        boost::spirit::error_status<>
        operator()(ScannerT const& scan, ErrorT const& error) const
        {
            boost::spirit::file_position lc = error.where.get_position();
            std::cout << "exception caught in:" << std::endl
                      << " File: " << lc.file
                      << " Line: " << lc.line
                      << " Col: " << lc.column << std::endl;

            std::cout << "Following was parsed until error:" << std::endl
                      << std::string(scan.first,error.where) << std::endl;

            //std::cout << "In this context: " << std::string(scan.first,scan.last) << std::endl;
            std::cout << "Error occoured in parsing element: " << error.descriptor._elemname << std::endl;
            switch( error.descriptor._err ){
            case( xml_bad_end_elem ):
                std::cout << "End element name does not match start element name!" << std::endl;
                break;
            case( xml_missing_brack ):
                std::cout << "Missing end bracket!" << std::endl;
                break;
            case( xml_att_expected ):
                std::cout << "Expected specific attribute here!" << std::endl;
                break;
            case( xml_att_unexpected ):
                std::cout << "Unexpected attribute!" << std::endl;
                break;
            case( xml_starttag_end_exp ):
                std::cout << "Expected '/>' or '>' in start element " << std::endl;
                break;
            case( xml_bad_child ):
                std::cout << "A child node failed to parse!! " << std::endl;
                break;
            default:
                std::cout << "Default.." << std::endl;
            };
            return boost::spirit::error_status<>(boost::spirit::error_status<>::fail);
        }
    };
    /*@}*/

}
}

#endif /*XMLERRORHANDLER_HPP_*/
