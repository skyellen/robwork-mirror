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


#ifndef RW_LOADERS_XMLERRORHANDLER_HPP
#define RW_LOADERS_XMLERRORHANDLER_HPP

#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_exceptions.hpp>
#include <boost/spirit/include/classic_position_iterator.hpp>

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
        static boost::spirit::classic::guard<XMLError> XMLErrorGuard;

        /**
         * @brief Asserts if the element name of the endtag does not match the element
         * name of the starttag
         * @param elemname [in] name of the element that is being parsed
         * @return spirit assertion object of XMLError type
         */
        static boost::spirit::classic::assertion<XMLError> BadEndElem(const std::string& elemname){
            return boost::spirit::classic::assertion<XMLError>
                    ( XMLError(elemname,xml_bad_end_elem) );
        }

        /**
         * @brief
         * @param elemname [in] name of the element that is being parsed
         * @return spirit assertion object of XMLError type
         */
        static boost::spirit::classic::assertion<XMLError> MissingBrac(const std::string& elemname){
            return boost::spirit::classic::assertion<XMLError>
                    ( XMLError(elemname,xml_missing_brack) );
        }

        /**
         * @brief
         * @param elemname [in] name of the element that is being parsed
         * @return spirit assertion object of XMLError type
         */
        static boost::spirit::classic::assertion<XMLError> AttExpected(const std::string& elemname){
            return boost::spirit::classic::assertion<XMLError>
                    ( XMLError(elemname,xml_att_expected) );
        }

        /**
         * @brief
         * @param elemname [in] name of the element that is being parsed
         * @return spirit assertion object of XMLError type
         */
        static boost::spirit::classic::assertion<XMLError> StartEndExpected(const std::string& elemname){
            return boost::spirit::classic::assertion<XMLError>
                    ( XMLError(elemname, xml_starttag_end_exp) );
        }

        /**
         * @brief
         * @param elemname [in] name of the element that is being parsed
         * @return spirit assertion object of XMLError type
         */
        static boost::spirit::classic::assertion<XMLError> BadChild(const std::string& elemname){
            return boost::spirit::classic::assertion<XMLError>
                    ( XMLError(elemname, xml_bad_child) );
        }

        template <typename ScannerT, typename ErrorT>
        boost::spirit::classic::error_status<>
        operator()(ScannerT const& scan, ErrorT const& error) const
        {
            boost::spirit::classic::file_position lc = error.where.get_position();
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
            return boost::spirit::classic::error_status<>(boost::spirit::classic::error_status<>::fail);
        }
    };
    /*@}*/

}
}

#endif /*XMLERRORHANDLER_HPP_*/
