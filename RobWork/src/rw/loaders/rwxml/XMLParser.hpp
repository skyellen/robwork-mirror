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


#ifndef RW_LOADERS_XMLPARSER_HPP
#define RW_LOADERS_XMLPARSER_HPP

#include <complex>
#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/phoenix1.hpp>
#include <boost/spirit/include/classic_symbols.hpp>
#include <boost/spirit/include/classic_exceptions.hpp>
#include <boost/spirit/include/classic_position_iterator.hpp>

#include "XMLErrorHandler.hpp"

namespace rw {
namespace loaders {

    /**
     * @brief parser for xml elements with both attributes and child elements
     */
    template<typename ParsableAttrT, typename ParsableElemT>
    struct XMLAttElemParser: public boost::spirit::classic::parser< rw::loaders::XMLAttElemParser<ParsableAttrT,ParsableElemT> >
    {
    private:
        std::string _elemname;
        typename boost::spirit::classic::as_parser<ParsableAttrT>::type::embed_t _attr;
        typename boost::spirit::classic::as_parser<ParsableElemT>::type::embed_t _elem;
        bool _parseAttr;

    public:

        typedef XMLAttElemParser<ParsableAttrT, ParsableElemT> self_t;
        //! @brief constructor
        XMLAttElemParser( std::string const& elemname,
                       ParsableAttrT const& attr,
                       ParsableElemT const& elem,
                       bool parseAttr):
                           _elemname( elemname ),
                           _attr(attr),
                           _elem(elem),
                           _parseAttr(parseAttr)
        {}

        /**
         *
         * @param scan
         * @return
         */
        template <typename ScannerT>
        typename boost::spirit::classic::parser_result<self_t, ScannerT>::type
        parse(ScannerT const& scan) const
        {
            typedef typename boost::spirit::classic::parser_result<self_t, ScannerT>::type result_t;
            result_t hit = scan.empty_match();
            result_t tmphit = hit;

            typename ScannerT::iterator_t const  save(scan.first);

            // parse start tag
            tmphit = boost::spirit::classic::ch_p('<').parse(scan);
            if( !tmphit ) {
                return scan.no_match();
            }
            scan.concat_match(hit, tmphit);

            tmphit = boost::spirit::classic::str_p( _elemname.c_str() ).parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            if(_parseAttr) {
                // parse attributes
                tmphit = XMLErrorHandler::AttExpected(_elemname)
                             ( _attr ).parse(scan);
                scan.concat_match(hit, tmphit);
            }
            // parse startendtag either "/>" og ">"
            tmphit = boost::spirit::classic::str_p("/>").parse(scan);
            if(tmphit){
                scan.concat_match(hit, tmphit);
                return hit;
            }

            tmphit = XMLErrorHandler::StartEndExpected(_elemname)
                        (boost::spirit::classic::ch_p('>')).parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            // if the last then allso parse element
            tmphit = XMLErrorHandler::BadChild(_elemname)
                         (_elem).parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            // and endtag "</elemname>"
            tmphit = XMLErrorHandler::MissingBrac(_elemname)
                (boost::spirit::classic::str_p("</")).parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            tmphit = XMLErrorHandler::BadEndElem(_elemname)
                (boost::spirit::classic::str_p( _elemname.c_str() )).parse(scan);
                //(str_p( _elemname.c_str() )).parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            tmphit = XMLErrorHandler::MissingBrac(_elemname)
                        (boost::spirit::classic::ch_p('>')).parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            return hit;
        }
    };



    template<typename ParsableAttrT, typename ParsableElemT>
    XMLAttElemParser<ParsableAttrT, ParsableElemT> XMLAttElem_p(
                                       const std::string& elemname,
                                       ParsableAttrT const& attr,
                                       ParsableElemT const& elem)
    {
        return XMLAttElemParser<ParsableAttrT,ParsableElemT>
                   ( elemname, attr, elem, true);
    }

    template<typename ParsableAttT>
    XMLAttElemParser<ParsableAttT,ParsableAttT> XMLElem_p(
                                       const std::string& elemname,
                                       ParsableAttT const& elem)
    {
        return XMLAttElemParser<ParsableAttT,ParsableAttT>( elemname, elem, elem, false);
    }


    template<typename ParsableAttT>
    struct XMLAttParser: public boost::spirit::classic::parser< XMLAttParser<ParsableAttT> >
    {
    private:
        std::string _attrname;
        typename boost::spirit::classic::as_parser<ParsableAttT>::type::embed_t _attr;

    public:
        typedef XMLAttParser<ParsableAttT> self_t;

        XMLAttParser( std::string const& attrname,
                       ParsableAttT const& attr):
                           _attrname( attrname ),
                           _attr(attr)
        {}

        template <typename ScannerT>
        typename boost::spirit::classic::parser_result<self_t, ScannerT>::type
        parse(ScannerT const& scan) const
        {
            typedef typename boost::spirit::classic::parser_result<self_t, ScannerT>::type result_t;
            result_t hit = scan.empty_match();
            result_t tmphit = hit;

            typename ScannerT::iterator_t const  save(scan.first);

            tmphit = boost::spirit::classic::str_p( _attrname.c_str() ).parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            tmphit = boost::spirit::classic::ch_p('=').parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            tmphit = boost::spirit::classic::ch_p('"').parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            tmphit = _attr.parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            tmphit = boost::spirit::classic::ch_p('"').parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            return hit;
        }
    };

    template<typename ParsableAttT>
    XMLAttParser<ParsableAttT> XMLAtt_p(
                                       const std::string& attrname,
                                       ParsableAttT const& attr)
    {
        return XMLAttParser<ParsableAttT>( attrname, attr );
    }


} // loaders namespace
} // rw namespace


#endif /*XMLPARSER_HPP_*/

