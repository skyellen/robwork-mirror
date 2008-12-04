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

#ifndef RW_LOADERS_XMLPARSER_HPP
#define RW_LOADERS_XMLPARSER_HPP

#include <boost/spirit/phoenix.hpp>
#include <boost/spirit.hpp>
#include <boost/spirit/core.hpp>
#include <boost/spirit/symbols/symbols.hpp>
#include <boost/spirit/error_handling/exceptions.hpp>
#include <boost/spirit/iterator/position_iterator.hpp>

#include "XMLErrorHandler.hpp"

namespace rw {
namespace loaders {

    template<typename ParsableAttrT, typename ParsableElemT>
    struct XMLAttElemParser: public boost::spirit::parser< rw::loaders::XMLAttElemParser<ParsableAttrT,ParsableElemT> >
    {
    private:
        std::string _elemname;
        typename boost::spirit::as_parser<ParsableAttrT>::type::embed_t _attr;
        typename boost::spirit::as_parser<ParsableElemT>::type::embed_t _elem;
        bool _parseAttr;

    public:
        typedef XMLAttElemParser<ParsableAttrT, ParsableElemT> self_t;

        XMLAttElemParser( std::string const& elemname,
                       ParsableAttrT const& attr,
                       ParsableElemT const& elem,
                       bool parseAttr):
                           _elemname( elemname ),
                           _attr(attr),
                           _elem(elem),
                           _parseAttr(parseAttr)
        {}

        template <typename ScannerT>
        typename boost::spirit::parser_result<self_t, ScannerT>::type
        parse(ScannerT const& scan) const
        {
            typedef typename boost::spirit::parser_result<self_t, ScannerT>::type result_t;
            result_t hit = scan.empty_match();
            result_t tmphit = hit;

            typename ScannerT::iterator_t const  save(scan.first);

            // parse start tag
            tmphit = boost::spirit::ch_p('<').parse(scan);
            if( !tmphit ) {
                return scan.no_match();
            }
            scan.concat_match(hit, tmphit);

            tmphit = boost::spirit::str_p( _elemname.c_str() ).parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            if(_parseAttr) {
                // parse attributes
                tmphit = XMLErrorHandler::AttExpected(_elemname)
                             ( _attr ).parse(scan);
                scan.concat_match(hit, tmphit);
            }
            // parse startendtag either "/>" og ">"
            tmphit = boost::spirit::str_p("/>").parse(scan);
            if(tmphit){
                scan.concat_match(hit, tmphit);
                return hit;
            }

            tmphit = XMLErrorHandler::StartEndExpected(_elemname)
                        (boost::spirit::ch_p('>')).parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            // if the last then allso parse element
            tmphit = XMLErrorHandler::BadChild(_elemname)
                         (_elem).parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            // and endtag "</elemname>"
            tmphit = XMLErrorHandler::MissingBrac(_elemname)
                (boost::spirit::str_p("</")).parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            tmphit = XMLErrorHandler::BadEndElem(_elemname)
                (boost::spirit::str_p( _elemname.c_str() )).parse(scan);
                //(str_p( _elemname.c_str() )).parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            tmphit = XMLErrorHandler::MissingBrac(_elemname)
                        (boost::spirit::ch_p('>')).parse(scan);
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
    struct XMLAttParser: public boost::spirit::parser< XMLAttParser<ParsableAttT> >
    {
    private:
        std::string _attrname;
        typename boost::spirit::as_parser<ParsableAttT>::type::embed_t _attr;

    public:
        typedef XMLAttParser<ParsableAttT> self_t;

        XMLAttParser( std::string const& attrname,
                       ParsableAttT const& attr):
                           _attrname( attrname ),
                           _attr(attr)
        {}

        template <typename ScannerT>
        typename boost::spirit::parser_result<self_t, ScannerT>::type
        parse(ScannerT const& scan) const
        {
            typedef typename boost::spirit::parser_result<self_t, ScannerT>::type result_t;
            result_t hit = scan.empty_match();
            result_t tmphit = hit;

            typename ScannerT::iterator_t const  save(scan.first);

            tmphit = boost::spirit::str_p( _attrname.c_str() ).parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            tmphit = boost::spirit::ch_p('=').parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            tmphit = boost::spirit::ch_p('"').parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            tmphit = _attr.parse(scan);
            if( !tmphit ) return scan.no_match();
            scan.concat_match(hit, tmphit);

            tmphit = boost::spirit::ch_p('"').parse(scan);
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

