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


#include "XMLRWPreParser.hpp"


#include "XMLParserUtil.hpp"
#include "DependencyGraph.hpp"

#include "XMLErrorHandler.hpp"
#include "XMLParser.hpp"

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>

#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_symbols.hpp>

#include <fstream>
#include <iostream>
#include <vector>

using namespace boost::spirit::classic;
using namespace phoenix;

using namespace rw::loaders;
using namespace rw::common;

namespace {

    struct AppendToOutputFromFile{
        AppendToOutputFromFile(std::vector<char> &output,
                               const std::string& currentfile,
                               DependencyGraph &graph,
                               std::vector< std::pair<size_t,file_position> > &filemap)
            : _output(output),_currentfile(currentfile),_graph(graph),_filemap(filemap)
        {}

        void operator()(std::string const& filename) const {
            append(filename);
        }

        template < typename IteratorT >
        void operator()(IteratorT const& first, IteratorT const& last) const {
           append(std::string(first,last));
        }

        void append( std::string const& filename) const
        {
            std::string absfilename;
            if( StringUtil::isAbsoluteFileName( filename ) ){
                absfilename = filename;
            } else {
                // get directory name from filemap
                std::string dir = StringUtil::getDirectoryName(
                    _filemap.back().second.file );
                absfilename = dir + filename;
            }

            //std::cout << "Absolute filename: " << absfilename << " pos: " <<
            //_output.size()<< std::endl;

            _graph.addDependency(_currentfile, absfilename);

            if( _graph.hasCycleDependency() ){
                RW_THROW(
                    "ERROR: including " << absfilename
                    << " from file " << _currentfile
                    << " because of cyclic dependencies!!!");
            } else {
                file_position pos(absfilename);
                file_position oldpos = _filemap.back().second;
                std::pair<size_t,file_position> mapdata(
                    _output.size(), pos );
                _filemap.push_back( mapdata );
                XMLRWPreParser::parse(absfilename, _output, _filemap, _graph);

                //std::cout << "Last char in output: " <<
                //(int(_output[_output.size()-1]))<< std::endl ;

                //std::cout << "Parsed include succes" << std::endl;
                // remember to push the old back again
                mapdata.first = _output.size();
                mapdata.second = oldpos;
                _filemap.push_back(mapdata);

            }
        }

        std::vector<char> &_output;
        const std::string &_currentfile;
        DependencyGraph &_graph;
        std::vector< std::pair<size_t,file_position> > &_filemap;
    };

    struct XMLPreParser: grammar<
        XMLPreParser, result_closure<std::vector<char> >::context_t >
    {
    protected:
        DependencyGraph &_graph;
        std::string _filename;
        std::vector<char> &_output;
        std::vector< std::pair<size_t,file_position> > &_filemap;

    public:
        XMLPreParser(
            std::string filename,
            DependencyGraph &graph,
            std::vector<char> &output,
            std::vector< std::pair<size_t,file_position> > &filemap)
            :
            _graph(graph),
            _filename(filename),
            _output(output),
            _filemap(filemap)
        {}

        template <typename ScannerT>
        struct definition {
        public:
            definition(XMLPreParser const &self);
             /**
             * @brief Gets start rule
             * @return the start rule
             */
            rule<ScannerT> const start() const { return rwpre; }

            std::string _id, _define;
            std::map<std::string, std::string> _map;
        private:
            rule<ScannerT > rwpre, build, define, use, include, main;
            rule<ScannerT, result_closure<std::string >::context_t> quotedStr;
            rule<ScannerT, result_closure<std::string >::context_t> attrstr_r;

        };
    };

    template <typename ScannerT>
    XMLPreParser::definition<ScannerT>::definition(XMLPreParser const &self)
    {
        /* The  */
        rwpre = XMLErrorHandler::XMLErrorGuard( main )[XMLErrorHandler()] ;

        main =
            (*(define | use | include | build ))
                [ self.result_ = var( self._output) ] >> end_p;

        build =  (+( anychar_p - ( ch_p('<') >>
                                            (   str_p("Define")
                                              | str_p("Use")
                                              | str_p("Include")
                                            ))
                                 ))[ AppendToOutput( self._output ) ]
            ;

        /* Saves this information in a map */
        define =
            XMLAttElem_p("Define",
                XMLAtt_p("id",attrstr_r[ var( _id ) = arg1 ]),
                (*( anychar_p - ( str_p("</") >> "Define" ) ))
                    [ InsertInMap( _map , _id ) ]
            );

        /* Inserts the previusly defined at this position */
        use =
            XMLAttElem_p("Use",
                XMLAtt_p("id",attrstr_r
                    [ AppendToOutputFromMap( _map, self._output ) ]),
                eps_p
            );

        /* Inserts text from include files */
        include =
            XMLAttElem_p("Include",
                XMLAtt_p("file",attrstr_r
                    [ AppendToOutputFromFile( self._output, self._filename, self._graph, self._filemap) ]),
                eps_p
            );

        attrstr_r = (*(anychar_p - '"'))[attrstr_r.result_ = construct_<std::string>(arg1,arg2)];

        quotedStr = '"' >> lexeme_d[ *(anychar_p - '"') ][quotedStr.result_ = construct_<std::string>(arg1,arg2)]
                                    >> '"';

    }
}

bool XMLRWPreParser::parse( const std::string& absfilename,
                            std::vector<char> &output,
                            std::vector< std::pair<size_t,file_position> > &filemap,
                            DependencyGraph &graph ){



    std::ifstream in( absfilename.c_str() );
    if (!in) {
        RW_THROW("Could not open input file: " << absfilename );
    }
    /*std::string absfilename = filename;
    if( !StringUtil::IsAbsoluteFileName( filename ) ){
        fs::path full_path;
        full_path = fs::system_complete( fs::path( filename.c_str(), fs::native ) );
        absfilename = full_path.native_file_string();
    }*/
    in.unsetf(std::ios::skipws); //  Turn of white space skipping on the stream

    std::vector<char> vec;
    std::copy(
        std::istream_iterator<char>(in),
        std::istream_iterator<char>(),
        std::back_inserter(vec));

    typedef position_iterator<std::vector<char>::const_iterator > iterator_t;
    iterator_t first(vec.begin(),vec.end());
    iterator_t last;

    XMLPreParser p(absfilename, graph, output, filemap);

    parse_info<iterator_t> info =
        boost::spirit::classic::parse( first, last, p,
            (blank_p | "<!--" >> *(anychar_p - "-->") >> "-->")
        );

    if ( info.full ) {
        return true;
    }
    RW_THROW("Parsing of workcell failed!!");
    return false;
}

#define RULE(name, definition) typeof(definition) name = definition

bool XMLRWPreParser::parse(
        const std::string& absfilename,
        std::vector<char> &output,
        std::vector< std::pair<size_t,file_position> > &filemap)
{/*
    if( !StringUtil::IsAbsoluteFileName( absfilename ) ){
        RW_THROW( "Error the absfilename is not an absolute absfilename: " << absfilename );
    }*/
    file_position pos(absfilename);
    std::pair<size_t,file_position> mapdata( 0 , pos );
    filemap.push_back( mapdata );

    DependencyGraph graph;
    graph.hasCycleDependency();
    bool res = XMLRWPreParser::parse( absfilename, output, filemap, graph );
    //std::cout << "Finished pre parsing step!!" << std::endl;
    return res;
}
