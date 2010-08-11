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


#include "TrajectoryLoader.hpp"

#include <boost/spirit/phoenix.hpp>
#include <boost/spirit.hpp>
#include <boost/spirit/core.hpp>
#include <boost/spirit/symbols/symbols.hpp>
#include <boost/spirit/error_handling/exceptions.hpp>
#include <boost/spirit/iterator/position_iterator.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/RPY.hpp>

#include <rw/common/Property.hpp>
#include <rw/common/StringUtil.hpp>

#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>
#include <rw/common/macros.hpp>

#include <rw/loaders/rwxml/XMLParser.hpp>
#include <rw/loaders/rwxml/XMLErrorHandler.hpp>

#include <rw/proximity/CollisionSetup.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <cassert>

using namespace rw;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::proximity;

using namespace boost::spirit;
using namespace phoenix;

namespace
{
    template < typename ResultT >
    struct result_closure: public boost::spirit::closure<result_closure<ResultT>, ResultT>
    {
        typedef boost::spirit::closure<result_closure<ResultT>, ResultT> base_t;
        typename base_t::member1 result_;
    };

    struct PushBackCollisionPair {
        PushBackCollisionPair(
            const std::string &prefix,
            const std::string &first,
            const std::string &second,
            rw::proximity::ProximityPairList& pairlist)
            :
            _prefix(prefix),_first(first),_second(second),_pairs(pairlist)
        {}

        template < typename IteratorT >
        void operator()(IteratorT const& first, IteratorT const& last) const {
            rw::proximity::ProximityPair pair(_prefix+_first, _prefix+_second);
            _pairs.push_back(pair);
        }
        const std::string &_prefix,&_first, &_second;
        rw::proximity::ProximityPairList& _pairs;
    };

    struct XMLTrajectoryParser:
        grammar<
        XMLTrajectoryParser,
        result_closure<rw::interpolator::Trajectory>::context_t>
    {
    public:

        XMLTrajectoryParser(){}

        template <typename ScannerT>
        struct definition {

        private:
            stored_rule<ScannerT,result_closure<Interpolator>::context_t> interpolator_r;
            stored_rule<ScannerT> interpolatorbody_r;

            rule<ScannerT, > interpolator_r;

            int dof;
        public:

            definition(XMLTrajectoryParser const &self)
            {
                guardwrap_r =
                    XMLErrorHandler::XMLErrorGuard( trajectory_r )[XMLErrorHandler()];

                trajectory_r =
                    XMLAttElem_p("Trajectory",
                                 XMLAtt_p("type", trajectorytype_r),
                                 *lazy_p(interpolator_r)
                    )
                    ;

                trajectorytype_r =
                      str_p("Cartesian")[ var(interpolator_r) = cartinterpolator_r ]
                    | str_p("Joint")[ var(interpolator_r) = jointinterpolator_r ]
                    ;

                cartinterpolator_r =
                    XMLAttElem_p("Interpolator",
                                 XMLAtt_p("type", cartinterpolatortype_r),

                                 )

                cartinterpolatortype_r =
                      str_p("StraightLine")[ var(interpolatorbody_r) =
                                             cartstraightbody_r ]
                    | str_p("CubicSpline")[ var(interpolatorbody_r) = cartcubicbody_r ]


                cartstraightbody_r =


                jointviapoint_r =
                    XMLAttElem("Via", XMLAtt("time",real_p),

                    )
                    ;

                cartviapoint_r =
                    XMLAttElem("Via", XMLAtt("time",real_p),

                    )
                    ;


                exclude_r =
                    XMLElem_p( "Exclude", *framepair_r )
//                        [std::cout << construct_<std::string>("Exclude") << std::endl]

                    ;

                framepair_r =
                    XMLAttElem_p( "FramePair",  framepair_attr_r, eps_p )
                        [ PushBackCollisionPair( self._prefix, first, second, pairs ) ]
//                        [std::cout << construct_<std::string>(arg1,arg2) << std::endl]
                    ;

                framepair_attr_r =
                    XMLAtt_p("first", attstr_r
                        [var(first) = construct_<std::string>(arg1,arg2)] ) >>
                    XMLAtt_p("second", attstr_r
                        [var(second) = construct_<std::string>(arg1,arg2)] )
                    ;

                attstr_r =  *(anychar_p - '"');

            }

            boost::spirit::rule<ScannerT> const start() const
            {
                return guardwrap_r;
            }

        private:
            boost::spirit::rule<ScannerT >
                colsetup_r, exclude_r, framepair_r, framepair_attr_r, attstr_r,guardwrap_r;


        };
    };
}

rw::proximity::CollisionSetup CollisionSetupLoader::load(
    const std::string& prefix,
    const std::string& file)
{
    //CollisionSetup::PairList exclude;

    std::vector<char> input;
    IOUtil::ReadFile(file, input);

    typedef position_iterator<std::vector<char>::const_iterator > iterator_t;
    iterator_t first(input.begin(),input.end());
    iterator_t last;

    rw::proximity::ProximityPairList pairs;
    XMLColSetupParser p(prefix);

    boost::spirit::parse_info<iterator_t> info =
        boost::spirit::parse( first, last, p[ var(pairs) = arg1],
            (space_p | "<!--" >> *(anychar_p - "-->") >> "-->")
        );

    if( !info.hit ){
        RW_THROW("Error parsing file: "<< file);
    }
    CollisionSetup colsetup(pairs);
    return colsetup;
}
