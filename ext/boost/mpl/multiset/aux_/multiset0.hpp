
#ifndef BOOST_MPL_MULTISET_AUX_MULTISET0_HPP_INCLUDED
#define BOOST_MPL_MULTISET_AUX_MULTISET0_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2003-2004
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Source: /home/ROBWORK/cvsnew/RobWork/ext/boost/mpl/multiset/aux_/multiset0.hpp,v $
// $Date: 2006-06-09 12:41:32 $
// $Revision: 1.1.1.1 $

#include <boost/mpl/multiset/aux_/tag.hpp>
#include <boost/mpl/int.hpp>

namespace boost { namespace mpl {

template< int dummy_ = 0 >
struct multiset0
{
    typedef aux::multiset_tag tag;

    typedef int_<1> count_;
    static char (& key_count(...) )[count_::value];
    static char (& ref_key_count(...) )[count_::value];
};

}}

#endif // BOOST_MPL_MULTISET_AUX_MULTISET0_HPP_INCLUDED
