
#ifndef BOOST_MPL_AUX_TEST_DATA_HPP_INCLUDED
#define BOOST_MPL_AUX_TEST_DATA_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2002-2004
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Source: /home/ROBWORK/cvsnew/RobWork/ext/boost/mpl/aux_/test/data.hpp,v $
// $Date: 2006-06-09 12:41:34 $
// $Revision: 1.1.1.1 $

#include <boost/noncopyable.hpp>

enum enum_ {};
struct UDT {};
struct incomplete;
class abstract { public: virtual ~abstract() = 0; };
using boost::noncopyable;

#endif // BOOST_MPL_AUX_TEST_DATA_HPP_INCLUDED
