
#ifndef BOOST_MPL_AUX_NA_ASSERT_HPP_INCLUDED
#define BOOST_MPL_AUX_NA_ASSERT_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2001-2004
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Source: /home/ROBWORK/cvsnew/RobWork/ext/boost/mpl/aux_/na_assert.hpp,v $
// $Date: 2006-06-09 12:41:34 $
// $Revision: 1.1.1.1 $

#include <boost/mpl/aux_/na.hpp>
#include <boost/mpl/aux_/config/msvc.hpp>
#include <boost/mpl/aux_/config/workaround.hpp>

#if !BOOST_WORKAROUND(_MSC_FULL_VER, <= 140050601)    \
    && !BOOST_WORKAROUND(__EDG_VERSION__, <= 243)
#   include <boost/mpl/assert.hpp>
#   define BOOST_MPL_AUX_ASSERT_NOT_NA(x) \
    BOOST_MPL_ASSERT_NOT((boost::mpl::is_na<type>)) \
/**/
#else
#   include <boost/static_assert.hpp>
#   define BOOST_MPL_AUX_ASSERT_NOT_NA(x) \
    BOOST_STATIC_ASSERT(!boost::mpl::is_na<x>::value) \
/**/
#endif

#endif // BOOST_MPL_AUX_NA_ASSERT_HPP_INCLUDED
