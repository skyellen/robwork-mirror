#ifndef LN_LINE_FIT_HPP
#define LN_LINE_FIT_HPP

#include "LinePolar.hpp"
#include "P2D.hpp"
#include <vector>

namespace LineFit
{
    typedef std::vector<P2D>::const_iterator const_iterator;
    typedef std::pair<const_iterator, const_iterator> const_iterator_pair;

    LinePolar fit(const_iterator_pair range);
    LinePolar fit(const_iterator a, const_iterator b);
    LinePolar fit(const std::vector<P2D>& pnts);
}

#endif
