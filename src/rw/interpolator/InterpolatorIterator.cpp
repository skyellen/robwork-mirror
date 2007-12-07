/*********************************************************************
 * RobWork Version 0.2
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
#include "InterpolatorIterator.hpp"

#include <rw/common/macros.hpp>

using namespace rw::interpolator;

InterpolatorIterator::InterpolatorIterator(const std::vector<FunctionSegment*>* segments):
    _endReached(false),
    _beginReached(true),
    _segments(segments),
    _index(0),
    _offset(0)
{
    //LPE: Removed because pathplanning::TrajectoryIterator needs to construct an 
    //     InterpolatorIterator with NULL as argument
    /*    if (!_segments)
        RW_THROW(
            "Null-pointer passed to InterpolatorIterator.\n"
            "Pass a pointer to the empty vector of segments instead.");*/
}

void InterpolatorIterator::operator-=(double val)
{
    *this += -val;
}

void InterpolatorIterator::operator+=(double val)
{
    const double length = getFun().getLength();
    const double pos = _offset + val;

    // Shift to next segment (only if larger than, to avoid problems when position is the very last one)
    // Solves many problems to keep within a segment even though the error
    // is within a small error.
    if (pos > length + 0.0000000001) {

        // If we are moving past the end of the last segment, then lock the
        // position to here:
        if (_index + 1 >= (int)_segments->size()) {
            _offset = length;
            _endReached = true;
        }

        // Otherwise go to the next segment recursively.
        else {
            ++_index;
            _offset = 0;
            *this += pos - length;
        }
    }

    // Shift to previous segment
    else if (pos < -0.0000000001) {

        // If we are moving past the start of the first segment, then lock the
        // position to here:
        if (_index == 0) {
            _offset = 0;
            _beginReached = true;
        }

        // Otherwise continue from the previous segment recursively.
        else {
            --_index;
            _offset = 0;
            *this += pos + getFun().getLength();
        }
    }

    // Otherwise we just move within the segment.
    else {
        _offset = pos;
        _endReached = false;
        _beginReached = false;
    }
}
