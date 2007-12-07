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

#include "QState.hpp"

using namespace rw::kinematics;

QState::QState():
    _contents(0),
    _setup(new QStateSetup())
{    
}

QState::QState(boost::shared_ptr<QStateSetup> setup) :
    _contents(setup->getDof()),
    _setup(setup)
{
    for (size_t i = 0; i < _contents.size(); i++)
        _contents[i] = 0;
}

const double* QState::getQ(const Frame& frame) const
{
    const int pos = _setup->getOffset(frame);

    // NB: This is _not_ the same as &_contents[pos] when pos ==
    // _contents.size(). It is OK to return a pointer to one element past the
    // end of the array.
    return &_contents[0] + pos;
}

void QState::setQ(const Frame& frame, const double* vals)
{
    const int pos = _setup->getOffset(frame);
    const int dof = frame.getDof();

    // See above with regards to the (+ pos) expression.
    memmove(&_contents[0] + pos, vals, dof * sizeof(double));
}
