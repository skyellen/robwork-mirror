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

#include "StateData.hpp"
#include "StateSetup.hpp"

using namespace rw::kinematics;

QState::QState():
    _contents(1)
{    
}

QState::QState(boost::shared_ptr<StateSetup> setup) :
    _contents(setup->size()+1),
    _setup(setup)
{
    for (size_t i = 0; i < _contents.size(); i++)
        _contents[i] = 0;
}

const double* QState::getQ(const StateData& data) const
{
    const int pos = _setup->getOffset(data);

    // NB: This is _not_ the same as &_contents[pos] when pos ==
    // _contents.size(). It is OK to return a pointer to one element past the
    // end of the array.
    return &_contents[0] + pos;
}

void QState::setQ(const StateData& data, const double* vals)
{
    const int pos = _setup->getOffset(data);
    const int dof = data.size();
    // See above with regards to the (+ pos) expression.
    memmove(&_contents[0] + pos, vals, dof * sizeof(double));
}

QState& QState::operator=(const QState &rhs) {
    _setup = rhs._setup;
    _contents = rhs._contents;
    return *this;
}
