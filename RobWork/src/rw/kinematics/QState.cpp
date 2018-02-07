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

QState::~QState(){ }

const double* QState::getQ(const StateData& data) const
{
    const int pos = _setup->getOffset(data);
    if (pos < 0)
    	RW_THROW("The values can not be retrieved from this state, as this state does not appear to hold that data. Please make sure you use the correct State object.");

    // NB: This is _not_ the same as &_contents[pos] when pos ==
    // _contents.size(). It is OK to return a pointer to one element past the
    // end of the array.
    return &_contents[0] + pos;
}

double* QState::getQ(const StateData& data)
{
    const int pos = _setup->getOffset(data);
    if (pos < 0)
    	RW_THROW("The values can not be retrieved from this state, as this state does not appear to hold that data. Please make sure you use the correct State object.");

    // NB: This is _not_ the same as &_contents[pos] when pos ==
    // _contents.size(). It is OK to return a pointer to one element past the
    // end of the array.
    return &_contents[0] + pos;
}

void QState::setQ(const StateData& data, const double* vals)
{
    const int pos = _setup->getOffset(data);
    if (pos < 0)
    	RW_THROW("The new values can not be set in this state, as this state does not appear to hold that data. Please make sure you use the correct State object.");
    const int dof = data.size();
    // See above with regards to the (+ pos) expression.
    memmove(&_contents[0] + pos, vals, dof * sizeof(double));
}

QState& QState::operator=(const QState &rhs) {
    _setup = rhs._setup;
    _contents = rhs._contents;
    return *this;
}
