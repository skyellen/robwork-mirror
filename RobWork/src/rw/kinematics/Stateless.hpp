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

#ifndef RW_KINEMATICS_STATELESS_HPP_
#define RW_KINEMATICS_STATELESS_HPP_

#include "StateStructure.hpp"
#include "StatelessData.hpp"

namespace rw {
namespace kinematics {

	class State;
	class StateData;

    /**
     * @brief interface for a stateless or typically a part stateless class.
     */
    class Stateless {
    protected:

    	/**
    	 * @brief constructor
    	 */
    	Stateless():_registered(false) {}

    public:
    	typedef rw::common::Ptr<Stateless> Ptr;

    	//! destructor
    	virtual ~Stateless(){}

    	/**
    	 * @brief initialize this stateless data to a specific state
    	 * @param state [in] the state in which to register the data.
    	 *
    	 * @note the data will be registered in the state structure of the \b state
    	 * and any copies or other instances of the \b state will therefore also
    	 * contain the added states.
    	 */
    	virtual void registerIn(State& state);

    	//! register this stateless object in a statestructure.
    	virtual void registerIn(StateStructure::Ptr state);

    	//! unregisters all state data of this stateless object
    	virtual void unregister();

    	StateStructure::Ptr getStateStructure(){ return _stateStruct; }

    	const StateStructure::Ptr getStateStructure() const { return _stateStruct; }

    	bool isRegistered(){ return _registered;}
    protected:

    	//! implementations of sensor should add all their stateless data on initialization
    	template<class T>
    	void add( StatelessData<T>& data ){
    		add( data.getStateData() );
    	}

    	void add( StateData* data ){
    		add( boost::shared_ptr<StateData>(data) );
    	}

    	//! implementations of sensor should add all their state data on initialization
    	void add( boost::shared_ptr<StateData> data ){
    		if(_registered)
    			_stateStruct->addData( data );
    		_datas.push_back( data );
    	}

    	bool _registered;
    	std::vector<boost::shared_ptr<StateData> > _datas;
    	StateStructure::Ptr _stateStruct;
    };

}
}
#endif /* STATELESSOBJECT_HPP_ */
