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

#ifndef RW_KINEMATICS_STATELESSDATA_HPP_
#define RW_KINEMATICS_STATELESSDATA_HPP_

#include <rw/common/StringUtil.hpp>

#include "StateData.hpp"
#include "StateCache.hpp"
#include "StateStructure.hpp"

namespace rw {
namespace kinematics {

    /**
     * @brief class for enabling statelessness in classes that are data containers
     */
	template<class DATA>
    class StatelessData {
    public:
    	/**
    	 * @brief constructor
    	 * @param dN [in] the number of elements of type DATA that should be allocated in the state.
    	 */
    	StatelessData(int dN=1):
            _N(dN)
        {
            _sdata = boost::shared_ptr<StateData>(new StateData((sizeof(DATA)*dN)/sizeof(double)+1, rw::common::StringUtil::ranName("sdata")));
        }

    	/**
    	 * @copydoc StatelessData(int)
    	 * @param cache [in] data cache.
    	 */
    	StatelessData(int dN, rw::common::Ptr<StateCache> cache):
            _N(dN)
        {
            _sdata = boost::shared_ptr<StateData>(new StateData((sizeof(DATA)*dN)/sizeof(double)+1, rw::common::StringUtil::ranName("sdata"), cache));
        }

    	//! destructor
    	virtual ~StatelessData(){
    	}

    	/**
    	 * @brief initialize this stateless data to a specific state
    	 * @param state [in] the state in which to register the data.
    	 *
    	 * @note the data will be registered in the state structure of the \b state
    	 * and any copies or other instances of the \b state will therefore also
    	 * contain the added states.
    	 */
    	void init(State& state){
    		state.getStateStructure()->addData( _sdata );
    	}


    	/**
    	 * @brief get the data from the \b state
    	 * @param state [in] the state in which the data is saved
    	 * @return reference to data
    	 */
        DATA* getArray(const rw::kinematics::State& state){
            return (DATA*)_sdata->getData(state);
        }

    	/**
    	 * @brief get the data from the \b state
    	 * @param state [in] the state in which the data is saved
    	 * @return reference to data
    	 */
        DATA& get(const rw::kinematics::State& state){
            return *((DATA*)_sdata->getData(state));
        }

    	/**
    	 * @brief get the data from the \b state
    	 * @param state [in] the state in which the data is saved
    	 * @return reference to data
    	 */
        const DATA& get(const rw::kinematics::State& state) const {
            return ((DATA*)_sdata->getData(state))[0];
        }

    	/**
    	 * @brief get the data from the \b state
    	 * @param i [in] the index of the data.
    	 * @param state [in] the state in which the data is saved
    	 * @return reference to data
    	 */
        DATA& get(int i, const rw::kinematics::State& state){
        	RW_ASSERT(i>=0);
        	RW_ASSERT(i<_N);
        	return ((DATA*)_sdata->getData(state))[i];
        }

    	/**
    	 * @brief get the data from the \b state
    	 * @param i [in] the index of the data.
    	 * @param state [in] the state in which the data is saved
    	 * @return reference to data
    	 */
        const DATA& get(int i, const rw::kinematics::State& state) const {
            return ((DATA*)_sdata->getData(state))[i];
        }

        /**
         * @brief set data element in state
         * @param data [in] data to copy into state
         * @param state [in] the state in which to change data
         */
        void set(const DATA& data, rw::kinematics::State& state) {
            ((DATA*)_sdata->getData(state))[0] = data;
        }

        /**
         * @brief set data element in state
         * @param data [in] data to copy into state
    	 * @param i [in] the index of the data.
         * @param state [in] the state in which to change data
         */
        void set(const DATA& data, int i, rw::kinematics::State& state) {
            ((DATA*)_sdata->getData(state))[i] = data;
        }

        /**
         * @brief number of array elements
         * @return number of elements in array
         */
        int getN() const {return _N;}

        /**
         * @brief get the cache of this statedata object. If it has no cache then
         * the returned pointer will be NULL.
         * @param state [in] state in which to get cache from.
         * @return
         */
        template<class CACHE_TYPE>
        CACHE_TYPE* getStateCache(rw::kinematics::State& state) const {
        	return static_cast<CACHE_TYPE*>(_sdata->getCache(state).get());
        }

        //! @copydoc getStateCache(rw::kinematics::State&) const
        template<class CACHE_TYPE>
        CACHE_TYPE* getStateCache(const rw::kinematics::State& state) const {
        	return static_cast<CACHE_TYPE*>(_sdata->getCache(state).get());
        }

        /**
         * @brief Get the state data.
         * @return state data.
         */
        boost::shared_ptr<StateData> getStateData(){return _sdata;}


    private:
        int _N;
        boost::shared_ptr<StateData> _sdata;
    };

}
}
#endif /* STATELESSOBJECT_HPP_ */
