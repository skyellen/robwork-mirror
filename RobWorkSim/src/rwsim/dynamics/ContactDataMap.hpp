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

#ifndef RWSIM_DYNAMICS_CONTACTDATAMAP_HPP_
#define RWSIM_DYNAMICS_CONTACTDATAMAP_HPP_

#include <rw/common/macros.hpp>
#include <iostream>
#include <map>
#include <vector>

namespace rwsim {
namespace dynamics {
	//! @addtogroup dynamics
	//! @{
    /**
     * @brief This class is a storage component for storing collision/contact
     * data information.
     */
    class ContactDataMap
    {
    public:

        struct NewtonData {
            double cr;
        };

        struct ChatterjeeData {
            double crN;
            double crT;
        };


    	ContactDataMap();

    	virtual ~ContactDataMap();

    	/**
    	 * @brief
    	 */
    	double getContactMargin();

        /**
          * @brief add an object type to the contact data map and associates it with
          * a description.
          * @param name [in] the name of the object type
          * @param desc [in] the description of the object type
          */
         void add(const std::string& name, const std::string& desc){
             if( _nameToID.find(name)==_nameToID.end() ){
                 _nameToID[name] = _objectCnt;
                 _objectNames.push_back("");
                 _objectCnt++;
             }

             int cid = getDataID( name );
             _idToDesc[cid] = desc;
             _objectNames[cid] = name;
         }

         /**
           * @brief converts a string of object type name to an int identifier.
           * @param material [in] name of material
           * @return
           */
         int getDataID( const std::string& objType ){
             if(_nameToID.find(objType)==_nameToID.end() ){
                 RW_THROW("Object type \"" << objType << "\" does not exist!");
             }
             return _nameToID[objType];
         }

         const std::string& getObjectTypeName( int id ){
             return _objectNames[id];
         }

         const std::vector<std::string>& getObjecTypes(){
             return _objectNames;
         }

         int getMaxID(){return _objectCnt;};


    	/**
    	 * @brief adds newton data description to the collision between nameA and nameB
    	 */
    	void addNewtonData(const std::string &nameA,
    	                   const std::string &nameB,
    	                   const NewtonData& data);

        /**
         * @brief adds chatterjee data description to the collision between nameA and nameB
         */
        void addChatterjeeData(const std::string &nameA,
                           const std::string &nameB,
                           const ChatterjeeData& data);

        const NewtonData& getNewtonData(const std::string &nameA,
                                        const std::string &nameB);

        const NewtonData& getNewtonData(int idA, int idB);


    	/**
    	 *
    	 */
    	const ChatterjeeData& getChatterjeeData(const std::string &nameA,
    	                                        const std::string nameB);


    private:
        typedef std::pair<int,int> IDPair;
        std::vector<std::string> _objectNames;
        std::map<std::string, int> _nameToID;
        std::map<int, std::string> _idToDesc;
        int _objectCnt;

        typedef std::map<IDPair, NewtonData> NewtonMap;
        typedef std::map<IDPair, ChatterjeeData> ChatterjeeMap;

        NewtonMap _newtonDataMap;
    	ChatterjeeMap _chatterjeeDataMap;

    };
    //! @}

}
}

#endif /*CONTACTDATAMAP_HPP_*/
