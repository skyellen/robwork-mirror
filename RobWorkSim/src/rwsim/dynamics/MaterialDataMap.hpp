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

#ifndef RWSIM_DYNAMICS_MATERIALDATAMAP_HPP_
#define RWSIM_DYNAMICS_MATERIALDATAMAP_HPP_

#include <rw/common/macros.hpp>
#include <iostream>
#include <map>
#include <rw/math/Q.hpp>
#include <vector>

namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
	//! @{

    typedef enum {Coulomb} FrictionType;

    typedef std::pair<std::string, rw::math::Q> FrictionParam;
    struct FrictionData {
        int type;
        std::vector<FrictionParam> parameters;
    };

    /**
     *
     */
    class MaterialDataMap
    {
    public:

        /**
         * @brief constructor
         */
    	MaterialDataMap();

    	/**
    	 * @brief destructor
    	 */
    	virtual ~MaterialDataMap();

    	/**
    	 * @brief add a material name with a description.
    	 * @param name [in] name of material
    	 * @param desc [in] description of material
    	 */
    	void add(std::string name, std::string desc){
    	    if( _matToMatID.find(name)==_matToMatID.end() ){
    	        _matToMatID[name] = _matCnt;
    	        _mat.push_back("");
    	        _matCnt++;
    	    }

    	    int mat = getDataID( name );
    	    _matToDesc[mat] = desc;
    	    _mat[mat] = name;
    	}

        /**
          * @brief converts a string of material type name to an int identifier.
          * @param material [in] name of material
          * @return
          */
    	int getDataID( const std::string& material ) const {
    		std::map<std::string, int>::const_iterator foundMat = _matToMatID.find(material);
    	    if(foundMat ==_matToMatID.end() ){
    	        RW_THROW("Material \"" << material << "\" does not exist!");
    	    }
    	    return foundMat->second;
    	}

    	const std::string& getMaterialName( int id ) const {
    	    return _mat[id];
    	}

    	const std::vector<std::string>& getMaterials(){
    	    return _mat;
    	}

    	int getMaxMatID() const {return _matCnt;};

    	/**
    	 * @brief Test if the given material pair has friction data in map.
    	 * @param matAID [in] id of first material.
    	 * @param matBID [in] id of second material.
    	 * @param dataType [in] the type of friction data (default is zero).
    	 * @return true if friction data exists - false otherwise.
    	 */
    	bool hasFrictionData(int matAID, int matBID, int dataType=0) const;

    	/**
    	 * @brief Test if the given material pair has friction data in map.
    	 * @param matAID [in] the name of the first material.
    	 * @param matBID [in] the name of the second material.
    	 * @param dataType [in] the type of friction data (default is zero).
    	 * @return true if friction data exists - false otherwise.
    	 */
    	bool hasFrictionData(const std::string& matAID, const std::string& matBID, int dataType=0) const;

        /**
         *
         * @param materialA
         * @param materialB
         * @param dataType [in] default type is 0
         * @return
         */
        const FrictionData&
            getFrictionData(int matAID, int matBID, int dataType=0) const;

        const FrictionData&
            getFrictionData(const std::string& matAID, const std::string& matBID, int dataType=0) const;


    	/**
    	 *
    	 * @param materialA
    	 * @param materialB
    	 * @param data
    	 */
        void addFrictionData(const std::string& materialA,
                             const std::string& materialB,
                             const FrictionData& data);

        /**
         *
         * @param type
         * @return
         */
        const FrictionData& getDefaultFriction(int type) const {
            return _defaultFrictionData;
        }

    private:
        std::map<std::string, int> _strToIntID;

        typedef std::pair<int,int> MatIDPair;
        typedef std::map<MatIDPair, std::vector<FrictionData> > FrictionMap;
        FrictionMap _frictionMap;
        std::vector<std::string> _mat;
        std::map<std::string, int> _matToMatID;
        std::map<int, std::string> _matToDesc;
        FrictionData _defaultFrictionData;
        int _matCnt;
    };
    //! @}
}
}

#endif /*MATERIALDATAMAP_HPP_*/
