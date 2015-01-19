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

#include <map>
#include <rw/math/Q.hpp>
#include <vector>

namespace rwsim {
namespace dynamics {
//! @addtogroup rwsim_dynamics
//! @{

//! @brief Predefined friction types.
typedef enum {Coulomb, Custom} FrictionType;

//! @brief Definition of a parameter for a friction model.
typedef std::pair<std::string, rw::math::Q> FrictionParam;

//! @brief Definition of a friction model.
struct FrictionData {
	//! @brief The FrictionType
	int type;
	//! @brief The name of the model.
	std::string typeName;
	//! @brief The parameters for the model.
	std::vector<FrictionParam> parameters;
};

/**
 * @brief A map of materials and friction models defined between materials.
 */
class MaterialDataMap
{
public:
	//! @brief Constructor
	MaterialDataMap();

	//! @brief Destructor
	virtual ~MaterialDataMap();

	/**
	 * @brief Add a material name with a description.
	 * @param name [in] name of material.
	 * @param desc [in] description of material.
	 */
	void add(const std::string& name, const std::string& desc);

	/**
	 * @brief Converts a string of material type name to an int identifier.
	 * @param material [in] name of material.
	 * @return the id of the material.
	 */
	int getDataID(const std::string& material) const;

	/**
	 * @brief Convert an id of a material to the name of the material.
	 * @param id [in] the id of the material.
	 * @return the name of the material.
	 */
	const std::string& getMaterialName( int id ) const;

	/**
	 * @brief Get a list of the names of all materials.
	 * @return a list of material names.
	 */
	const std::vector<std::string>& getMaterials();

	/**
	 * @brief Get the number of materials.
	 * @return the maximum id of a material.
	 */
	int getMaxMatID() const;

	/**
	 * @brief Test if the given material pair has friction data in map.
	 * @param matAID [in] id of first material.
	 * @param matBID [in] id of second material.
	 * @param dataType [in] the type of friction data (default is Coulomb).
	 * @return true if friction data exists - false otherwise.
	 */
	bool hasFrictionData(int matAID, int matBID, int dataType = Coulomb) const;

	/**
	 * @brief Test if the given material pair has friction data in map.
	 * @param matAID [in] the name of the first material.
	 * @param matBID [in] the name of the second material.
	 * @param dataType [in] the type of friction data (default is Coulomb).
	 * @return true if friction data exists - false otherwise.
	 */
	bool hasFrictionData(const std::string& matAID, const std::string& matBID, int dataType = Coulomb) const;

	/**
	 * @brief Get a specific friction model for a pair of materials.
	 * @param matAID [in] id of first material.
	 * @param matBID [in] id of second material.
	 * @param dataType [in] the type of friction data (default is Coulomb).
	 * @return the friction data.
	 */
	const FrictionData& getFrictionData(int matAID, int matBID, int dataType = Coulomb) const;

	/**
	 * @brief Get all friction data associated to the given pair of materials.
	 * @param matAID [in] the id of the first material.
	 * @param matBID [in] the id of the second material.
	 * @return a vector of friction data.
	 */
	const std::vector<FrictionData> getFrictionDatas(int matAID, int matBID) const;

	/**
	 * @brief Get a specific friction model for a pair of materials.
	 * @param matAID [in] the name of the first material.
	 * @param matBID [in] the name of the second material.
	 * @param dataType [in] the type of friction data (default is Coulomb).
	 * @return the friction data.
	 */
	const FrictionData&	getFrictionData(const std::string& matAID, const std::string& matBID, int dataType = Coulomb) const;

	/**
	 * @brief Get all friction data associated to the given pair of materials.
	 * @param matAID [in] the name of the first material.
	 * @param matBID [in] the name of the second material.
	 * @return a vector of friction data.
	 */
	const std::vector<FrictionData> getFrictionDatas(const std::string& matAID, const std::string& matBID) const;

	/**
	 * @brief Add friction data for the given pair of materials.
	 * @param materialA [in] the name of the first material.
	 * @param materialB [in] the name of the second material.
	 * @param data [in] the data to add.
	 */
	void addFrictionData(const std::string& materialA,
			const std::string& materialB,
			const FrictionData& data);

	/**
	 * @brief Get the default friction model.
	 * @param type [in] the type of model.
	 * @return the default friction data.
	 */
	const FrictionData& getDefaultFriction(int type) const;

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
