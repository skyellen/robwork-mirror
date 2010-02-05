/*
 * ODEMaterialMap.hpp
 *
 *  Created on: 14-05-2009
 *      Author: jimali
 */

#ifndef ODEMATERIALMAP_HPP_
#define ODEMATERIALMAP_HPP_

#include <dynamics/MaterialDataMap.hpp>
#include <dynamics/ContactDataMap.hpp>
#include <ode/ode.h>
#include <vector>
#include "ODEBody.hpp"

class ODEMaterialMap {
public:
    /**
     * @brief Constructor
     * @param map
     * @param cmap
     * @param odeBodies
     * @return
     */
	ODEMaterialMap(dynamics::MaterialDataMap& map,
				   dynamics::ContactDataMap& cmap,
				   std::vector<ODEBody*> odeBodies);

	/**
	 * @brief copies contact properties between body \b b1 and body \b b2 into
	 * the dContact.
	 * @param con
	 * @param b1
	 * @param b2
	 */
	void setContactProperties(dContact &con, ODEBody *b1, ODEBody *b2);

private:
	dynamics::MaterialDataMap &_map;
	dynamics::ContactDataMap &_cmap;

	std::vector<float> _muMap;
	std::vector<float> _bounceMap;
	std::vector<float> _bounceVelMap;
	std::vector<float> _cfmMap;
	std::vector<float> _erpMap;
};


#endif /* ODEMATERIALMAP_HPP_ */
