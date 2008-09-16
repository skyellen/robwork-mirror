/*
 * TactileArrayUtil.hpp
 *
 *  Created on: 26-08-2008
 *      Author: jimali
 */

#ifndef TACTILEARRAYUTIL_HPP_
#define TACTILEARRAYUTIL_HPP_

#include <vector>

#include <rw/kinematics/State.hpp>
#include <rw/sensor/Contact3D.hpp>
#include <rw/sensor/TactileArray.hpp>

class TactileArrayUtil {

public:

    static std::vector<Contact3D>
        estimateContacts(const rw::sensor::TactileArray& arraySensor,
                         const rw::kinematics::State& state,
                         double minContactForce);

};


#endif /* TACTILEARRAYUTIL_HPP_ */
