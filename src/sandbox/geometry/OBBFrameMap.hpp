/*
 * OBBFrameMap.hpp
 *
 *  Created on: 15-10-2008
 *      Author: jimali
 */

#ifndef OBBFRAMEMAP_HPP_
#define OBBFRAMEMAP_HPP_

#include "OBB.hpp"

class OBBFrameMap {
public:

    OBBFrameMap(){};


    void addOBB(Frame *frame, const std::vector<Geometry*>& geoms);

    const OBB& getOBB(Frame *frame);

};


#endif /* OBBFRAMEMAP_HPP_ */
