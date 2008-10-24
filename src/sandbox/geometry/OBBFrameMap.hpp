/*********************************************************************
 * RobWork Version 0.3
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
