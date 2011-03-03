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

#include "ODEMaterialMap.hpp"

#include "ODEBody.hpp"

#include <rwsim/dynamics/MaterialDataMap.hpp>
#include <rwsim/dynamics/ContactDataMap.hpp>

#include <ode/ode.h>
#include <vector>

using namespace rwsim::dynamics;
using namespace rwsim::simulator;

ODEMaterialMap::ODEMaterialMap(MaterialDataMap& map,
                               ContactDataMap& cmap, std::vector<
                                       ODEBody*> odeBodies) :
    _map(map), _cmap(cmap)
{
    int nrMaterials = map.getMaxMatID();
    _muMap.resize(nrMaterials, 0.4);
    _bounceMap.resize(nrMaterials, 0.05);
    _bounceVelMap.resize(nrMaterials, 0.0001);
    _cfmMap.resize(nrMaterials, 0.001);
    _erpMap.resize(nrMaterials, 0.2);

    /*for(int i=0;i<odeBodies.size();i++){
     int id = odeBodies[i]->getMaterialID();
     // initialize values that belong too this material
     }
     */
}

void ODEMaterialMap::setContactProperties(dContact &con, ODEBody *b1,
                                          ODEBody *b2)
{
    using namespace dynamics;
    int mid1 = b1->getMaterialID();
    int mid2 = b2->getMaterialID();
    int cid1 = b1->getContactID();
    int cid2 = b2->getContactID();

    const FrictionData& data = _map.getFrictionData(mid1, mid2);

    double restitutionThres = 0.00001;
    double cfm = 0.000001;
    double erp = 0.2;

    const ContactDataMap::NewtonData& cdata = _cmap.getNewtonData(cid1, cid2);

//    con.surface.mode = dContactBounce ;

    con.surface.mode =
            dContactBounce
            | dContactSoftCFM
            | dContactSoftERP
            | dContactApprox1;

    con.surface.bounce = cdata.cr;
    con.surface.bounce_vel = restitutionThres;

    //if(data.type == Coulomb){
    con.surface.mu = data.parameters[0].second(0);
    con.surface.soft_cfm = cfm;
    con.surface.soft_erp = erp;
    //}

}
