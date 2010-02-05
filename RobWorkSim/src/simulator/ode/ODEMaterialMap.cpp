
#include "ODEMaterialMap.hpp"

#include "ODEBody.hpp"

#include <dynamics/MaterialDataMap.hpp>
#include <dynamics/ContactDataMap.hpp>

#include <ode/ode.h>
#include <vector>

ODEMaterialMap::ODEMaterialMap(dynamics::MaterialDataMap& map,
                               dynamics::ContactDataMap& cmap, std::vector<
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

    const dynamics::FrictionData& data = _map.getFrictionData(mid1, mid2);

    double restitutionThres = 0.0001;
    double cfm = 0.0001;
    double erp = 0.3;

    const ContactDataMap::NewtonData& cdata = _cmap.getNewtonData(cid1, cid2);

    con.surface.mode = dContactBounce | dContactSoftCFM | dContactSoftERP
            | dContactApprox1;

    con.surface.bounce = cdata.cr;
    con.surface.bounce_vel = restitutionThres;

    //if(data.type == dynamics::Coulomb){
    con.surface.mu = data.parameters[0].second(0);
    con.surface.soft_cfm = cfm;
    con.surface.soft_erp = erp;
    //}

}
