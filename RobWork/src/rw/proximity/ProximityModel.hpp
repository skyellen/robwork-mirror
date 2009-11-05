/*
 * ProximityModel.hpp
 *
 *  Created on: 24-09-2009
 *      Author: jimali
 */

#ifndef PROXIMITYMODEL_HPP_
#define PROXIMITYMODEL_HPP_

#include <rw/common/Ptr.hpp>

namespace rw {
namespace proximity {

    class ProximityStrategy;

    class ProximityModel {
    public:
        ProximityModel(ProximityStrategy *owner_tmp):
            owner(owner_tmp)
        {}

        virtual ~ProximityModel();

        //void addModel(const CollisionModelInfo& model){
        //    _collisionModels.push_back();
        //}

        //std::vector<CollisionModelInfo> _collisionModels;
        ProximityStrategy *owner;
    };

    typedef rw::common::Ptr<ProximityModel> ProximityModelPtr;

}
}

#endif /* PROXIMITYMODEL_HPP_ */
