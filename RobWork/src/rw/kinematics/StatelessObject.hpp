/*
 * StatelessObject.hpp
 *
 *  Created on: 26/01/2011
 *      Author: jimali
 */

#ifndef STATELESSOBJECT_HPP_
#define STATELESSOBJECT_HPP_

#include <rw/kinematics/StateStructure.hpp>
#include "ObjectStateData.hpp"

namespace rw {
namespace kinematics {

    class StatelessObject {
    public:

        virtual void addStateData(rw::kinematics::StateStructure::Ptr statestructure){
            for(size_t i=0;i<_objectDatas.size();i++){
                statestructure->addData(_objectDatas[i]);
            }
        }

    protected:
        void addObjectStateData(ObjectStateData* objStateData){
            _objectDatas.push_back(objStateData);
        }

        std::vector<ObjectStateData*> _objectDatas;
    };

}
}
#endif /* STATELESSOBJECT_HPP_ */
