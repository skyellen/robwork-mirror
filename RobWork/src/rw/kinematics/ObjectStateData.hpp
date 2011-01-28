/*
 * ObjectStateData.hpp
 *
 *  Created on: 26/01/2011
 *      Author: jimali
 */

#ifndef OBJECTSTATEDATA_HPP_
#define OBJECTSTATEDATA_HPP_

#include <rw/kinematics/StateData.hpp>
#include <rw/kinematics/State.hpp>

namespace rw {
namespace kinematics {

    /**
     * @brief the object state saves a single integer in the state array which
     * is used as index to a specific implementation of the object state in StatelessObject
     */
    class ObjectStateData: public rw::kinematics::StateData {
    public:
        typedef rw::common::Ptr<ObjectStateData> Ptr;

        ObjectStateData():StateData(1, "")
        {}

        void setObjectID(int id, rw::kinematics::State& state) {
            double* q = getData(state);
            q[0] = (double)id;
        }

        int getObjectID(const rw::kinematics::State& state) const{
            const double* q = getData(state);
            return (int)q[0];
        }

        virtual ObjectStateData::Ptr clone() = 0;

    };

}
}

#endif /* OBJECTSTATEDATA_HPP_ */
