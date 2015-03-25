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


#include "TactileArrayUtil.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/kinematics/Kinematics.hpp>
using namespace rw::math;
using namespace rw::sensor;
using namespace rw::kinematics;

namespace {

    /**
     * @brief gets the average value of a 3x3 vertex matrix and the associated
     * tactile values around the coordinate (i,j).
     * @return the max tactile value and the calculatedcontact point
     */
    std::pair<double,Vector3D<> > getWeightAverage(
                          size_t i, size_t j,
                          const TactileArray::ValueMatrix& tMatrix,
                          const TactileArray::VertexMatrix& vMatrix)
    {
        int i_low = (int)i-1,i_upp=(int)i+1;
        if(i_low<0) i_low = 0;
        if(i_upp >= (int)tMatrix.rows()) i_upp = (int)tMatrix.cols()-1;

        int j_low = (int)j-1,j_upp=(int)j+1;
        if(j_low<0) j_low = 0;
        if(j_upp >= (int)tMatrix.rows()) j_upp = (int)tMatrix.cols()-1;

        Vector3D<> wp(0,0,0);
        double valSum = 0, maxVal=0;
        for(int k=i_low;k<=i_upp;k++)
            for(int l=j_low;l<=j_upp;l++){
                wp += vMatrix[k][l]*tMatrix(k,l);
                valSum += tMatrix(k,l);
                maxVal = std::max(tMatrix(k,l), (float)maxVal);
            }
        return std::make_pair(maxVal, wp/valSum);
    }

}

std::vector<Contact3D> TactileArrayUtil::estimateContacts(
    const TactileArray& arraySensor, const State& state, double minContactForce)
{
    // search for point contacts using a 3x3 mask. only save the 3
    std::vector<Contact3D> contacts;
    const TactileArray::ValueMatrix& data = arraySensor.getTexelData(state);
    const TactileArray::VertexMatrix& centers = arraySensor.getCenters();
    const TactileArray::VertexMatrix& normals = arraySensor.getNormals();
    const Transform3D<> fTgeom = arraySensor.getTransform();
    Transform3D<> wTf = Kinematics::worldTframe(arraySensor.getSensorModel()->getFrame(), state);

    for(Eigen::DenseIndex i=0;i<data.rows(); i++){
        for(Eigen::DenseIndex j=0;j<data.cols(); j++){
            std::pair<double,Vector3D<> > avgPoint =
                getWeightAverage(i,j,data,centers);
            Vector2D<> texelSize = arraySensor.getTexelSize((int)i,(int)j);
            double normalForce = avgPoint.first*(texelSize(0)*texelSize(1));
            if( normalForce>minContactForce ){
                Contact3D c;
                c.p = wTf * fTgeom * avgPoint.second;
                // calculate the normal in the contact point
                c.n = wTf * fTgeom * getWeightAverage(i,j,data,normals).second;
                // calculate the curvature around the contact
                //c.curvature =
                c.normalForce = normalForce;

                c.f = c.n * c.normalForce;

                contacts.push_back(c);
            }
        }
    }
    return contacts;
}
