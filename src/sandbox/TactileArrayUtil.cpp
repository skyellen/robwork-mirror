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
        int i_low = i-1,i_upp=i+1;
        if(i_low<0) i_low = 0;
        if(i_upp>=tMatrix.size1()) i_upp = tMatrix.size1()-1;

        int j_low = j-1,j_upp=j+1;
        if(j_low<0) j_low = 0;
        if(j_upp>=tMatrix.size2()) j_upp = tMatrix.size2()-1;

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
    const TactileArray::ValueMatrix& data = arraySensor.getTexelData();
    const TactileArray::VertexMatrix& centers = arraySensor.getCenters();
    const TactileArray::VertexMatrix& normals = arraySensor.getNormals();
    const Transform3D<> fTgeom = arraySensor.getTransform();
    Transform3D<> wTf = Kinematics::worldTframe(arraySensor.getFrame(), state);

    for(size_t i=0;i<data.size1(); i++){
        for(size_t j=0;j<data.size2(); j++){
            std::pair<double,Vector3D<> > avgPoint =
                getWeightAverage(i,j,data,centers);
            Vector2D<> texelSize = arraySensor.getTexelSize(i,j);
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
