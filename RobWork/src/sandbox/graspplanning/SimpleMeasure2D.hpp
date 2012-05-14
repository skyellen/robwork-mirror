#ifndef SIMPLEMEASURE_HPP_
#define SIMPLEMEASURE_HPP_

#include <rw/math/Vector2D.hpp>
#include <rw/math/Constants.hpp>
#include "QualityMeasure2D.hpp"

namespace rw {
namespace graspplanning {


class SimpleMeasure: public QualityMeasure2D {

public:

    /**
     * @brief
     * @param maxDist [in] the maximum distance between any two contacts
     */
    SimpleMeasure(double maxDist, const rw::math::Vector2D<>& objCenter):
        _maxDist(maxDist), _objCenter(objCenter)
    {
    }

    /**
     *
     */
    double computeQuality(const Grasp2D& grasp) const {
        using namespace rw::math;
        const int N = grasp.contacts.size();

        const double closureWeight = 0.3;
        const double approachWeight = 0.3;
        const double curvatureWeight = 0.5;
        const double centerWeight = 0.4;


        // 1. first use sum of dot prod between approach vector and surface vector as measure
        double dotSum(0);
        for(int i=0; i<N; i++){
            rw::math::Vector2D<> v1 = grasp.contacts[i].n;
            rw::math::Vector2D<> v2 = grasp.approach[i];
            double angle = fabs(acos( dot(v1,v2)/ (v1.norm2()*v2.norm2())));
            //std::cout << "Angle: " << angle << std::endl;
            dotSum += rw::math::Pi-angle;
        }
        const double approachAngleQuality = dotSum/(N*rw::math::Pi)*approachWeight;

        // 2. if curvature is flat then give higer quality
        double curvature(0);
        for(int i=0; i<N; i++){
            curvature += fabs(rw::math::Pi-fabs(grasp.contacts[i].avgCurvature));
        }
        const double curvatureQuality = curvature/(N*rw::math::Pi)*curvatureWeight;

        // 3. force closure - if aproach vectors turn toward each other increase quality
        Vector2D<> acenter(0,0);
        for(int i=0; i<N; i++){
            rw::math::Vector2D<> ai = grasp.approach[i];
            acenter += normalize(ai);
        }
        const double forceClosureQuality = (1-(acenter/N).norm2())*closureWeight;

        // 4. give higher quality the grasps that are closer to center
        Vector2D<> graspcenter(0,0);
        for(int i=0; i<N; i++){
            rw::math::Vector2D<> gi = grasp.contacts[i].p;
            graspcenter += gi;
        }
        const double centerQuality = (1/( (graspcenter-_objCenter).norm2()*10+1))*centerWeight;

        // 4. give higer quality the less torque is induced because of contact positions
        //std::cout << "Quality of the grasp " << std::endl
        //          << " - Approach Angle Quality: " << approachAngleQuality << std::endl
        //          << " - Curvature Quality: " << curvatureQuality << std::endl
        //          << " - Force Closure Quality: " << forceClosureQuality << std::endl;

        return (approachAngleQuality+curvatureQuality+forceClosureQuality+centerQuality );
    }
private:
    double _maxDist;
    rw::math::Vector2D<> _objCenter;

};

}
}

#endif /*SIMPLEMEASURE_HPP_*/
