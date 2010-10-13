#ifndef PLANNING_CONTOUR2DGRASPGEN_HPP_
#define PLANNING_CONTOUR2DGRASPGEN_HPP_

#include "Contour2DInfoMap.hpp"
#include <rw/math/Vector2D.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/kinematics/State.hpp>
#include "QualityMeasure2D.hpp"
#include "Grasp2D.hpp"

namespace rw {
namespace graspplanning {

    /**
     * @brief generates N good contact points on the 2D contour
     */
    class Contour2DGraspGen {
    public:

        /**
         * @brief
         */
        virtual void reset( const rw::geometry::Contour2D& contour ) = 0;

        /**
         * @brief computes a list of the best grasps in the grasp candidate list.
         */
        virtual std::vector<Grasp2D>
            computeGrasps(const QualityMeasure2D& measure, double minQuality, const unsigned int nrOfGrasps) = 0;

        /**
         * @brief gets grasp candidate at idx
         */
        virtual Grasp2D getGrasp(int idx) = 0 ;
    };

}
}

#endif /*CG3Grasp2DGen_HPP_*/
