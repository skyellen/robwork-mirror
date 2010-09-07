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
         * @brief constructor
         */
        Contour2DGraspGen(
            const rw::models::TreeDevice& hand,
            const rw::kinematics::State& state,
            int thetaRes=100, bool counterclock=true);

        //! @brief destructor
        virtual ~CG3Grasp2DGen() {};

        /**
         *
         */
        void init(
            const Contour2D& contour,
            int psiRes, int phiRes);

        /**
         * @brief computes a list of the best grasps in the grasp candidate list.
         */
        std::vector<Grasp2D>
            computeGrasps(const QualityMeasure2D& measure, double minQuality, const unsigned int nrOfGrasps);

        /**
         * @brief gets grasp candidate at idx
         */
        Grasp2D getGrasp(int idx);

        /**
         * @brief sets the maximum allowed distance that the projection of thumb
         * contact onto the vector between the other two fingers can deviate from
         * the center of the vector. [0;0.5]
         */
        void setUniformFilter(double acceptUniform){
            _acceptUniform = acceptUniform;
        }

        double getUniformFilter(){
            return _acceptUniform;
        }


        void setMaxCurvature(double curvThres){
            _sqrCurvThres = curvThres*curvThres;
        }
        double getMaxCurvature() const {
            return sqrt(_sqrCurvThres);
        }

        /**
         * @brief sets the maximum allowed angle
         */
        void setPerpFilter(double acceptPerp){
            _acceptPerp = acceptPerp;
        }
        double getPerpFilter() const{
            return _acceptPerp;
        }

        /**
         * @brief set the maximum allowed angle between thumb approach and the vector between
         * the two other fingers. [0;Pi/2]
         */
        void setDirFilter(double direction){
            _acceptDirs = cos(direction);
        }
        double getDirFilter() const{
            return _acceptDirs;
        }

        /**
         * @brief sets the maximum allowed distance between any fingers
         * in meters.
         */
        void setMaxGraspWidth(double maxWidth){
            _maxGraspWidth = maxWidth;
        }
        double getMaxGraspWidth() const{
            return _maxGraspWidth;
        }

        const Contour2D& getContour(){
            return _infoMap.getContour();
        }

    private:
        const rw::models::TreeDevice& _hand;
        rw::kinematics::State _state;
        bool _counterClock;
        int _thetaRes,_psiRes,_phiRes;
        double _thetaStep,_psiStep,_phiStep;
        double  _sqrCurvThres;
        double _acceptUniform;
        double _acceptPerp;
        double _acceptDirs;
        double _maxGraspWidth;

        double _L, _sqrL;
        double _h, _w;

        // params for filter rules


        rw::math::Vector2D<> _center;
        rw::math::Vector2D<> _f1Dir,_f2Dir,_thumDir;

        Contour2DInfoMap _infoMap;

        std::vector<Grasp2D> _graspCandidates;
    };

}
}

#endif /*CG3Grasp2DGen_HPP_*/
