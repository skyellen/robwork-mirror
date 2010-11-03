#ifndef RW_GRASPPLANNING_CONTOUR2DUTIL_HPP_
#define RW_GRASPPLANNING_CONTOUR2DUTIL_HPP_

#include "Contour2D.hpp"
#include <rw/math/Vector2D.hpp>
#include <rw/math/Rotation2D.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/optional.hpp>

namespace rw {
namespace geometry {

/**
 * @brief utility functions for Contour2D objects
 */
class Contour2DUtil {

public:
    //! covariance matrix type
    typedef boost::numeric::ublas::bounded_matrix<double,2,2> CovarMatrix2D;

    /**
     * @brief calculates the r'th sequence moment of an ordered contour
     * @param contour [in]
     * @param c [in] center
     * @param r [in]
     */
    static double calcSequenceMoment(
        const Contour2D& contour, const rw::math::Vector2D<>& c, const int r);

    /**
     * @brief calculates the r'th central moments of the contour
     * @param contour
     * @param c [in] center
     * @param r [in]
     * @return
     */
    static double calcCentralMoments(
        const Contour2D& contour, const rw::math::Vector2D<>& c, const int r);

    /**
     * @brief Calculates the covariance of the contour "contour" with mean coordinate
     * c
     * @param contour [in] the contour
     * @param c [in] the mean of the contour points/coordinates
     * @return the 2x2 covariance matrix of the contour
     */
    static CovarMatrix2D calcCovarianceMatrix(
        const Contour2D& contour, const rw::math::Vector2D<>& c);

    /**
     * @brief calculates the orientation of a contour where the first
     * axis is the longest principal axis posible.
     */
    static rw::math::Rotation2D<> calcOrientation(
        const Contour2D& contour, const rw::math::Vector2D<>& c);

    /**
     * @brief calculates the centroid or the mean of the points in the contour.
     */
    static rw::math::Vector2D<> calcCentroid(const Contour2D& contour);

    /**
     * @brief calculates a tight fitting 2d bounding box for the contour.
     *
     */
   /* OBB2D calcOBB(const contour2D& contour){
        rw::math::Vector2D<> c = calcCentroid(contour);
        Rotation2D<> orin = calcOrientation(contour,c);
    }*/

    /**
     * @brief extracts the local curvature around the contour point
     * defined by idx.
     */
    static double getCurvature(int idx, int pixelStep, const Contour2D& contour);

    /**
     * @brief calculate normals of a contour
     * @param idx
     * @param pixelStep
     * @param contour
     * @param counterClock
     * @return
     */
    static rw::math::Vector2D<> calcNormal(int idx, int pixelStep, const Contour2D& contour, bool counterClock=true);

    /**
     * @brief recalculate normals of a contour
     * @param contour
     */
    static void recalcNormal(Contour2D& contour);

    /**
     * @brief extracts the outer contour of a contour
     * @param contour
     * @return new contour
     */
	static Contour2D::Ptr getOuterContour(const Contour2D& contour, double resolution);
};

}
}

#endif /*CONTOUR2DUTIL_HPP_*/
