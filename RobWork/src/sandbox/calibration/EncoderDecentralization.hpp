#ifndef EncoderDecentralization_HPP_
#define EncoderDecentralization_HPP_



#include <rw/math/Q.hpp>
#include <rw/trajectory/Path.hpp>


namespace rw { namespace models {

/** @addtogroup models */
    /*@{*/

	/**
     * @brief Functions to compensate for encoder decentralization errors.
     * @author Daniel Schlichting Kirkegaard and Rune Søe-Knudsen
     *
     *
     * The basic formula are developed by Daniel Schlichting Kirkegaard and Rune Søe-Knudsen in theirs master thesis about Automated Robot Calibration.
     *
     *	@f$\theta = \phi - \left(Sin\left(\phi\right)\cdot \tau+Cos\left(\phi\right)\cdot \sigma \right) @f$ where @f$ \theta @f$ is the real angle, @f$ \phi @f$ is the measured encoder angle and @f$ \tau @f$ and @f$ \sigma @f$ is decentralization parameters.
     */

class EncoderDecentralization {

	public:
		/**
		 * @brief Calculate the real angle from a measured angle with known decentralization parameters .
		 * @param phi Measured angle, @f$\phi@f$.
		 * @param tau Decentralization parameter @f$\tau@f$.
		 * @param sigma Decentralization parameter @f$\sigma@f$.
		 * @return The real angle, @f$\theta@f$.
		 */
		static double calcRealAngle(const double phi, const double tau, const double sigma);

		/**
		 * @copydoc calcRealAngle
		 */
		static rw::math::Q calcRealAngle(const rw::math::Q &phi, const rw::math::Q &tau, const rw::math::Q &sigma);

		/**
		 * @copydoc calcRealAngle
		 */
		static rw::trajectory::QPath calcRealAngle(const rw::trajectory::QPath &phi, const rw::math::Q &tau, const rw::math::Q &sigma);

		/**
		 * @brief Calculate the encoder angle from a real angle with known decentralization parameters.
		 * @param theta Real angle, @f$\theta@f$.
		 * @param tau Decentralization parameter @f$\tau@f$.
		 * @param sigma Decentralization parameter @f$\sigma@f$.
		 * @param maxError Define the maximal tolerated error of the @f$\phi@f$ angle.
		 * @param maxIterations Define maximal iteration that are allowed.
		 * @return The encoder angle, @f$\phi@f$.
		 *
		 * The iteration loop terminates then the error are less that the maxError or the maxIterations counter expire.
		 */
		static double calcEncoderAngle(const double theta, const double tau, const double sigma, const double maxError = 1e-10, const unsigned int maxIterations=10);

		/**
		 * @copydoc calcEncoderAngle
		 */
		static rw::math::Q calcEncoderAngle(const rw::math::Q &theta, const rw::math::Q &tau, const rw::math::Q &sigma, const double maxError = 1e-10, const unsigned int maxIterations=10);

		/**
		 * @copydoc calcEncoderAngle
		 */
		static rw::trajectory::QPath calcEncoderAngle(const rw::trajectory::QPath &theta, const rw::math::Q &tau, const rw::math::Q &sigma, const double maxError = 1e-10, const unsigned int maxIterations=10);

	private:

};
/*@}*/
} }
#endif /* EncoderDecentralization_HPP_ */
