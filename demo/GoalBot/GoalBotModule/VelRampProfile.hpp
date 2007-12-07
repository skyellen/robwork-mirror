#ifndef VELRAMPPROFILE_HPP
#define VELRAMPPROFILE_HPP

#include <math/Vector2D.hpp>

class VelRampProfile {
public:
    VelRampProfile(const rw::math::Vector2D<>& poslimits, const rw::math::Vector2D<>& vellimits, const rw::math::Vector2D<>& acclimits);

    ~VelRampProfile();



    /**
     * @brief Returns the velocity needed for moving towards goal, when having the current
     * position \f$pos\f$ and velocity \f$vel\f$.
     * @param goal [in] the goal
     * @param pos [in] the current position
     * @param vel [in] the current velocity
     * @param dt [in] time interval
     */
    double getVelocity(double goal, double pos, double vel, double dt) const;




private:
    rw::math::Vector2D<> _poslimits;
    rw::math::Vector2D<> _vellimits;
    rw::math::Vector2D<> _acclimits;


};


#endif //#ifndef VELRAMPPROFILE_HPP
