#ifndef RW_MODELS_DHPARAMETERSET_HPP
#define RW_MODELS_DHPARAMETERSET_HPP

#include "SerialDevice.hpp"
#include <vector>

namespace rw {
namespace models {

/**
 * @brief Simple class to help represent a set of Denavit-Hartenberg
 * parameters
 */
class DHParameterSet
{
public:
    /**
     * @brief Constructor
     * @param alpha [in] \f$\alpha_{i-1}\f$
     * @param a [in] \f$a_{i-1}\f$
     * @param d [in] \f$d_{i}\f$
     * @param theta [in] \f$\theta_{i-1}\f$
     */
    DHParameterSet(double alpha, double a, double d, double theta) :
        _alpha(alpha),
        _a(a),
        _d(d),
        _theta(theta)
    {}

    /**
     * @brief Constructor
     * @param alpha [in] \f$\alpha_{i-1}\f$
     * @param a [in] \f$a_{i-1}\f$
     * @param d [in] \f$d_{i}\f$
     * @param theta [in] \f$\theta_{i-1}\f$
     *
     */
    DHParameterSet(double alpha, double a, double d, double theta, const std::string& type) :
        _alpha(alpha),
        _a(a),
        _d(d),
        _theta(theta),
        _type(type)
    {}

    /** @brief \f$\alpha_{i-1}\f$ **/
    double alpha() const {
        return _alpha;
    }

    /** @brief \f$a_{i-1}\f$ **/
    double a() const {
        return _a;
    }

    /** @brief \f$d_{i} \f$ **/
    double d() const {
        return _d;
    }

    /** $brief \f$\theta_{i} \f$ **/
    double theta() const {
        return _theta;
    }

    std::string getType() const {
    	return _type;
    }

    /**
     * @brief Returns the DH-Parameters for a SerialDevice. 
     *
     * If no or only a partial DH representation exists only the list will be empty or non-complete.
     *
     * @param device [in] SerialDevice for which to get the DH parameters
     * @return The set of DH parameters
     */
    static std::vector<DHParameterSet> getDHParameters(SerialDevicePtr device);


private:
    /** @brief \f$\alpha_{i-1}\f$ **/
    double _alpha;
    /** @brief \f$a_{i-1}\f$ **/
    double _a;
    /** @brief \f$d_{i} \f$ **/
    double _d;
    /** $brief \f$\theta_{i} \f$ **/
    double _theta;

    std::string _type;
};

} //end namespace models
} //end namespace rw


#endif //#ifndef RW_MODELS_DHPARAMETERSET_HPP
