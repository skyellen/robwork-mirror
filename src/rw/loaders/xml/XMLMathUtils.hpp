#ifndef RW_LOADERS_XERCESXMLMATHUTILS_HPP
#define RW_LOADERS_XERCESXMLMATHUTILS_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Rotation2D.hpp>
#include <rw/math/VelocityScrew6D.hpp>

#include <xercesc/dom/DOMElement.hpp>

#include <map>
#include <string>

namespace rw {
namespace loaders {

/**
 * @brief Utility class to help read in the content of a XML-files parsed with Xerces
 */
class XMLMathUtils
{
private:
    /* Static variable used to make sure Xerces is initialized
     * This variable need to be placed BEFORE the XMLCh* identifiers in order to Xerces to be initialized*/
    static const bool _initialized;
public:
    /** @brief Identifier for rw::math::Q in the XML format  */
    static const XMLCh* QId;

    /** @brief Identifier for rw::math::Vector3D<> in the XML format  */
    static const XMLCh* Vector3DId;

    /** @brief Identifier for rw::math::Vector2D<> in the XML format  */
    static const XMLCh* Vector2DId;

    /** @brief Identifier for rw::math::Rotation3D<> in the XML format  */
    static const XMLCh* Rotation3DId;

    /** @brief Identifier for rw::math::RPY<> in the XML format  */
    static const XMLCh* RPYId;

    /** @brief Identifier for rw::math::EAA<> in the XML format  */
    static const XMLCh* EAAId;

    /** @brief Identifier for rw::math::Quaternion<> in the XML format  */
    static const XMLCh* QuaternionId;

    /** @brief Identifier for rw::math::Rotation2D<> in the XML format  */
    static const XMLCh* Rotation2DId;

    /** @brief Identifier for rw::math::Transform3D<> in the XML format  */
    static const XMLCh* Transform3DId;

    /** @brief Identifier for rw::math::VelocityScrew6D<> in the XML format  */
    static const XMLCh* VelocityScrew6DId;

    /** @brief Identifier for the position specification used in Transform3D  */
    static const XMLCh* PosId;

    /** @brief Identifier for matrix specification used in Transform3D  */
    static const XMLCh* MatrixId;

    /** @brief Identifier for specifying the linear part in a VelocityScrew6D  */
    static const XMLCh* LinearId;

    /** @brief Identifier for specifying the angular part in a VelocityScrew6D  */
    static const XMLCh* AngularId;

    /** @brief Identifier for the unit attribute */
    static const XMLCh* UnitAttributeId;


    /**
     * @brief Returns the conversion value for a given unit
     *
     * The value returned can be used to convert the values specified in the element to
     * the standard units used in RobWork.
     *
     * Throws an exception if \b key cannot be matched to a unit
     *
     * @param key [in] The key for which the get the unit
     * @return Conversion value
     */
    static double getUnit(const XMLCh* key);

    /**
     * @brief Returns rw::math::Q element read from \b element
     *
     * Read in \b element and returns a rw::math::Q corresponding to the content.
     * If \b doCheckHeader = true it checks that the elements tag name matches QId.
     * If the name does not an exception is thrown.
     *
     * @param element [in] Element to read
     * @param doCheckHeader [in] True if the header name should be checked
     * @return The element read
     */
    static rw::math::Q readQ(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Returns rw::math::Vector3D<> element read from \b element
     *
     * Read in \b element and returns a rw::math::Vector3D<> corresponding to the content.
     * If \b doCheckHeader = true it checks that the elements tag name matches Vector3DId.
     * If the name does not an exception is thrown.
     *
     * @param element [in] Element to read
     * @param doCheckHeader [in] True if the header name should be checked
     * @return The element read
     */
    static rw::math::Vector3D<> readVector3D(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Returns rw::math::Vector2D<> element read from \b element
     *
     * Read in \b element and returns a rw::math::Vector2D<> corresponding to the content.
     * If \b doCheckHeader = true it checks that the elements tag name matches Vector3DId.
     * If the name does not an exception is thrown.
     *
     * @param element [in] Element to read
     * @param doCheckHeader [in] True if the header name should be checked
     * @return The element read
     */
    static rw::math::Vector2D<> readVector2D(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Returns rw::math::RPY<> element read from \b element
     *
     * Read in \b element and returns a rw::math::RPY<> corresponding to the content.
     * If \b doCheckHeader = true it checks that the elements tag name matches Vector3DId.
     * If the name does not an exception is thrown.
     *
     * @param element [in] Element to read
     * @param doCheckHeader [in] True if the header name should be checked
     * @return The element read
     */
    static rw::math::RPY<> readRPY(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Returns rw::math::EAA<> element read from \b element
     *
     * Read in \b element and returns a rw::math::EAA<> corresponding to the content.
     * If \b doCheckHeader = true it checks that the elements tag name matches Vector3DId.
     * If the name does not an exception is thrown.
     *
     * @param element [in] Element to read
     * @param doCheckHeader [in] True if the header name should be checked
     * @return The element read
     */
    static rw::math::EAA<> readEAA(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Returns rw::math::Quaternion<> element read from \b element
     *
     * Read in \b element and returns a rw::math::Quaternion<> corresponding to the content.
     * If \b doCheckHeader = true it checks that the elements tag name matches Vector3DId.
     * If the name does not an exception is thrown.
     *
     * @param element [in] Element to read
     * @param doCheckHeader [in] True if the header name should be checked
     * @return The element read
     */
    static rw::math::Quaternion<> readQuaternion(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Returns rw::math::Rotation3D<> element read from \b element
     *
     * Read in \b element and returns a rw::math::Rotation3D<> corresponding to the content.
     * If \b doCheckHeader = true it checks that the elements tag name matches Vector3DId.
     * If the name does not an exception is thrown.
     *
     * @param element [in] Element to read
     * @param doCheckHeader [in] True if the header name should be checked
     * @return The element read
     */
    static rw::math::Rotation3D<> readRotation3D(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Returns rw::math::Rotation2D<> element read from \b element
     *
     * Read in \b element and returns a rw::math::Rotation2D<> corresponding to the content.
     * If \b doCheckHeader = true it checks that the elements tag name matches Vector3DId.
     * If the name does not an exception is thrown.
     *
     * @param element [in] Element to read
     * @param doCheckHeader [in] True if the header name should be checked
     * @return The element read
     */
    static rw::math::Rotation2D<> readRotation2D(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Returns rw::math::Rotation3D<> element read from \b element
     *
     * Read in \b element and returns a rw::math::Rotation3D corresponding to the content.
     * The content can be either a RPY, EAA, Quaternion or Rotation3D.
     * If \b doCheckHeader = true it checks that the elements tag name matches Vector3DId.
     * If the name does not an exception is thrown.
     *
     * @param element [in] Element to read
     * @param doCheckHeader [in] True if the header name should be checked
     * @return The element read
     */
    static rw::math::Rotation3D<> readRotation3DStructure(xercesc::DOMElement* element);

    /**
     * @brief Returns rw::math::Transform3D<> element read from \b element
     *
     * Read in \b element and returns a rw::math::Transform3D<> corresponding to the content.
     * If \b doCheckHeader = true it checks that the elements tag name matches Vector3DId.
     * If the name does not an exception is thrown.
     *
     * @param element [in] Element to read
     * @param doCheckHeader [in] True if the header name should be checked
     * @return The element read
     */
    static rw::math::Transform3D<> readTransform3D(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Returns rw::math::VelocityScrew6D<> element read from \b element
     *
     * Read in \b element and returns a rw::math::VelocityScrew6D<> corresponding to the content.
     * If \b doCheckHeader = true it checks that the elements tag name matches Vector3DId.
     * If the name does not an exception is thrown.
     *
     * @param element [in] Element to read
     * @param doCheckHeader [in] True if the header name should be checked
     * @return The element read
     */
    static rw::math::VelocityScrew6D<> readVelocityScrew6D(xercesc::DOMElement* element, bool doCheckHeader = false);



    /**
     * @brief Creates a DOMElement to represent \b q
     *
     * Creates a DOMElement owned by \b doc and representing \b q
     *
     * This method may throw an exception in case of errors
     *
     * @param q [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createQ(const rw::math::Q& q, xercesc::DOMDocument* doc);

    /**
     * @brief Creates a DOMElement to represent \b v
     *
     * Creates a DOMElement owned by \b doc and representing \b v
     *
     * This method may throw an exception in case of errors
     *
     * @param v [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createVector3D(const rw::math::Vector3D<>& v, xercesc::DOMDocument* doc);

    /**
     * @brief Creates a DOMElement to represent \b v
     *
     * Creates a DOMElement owned by \b doc and representing \b v
     *
     * This method may throw an exception in case of errors
     *
     * @param v [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createVector2D(const rw::math::Vector2D<>& v, xercesc::DOMDocument* doc);

    /**
     * @brief Creates a DOMElement to represent \b rpy
     *
     * Creates a DOMElement owned by \b doc and representing \b rpy
     *
     * This method may throw an exception in case of errors
     *
     * @param rpy [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createRPY(const rw::math::RPY<>& rpy, xercesc::DOMDocument* doc);

    /**
     * @brief Creates a DOMElement to represent \b eaa
     *
     * Creates a DOMElement owned by \b doc and representing \b eaa
     *
     * This method may throw an exception in case of errors
     *
     * @param eaa [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createEAA(const rw::math::EAA<>& eaa, xercesc::DOMDocument* doc);

    /**
     * @brief Creates a DOMElement to represent \b quat
     *
     * Creates a DOMElement owned by \b doc and representing \b quat
     *
     * This method may throw an exception in case of errors
     *
     * @param quat [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createQuaternion(const rw::math::Quaternion<>& quat, xercesc::DOMDocument* doc);

    /**
     * @brief Creates a DOMElement to represent \b rot
     *
     * Creates a DOMElement owned by \b doc and representing \b rot
     *
     * This method may throw an exception in case of errors
     *
     * @param rot [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createRotation3D(const rw::math::Rotation3D<>& rot, xercesc::DOMDocument* doc);


    /**
     * @brief Creates a DOMElement to represent \b rot
     *
     * Creates a DOMElement owned by \b doc and representing \b rot
     *
     * This method may throw an exception in case of errors
     *
     * @param rot [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createRotation2D(const rw::math::Rotation2D<>& rot, xercesc::DOMDocument* doc);


    /**
     * @brief Creates a DOMElement to represent \b trans
     *
     * Creates a DOMElement owned by \b doc and representing \b trans
     *
     * This method may throw an exception in case of errors
     *
     * @param trans [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createTransform3D(const rw::math::Transform3D<>& trans, xercesc::DOMDocument* doc);

    /**
     * @brief Creates a DOMElement to represent \b vs
     *
     * Creates a DOMElement owned by \b doc and representing \b vs
     *
     * This method may throw an exception in case of errors
     *
     * @param vs [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createVelocityScrew6D(const rw::math::VelocityScrew6D<>& vs, xercesc::DOMDocument* doc);


private:
    XMLMathUtils() {};

    /*
     * Map used for mapping unit identifiers o their corresponding values
     */
    class UnitMap {
    public:
        UnitMap();
        std::map<std::string, double> _map;
    };

    static const UnitMap _Units;
};

} //end namespace loaders
} //end namespace rw

#endif //end include guard
