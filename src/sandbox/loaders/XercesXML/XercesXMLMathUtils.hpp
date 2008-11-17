/*
 * XercesXMLMathParser.hpp
 *
 *  Created on: Oct 29, 2008
 *      Author: lpe
 */

#ifndef XERCESXMLMATHUTILS_HPP
#define XERCESXMLMATHUTILS_HPP

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

class XercesXMLMathUtils
{
private:
    static const bool _initialized;
public:
    static const XMLCh* Q_ID;
    static const XMLCh* Vector3D_ID;
    static const XMLCh* Vector2D_ID;

    static const XMLCh* Rotation3D_ID;
    static const XMLCh* RPY_ID;
    static const XMLCh* EAA_ID;
    static const XMLCh* Quaternion_ID;

    static const XMLCh* Rotation2D_ID;
    static const XMLCh* Transform3D_ID;
    static const XMLCh* VelocityScrew6D_ID;

    static const XMLCh* Pos_ID;
    static const XMLCh* Linear_ID;
    static const XMLCh* Angular_ID;

    static const XMLCh* Unit_Attribute_ID;



    static double getUnit(const XMLCh* key);

    static rw::math::Q readQ(xercesc::DOMElement* element, bool doCheckHeader = false);

    static rw::math::Vector3D<> readVector3D(xercesc::DOMElement* element, bool doCheckHeader = false);

    static rw::math::Vector2D<> readVector2D(xercesc::DOMElement* element, bool doCheckHeader = false);

    static rw::math::RPY<> readRPY(xercesc::DOMElement* element, bool doCheckHeader = false);

    static rw::math::EAA<> readEAA(xercesc::DOMElement* element, bool doCheckHeader = false);
    static rw::math::Quaternion<> readQuaternion(xercesc::DOMElement* element, bool doCheckHeader = false);

    static rw::math::Rotation3D<> readRotation3D(xercesc::DOMElement* element, bool doCheckHeader = false);
    static rw::math::Rotation2D<> readRotation2D(xercesc::DOMElement* element, bool doCheckHeader = false);
    static rw::math::Rotation3D<> readRotation3DStructure(xercesc::DOMElement* element);

    static rw::math::Transform3D<> readTransform3D(xercesc::DOMElement* element, bool doCheckHeader = false);

    static rw::math::VelocityScrew6D<> readVelocityScrew6D(xercesc::DOMElement* element, bool doCheckHeader = false);

private:
    XercesXMLMathUtils() {};

    class UnitMap {
    public:
        UnitMap();
        std::map<std::string, double> _map;
    };

    static const UnitMap _Units;
};

#endif /* XERCESXMLMATHPARSER_HPP_ */
