/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

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

#include <rw/kinematics/State.hpp>

#include <xercesc/util/XercesDefs.hpp>

#include <string>

XERCES_CPP_NAMESPACE_BEGIN
class DOMDocument;
class DOMElement;
XERCES_CPP_NAMESPACE_END

namespace rw { namespace models { class WorkCell; } }

namespace rw {
namespace loaders {

    /** @addtogroup loaders */
    /*@{*/


/**
 * @brief Utility class to help read in the content of a XML-files parsed with Xerces
 */
class XMLBasisTypes
{
public:
	/**
	 * @brief Identifier for rw::math::Q in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idQ();
	/**
	 * @brief Identifier for rw::math::Vector3D<> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idVector3D();
	/**
	 * @brief Identifier for rw::math::Vector2D<> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idVector2D();
	/**
	 * @brief Identifier for rw::math::Rotation3D<> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idRotation3D();
	/**
	 * @brief Identifier for rw::math::RPY<> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idRPY();
	/**
	 * @brief Identifier for rw::math::EAA<> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idEAA();
	/**
	 * @brief Identifier for rw::math::Quaternion<> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idQuaternion();
	/**
	 * @brief Identifier for rw::math::Rotation2D<> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idRotation2D();
	/**
	 * @brief Identifier for rw::math::Transform3D<> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idTransform3D();
	/**
	 * @brief Identifier for rw::math::VelocityScrew6D<> in the XML format.
	 * @return the identifier.
	 */
    static const XMLCh* idVelocityScrew6D();
	/**
	 * @brief Identifier for the position specification used in Transform3D.
	 * @return the identifier.
	 */
    static const XMLCh* idPos();
	/**
	 * @brief Identifier for matrix specification used in Transform3D.
	 * @return the identifier.
	 */
    static const XMLCh* idMatrix();
	/**
	 * @brief Identifier for specifying the linear part in a VelocityScrew6D.
	 * @return the identifier.
	 */
    static const XMLCh* idLinear();
	/**
	 * @brief Identifier for specifying the angular part in a VelocityScrew6D.
	 * @return the identifier.
	 */
    static const XMLCh* idAngular();
	/**
	 * @brief Identifier for specifying a State.
	 * @return the identifier.
	 */
    static const XMLCh* idState();
	/**
	 * @brief Identifier for specifying a State.
	 * @return the identifier.
	 */
    static const XMLCh* idQState();
	/**
	 * @brief Identifier for specifying a State.
	 * @return the identifier.
	 */
    static const XMLCh* idTreeState();
	/**
	 * @brief Identifier for specifying a boolean.
	 * @return the identifier.
	 */
    static const XMLCh* idBoolean();
	/**
	 * @brief Identifier for specifying a double.
	 * @return the identifier.
	 */
    static const XMLCh* idDouble();
	/**
	 * @brief Identifier for specifying a float.
	 * @return the identifier.
	 */
    static const XMLCh* idFloat();
	/**
	 * @brief Identifier for specifying a integer.
	 * @return the identifier.
	 */
    static const XMLCh* idInteger();
	/**
	 * @brief Identifier for specifying a string.
	 * @return the identifier.
	 */
    static const XMLCh* idString();
	/**
	 * @brief Identifier for specifying a list of strings.
	 * @return the identifier.
	 */
    static const XMLCh* idStringList();
	/**
	 * @brief Identifier for specifying a list of integers.
	 * @return the identifier.
	 */
    static const XMLCh* idIntList();
	/**
	 * @brief Identifier for specifying a list of doubles.
	 * @return the identifier.
	 */
    static const XMLCh* idDoubleList();
	/**
	 * @brief Identifier for specifying a pair of strings.
	 * @return the identifier.
	 */
    static const XMLCh* idStringPair();
	/**
	 * @brief Identifier for the unit attribute.
	 * @return the identifier.
	 */
    static const XMLCh* idUnitAttribute();


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
     * If \b doCheckHeader = true it checks that the elements tag name matches Vector3D.
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
     * If \b doCheckHeader = true it checks that the elements tag name matches Vector2D.
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
     * If \b doCheckHeader = true it checks that the elements tag name matches RPY.
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
     * If \b doCheckHeader = true it checks that the elements tag name matches EAA.
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
     * If \b doCheckHeader = true it checks that the elements tag name matches Quaternion.
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
     * If \b doCheckHeader = true it checks that the elements tag name matches Rotation3D.
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
     * If \b doCheckHeader = true it checks that the elements tag name matches Rotation2D.
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
     * If the name does not an exception is thrown.
     *
     * @param element [in] Element to read
     * @return The element read
     */
    static rw::math::Rotation3D<> readRotation3DStructure(xercesc::DOMElement* element);

    /**
     * @brief Returns rw::math::Transform3D<> element read from \b element
     *
     * Read in \b element and returns a rw::math::Transform3D<> corresponding to the content.
     * If \b doCheckHeader = true it checks that the elements tag name matches Transform3D.
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
     * If \b doCheckHeader = true it checks that the elements tag name matches VelocityScrew6D.
     * If the name does not an exception is thrown.
     *
     * @param element [in] Element to read
     * @param doCheckHeader [in] True if the header name should be checked
     * @return The element read
     */
    static rw::math::VelocityScrew6D<> readVelocityScrew6D(xercesc::DOMElement* element, bool doCheckHeader = false);


    /**
     * @brief Returns rw::kinematics::State<> element read from \b element
     *
     * Reads in \b element and returns a rw::kinematics::State corresponding to the content.
     * If \b doCheckHeader = true it checks that the elements tag name matches State.
     * If the name does not an exception is thrown.
     * If the structure of the state does not match an exception is thrown.
     *
     * @param element [in] Element to read
     * @param workcell [in] WorkCell to which the state should match.
     * @param doCheckHeader [in] True if the header name should be checked
     * @return The State read from \b element
     */
    static rw::kinematics::State readState(xercesc::DOMElement* element,
		rw::common::Ptr<rw::models::WorkCell> workcell,
		bool doCheckHeader = false);


    /**
     * @brief Definition of a pair of strings
     */
    typedef std::pair<std::string, std::string> StringPair;

    /**
     * @brief Returns pair of strings from \b element
     *
     * Reads in \b element and returns a pair of strings.
     * If \b doCheckHeader = true it checks that the element tag name matches XMLBasicTypes::StringPairId.
     * Throws a rw::common::Exception if header does not match or if failing to read two strings.
     *
     * @param element [in] Element to read
     * @param doCheckHeader [in] True if the element name should be checked
     * @return Pair of strings
     */
    static StringPair readStringPair(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Reads in a list of string pairs that are childs of \b element
     *
     * Reads in all string pairs which are childs of the element \b element. This is
     * for example used when reading in DAF setups.
     *
     * Throws rw::common::Exception if failing to read and parse content.
     *
     * @param element [in] Element which string pairs as children
     * @return List of string pairs
     */
    static std::vector<StringPair> readStringPairs(xercesc::DOMElement* element);

    /**
     * @brief Reads in a list of strings that are childs of \b element
     *
     * Reads in all strings which are childs of the element \b element.
     *
     * Throws rw::common::Exception if failing to read and parse content.
     *
     * @param element [in] Element which string pairs as children
     * @return List of strings
     */
    static std::vector<std::string> readStringList(xercesc::DOMElement* element);

    /**
     * @brief Reads in a string element
     *
     * Reads in \b element as a string element.
     * Throws a rw::common::Exception on error
     *
     * @param element [in] Element to parse
     * @param doCheckHeader [in] True if the element name should be checked
     * @return String giving the content of \b element
     */
    static std::string readString(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Reads in a double element
     *
     * Read in \b element and converts the content to a double
     * Throws a rw::common::Exception if failing to read or parse.
     *
     * @param element [in] Element to read in
     * @param doCheckHeader [in] True if the element name should be checked
     * @return double represented in \b element
     */
    static double readDouble(xercesc::DOMElement* element, bool doCheckHeader = false);


    /**
     * @brief Reads in a double element
     *
     * Read in \b element and converts the content to a double
     * Throws a rw::common::Exception if failing to read or parse.
     *
     * @param element [in] Element to read in
     * @param doCheckHeader [in] True if the element name should be checked
     * @return double represented in \b element
     */
    static float readFloat(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Reads in an integer element
     *
     * Read in \b element and converts the content to an integer
     * Throws a rw::common::Exception if failing to read or parse.
     *
     * @param element [in] Element to read in
     * @param doCheckHeader [in] True if the element name should be checked
     * @return int represented in \b element
     */
    static int readInt(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Reads in a list of integers from \b element
     *
     * Read in \b element and converts the content to a list of integers
     * Throws a rw::common::Exception if failing to read or parse.
     *
     * @param element [in] Element to read in
     * @param doCheckHeader [in] True if the element name should be checked
     * @return std::vector<int> represented in \b element
     */
    static std::vector<int> readIntList(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Reads in a list of doubles from \b element
     *
     * Read in \b element and converts the content to a list of doubles
     * Throws a rw::common::Exception if failing to read or parse.
     *
     * @param element [in] Element to read in
     * @param doCheckHeader [in] True if the element name should be checked
     * @return std::vector<double> represented in \b element
     */    
    static std::vector<double> readDoubleList(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Reads in a boolean element
     *
     * Read in \b element and converts the content to a bool. Only if the content of the
     * node equals "true" is true returned. Otherwise the method returns false.
     *
     * Throws a rw::common::Exception if failing to read or parse.
     *
     * @param element [in] Element to read in
     * @param doCheckHeader [in] True if the element name should be checked
     * @return bool represented in \b element
     */
    static bool readBool(xercesc::DOMElement* element, bool doCheckHeader = false);

    /**
     * @brief Reads the text of a node
     *
     * Runs through all children of \b element until a DOMText-node if found. The value of this
     * node is the returned. In case of multiple nodes only the content of the first is
     * returned.
     *
     * If no DOMText-nodes can be found and \b exceptionOnEmpty is true it throws a rw::common::Exception.
     * Otherwise an empty string is returned.
     *
     * @param element [in] Element to read in
     * @param exceptionOnEmpty [in] Determines whether to throw an exception or return an empty string.
     * @return string content of first child DOMText-node
     */
    static std::string readElementText(xercesc::DOMElement* element, bool exceptionOnEmpty = true);

    /**
     * @brief Read element text and return as XMLCh*.
     *
     * Runs through all children of \b element until a DOMText-node if found. The value of this
     * node is the returned. In case of multiple nodes only the content of the first is
     * returned.
     *
     * If no DOMText-nodes can be found and \b exceptionOnEmpty is true it throws a rw::common::Exception.
     * Otherwise an empty string is returned.
     *
     * @param element [in] Element to read in
     * @param exceptionOnEmpty [in] Determines whether to throw an exception or return an empty string.
     * @return string content of first child DOMText-node
     */
    static const XMLCh* readElementTextXMLCh(xercesc::DOMElement* element, bool exceptionOnEmpty = true);

    /**
     * @brief Create an element with name \b id and content \b value in the DOMDocument \b doc
     *
     * Create an element named \b id and adds a DOMText-node containing \b value as a child to it.
     *
     * @param id [in] Name of the new element
     * @param value [in] Text value of the new element
     * @param doc [in] Document for which the new element shall be created
     *
     * @return The new element.
     */
    static xercesc::DOMElement* createElement(const XMLCh* id, const XMLCh* value, xercesc::DOMDocument* doc);

    /**
     * @brief Creates a DOMElement to represent \b q
     *
     * Creates a DOMElement owned by \b doc and representing \b q
     *
     * This method may throw a rw::common::Exception in case of errors
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
     * This method may throw a rw::common::Exception in case of errors
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
     * This method may throw a rw::common::Exception in case of errors
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
     * This method may throw a rw::common::Exception in case of errors
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
     * This method may throw a rw::common::Exception in case of errors
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
     * This method may throw a rw::common::Exception in case of errors
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
     * This method may throw a rw::common::Exception in case of errors
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
     * This method may throw a rw::common::Exception in case of errors
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
     * This method may throw a rw::comon::Exception in case of errors
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
     * This method may throw a rw::common::Exception in case of errors
     *
     * @param vs [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createVelocityScrew6D(const rw::math::VelocityScrew6D<>& vs, xercesc::DOMDocument* doc);

    /**
     * @brief Creates an element to represent \b state.
     *
     * Creates a DOMElement owned by \b doc and representing \b state
     *
     * The method may throw a rw::common::Exception in case of errors
     *
     * @param state [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createState(const rw::kinematics::State& state, xercesc::DOMDocument* doc);

    /**
     * @brief Creates an element to represent the rw::kinematics::QState contained in \b state.
     *
     * Creates a DOMElement owned by \b doc and representing the rw::kinematics::QState contained in \b state
     *
     * The method may throw a rw::common::Exception in case of errors
     *
     * @param state [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createQState(const rw::kinematics::State& state, xercesc::DOMDocument* doc);

    /**
     * @brief Creates an element to represent the rw::kinematics::TreeState contained in \b state.
     *
     * Creates a DOMElement owned by \b doc and representing the rw::kinematics::QState contained in \b state
     *
     * The method may throw a rw::common::Exception in case of errors
     *
     * @param state [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createTreeState(const rw::kinematics::State& state, xercesc::DOMDocument* doc);

     /**
      * @brief Creates an element to represent \b value.
      *
      * Creates a DOMElement owned by \b doc and representing \b value
      *
      * The method may throw a rw::common::Exception in case of errors
      *
      * @param value [in] Value to represent
      * @param doc [in] Document which should contain the element
      * @return Pointer to the newly created DOMElement
      */
    static xercesc::DOMElement* createBoolean(bool value, xercesc::DOMDocument* doc);

    /**
     * @brief Creates an element to represent \b value.
     *
     * Creates a DOMElement owned by \b doc and representing \b value
     *
     * The method may throw a rw::common::Exception in case of errors
     *
     * @param value [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createDouble(double value, xercesc::DOMDocument* doc);


    /**
     * @brief Creates an element to represent \b value.
     *
     * Creates a DOMElement owned by \b doc and representing \b value
     *
     * The method may throw a rw::common::Exception in case of errors
     *
     * @param value [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createFloat(float value, xercesc::DOMDocument* doc);


    /**
     * @brief Creates an element to represent \b value.
     *
     * Creates a DOMElement owned by \b doc and representing \b value
     *
     * The method may throw a rw::common::Exception in case of errors
     *
     * @param value [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createInteger(int value, xercesc::DOMDocument* doc);

    /**
     * @brief Creates an element to represent \b string.
     *
     * Creates a DOMElement owned by \b doc and representing \b string
     *
     * The method may throw a rw::common::Exception in case of errors
     *
     * @param string [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createString(const std::string& string, xercesc::DOMDocument* doc);

    /**
     * @brief Creates an element to represent \b strings.
     *
     * Creates a DOMElement owned by \b doc and representing \b strings
     *
     * The method may throw a rw::common::Exception in case of errors
     *
     * @param strings [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createStringList(const std::vector<std::string>& strings, xercesc::DOMDocument* doc);

    /**
     * @brief Creates an element to represent \b ints.
     *
     * Creates a DOMElement owned by \b doc and representing \b ints
     *
     * The method may throw a rw::common::Exception in case of errors
     *
     * @param ints [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createIntList(const std::vector<int>& ints, xercesc::DOMDocument* doc);
    
    /**
     * @brief Creates an element to represent \b doubles.
     *
     * Creates a DOMElement owned by \b doc and representing \b doubles
     *
     * The method may throw a rw::common::Exception in case of errors
     *
     * @param doubles [in] Value to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createDoubleList(const std::vector<double>& doubles, xercesc::DOMDocument* doc);

    /**
     * @brief Creates an element to represent strings \b first and \b second.
     *
     * Creates a DOMElement owned by \b doc and representing strings \b first and \b second
     *
     * The method may throw a rw::common::Exception in case of errors
     *
     * @param first [in] First string in the pair
     * @param second [in] Second string in the pair
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElement
     */
    static xercesc::DOMElement* createStringPair(const std::string& first, const std::string& second, xercesc::DOMDocument* doc);

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the XMLBasisTypes is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if XMLBasisTypes is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;

    XMLBasisTypes() {};
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif //end include guard
