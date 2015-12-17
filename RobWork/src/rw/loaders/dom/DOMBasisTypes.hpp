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

#ifndef RW_LOADERS_BASISTYPES_HPP
#define RW_LOADERS_BASISTYPES_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Transform2D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Rotation2D.hpp>
#include <rw/math/VelocityScrew6D.hpp>

#include <rw/kinematics/State.hpp>
#include <rw/models/WorkCell.hpp>

#include <rw/common/DOMElem.hpp>

#include <map>
#include <string>

namespace rw {
namespace loaders {

    /** @addtogroup loaders */
    /*@{*/


/**
 * @brief Utility class to help read in the content of a XML-files parsed with Xerces
 */
class DOMBasisTypes
{
public:
    /** @brief Identifier for rw::math::Q in the XML format  */
    static const std::string QId;

    /** @brief Identifier for rw::math::Vector3D<> in the XML format  */
    static const std::string Vector3DId;

    /** @brief Identifier for rw::math::Vector2D<> in the XML format  */
    static const std::string Vector2DId;

    /** @brief Identifier for rw::math::Rotation3D<> in the XML format  */
    static const std::string Rotation3DId;

    /** @brief Identifier for rw::math::RPY<> in the XML format  */
    static const std::string RPYId;

    /** @brief Identifier for rw::math::EAA<> in the XML format  */
    static const std::string EAAId;

    /** @brief Identifier for rw::math::Quaternion<> in the XML format  */
    static const std::string QuaternionId;

    /** @brief Identifier for rw::math::Rotation2D<> in the XML format  */
    static const std::string Rotation2DId;

    /** @brief Identifier for rw::math::Transform2D<> in the XML format  */
    static const std::string Transform2DId;

    /** @brief Identifier for rw::math::Transform3D<> in the XML format  */
    static const std::string Transform3DId;

    /** @brief Identifier for Eigen::MatrixXd<> in the XML format  */
    static const std::string MatrixId;

    /** @brief Identifier for rw::math::VelocityScrew6D<> in the XML format  */
    static const std::string VelocityScrew6DId;

    /** @brief Identifier for the position specification used in Transform3D  */
    static const std::string PosId;

    // /** @brief Identifier for matrix specification used in Transform3D  */
    //static const std::string MatrixId;

    /** @brief Identifier for specifying the linear part in a VelocityScrew6D  */
    static const std::string LinearId;

    /** @brief Identifier for specifying the angular part in a VelocityScrew6D  */
    static const std::string AngularId;

    /** @brief Identifier for specifying a State */
    static const std::string StateId;

    /** @brief Identifier for specifying a State */
    static const std::string QStateId;

    /** @brief Identifier for specifying a State */
    static const std::string TreeStateId;

    /** @brief Identifier for specifying a boolean*/
    static const std::string BooleanId;

    /** @brief Identifier for specifying a double */
    static const std::string DoubleId;

    /** @brief Identifier for specifying a float */
    static const std::string FloatId;

    /** @brief Identifier for specifying an integer */
    static const std::string IntegerId;

    /** @brief Identifier for specifying a string */
    static const std::string StringId;

    /** @brief Identifier for specifying a list of strings */
    static const std::string StringListId;

    /** @brief Identifier for specifying a list of integers */
    static const std::string IntListId;

    /** @brief Identifier for specifying a list of doubles */
    static const std::string DoubleListId;

    /** @brief Identifier for specifying a pair of strings */
    static const std::string StringPairId;

    /** @brief Identifier for the unit attribute */
    static const std::string UnitAttributeId;



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
    static double getUnit(const std::string key);

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
    static rw::math::Q readQ(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static rw::math::Vector3D<> readVector3D(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static rw::math::Vector2D<> readVector2D(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static rw::math::RPY<> readRPY(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static rw::math::EAA<> readEAA(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static rw::math::Quaternion<> readQuaternion(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static rw::math::Rotation3D<> readRotation3D(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static rw::math::Rotation2D<> readRotation2D(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static rw::math::Rotation3D<> readRotation3DStructure(rw::common::DOMElem::Ptr element);

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
    static rw::math::Transform3D<> readTransform3D(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static rw::math::VelocityScrew6D<> readVelocityScrew6D(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);


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
    static rw::kinematics::State readState(rw::common::DOMElem::Ptr element,
		rw::models::WorkCell::Ptr workcell,
		bool doCheckHeader = false);

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
    static std::pair<std::string, std::string> readStringPair(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static std::vector<std::pair<std::string, std::string> > readStringPairs(rw::common::DOMElem::Ptr element);

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
    static std::vector<std::string> readStringList(rw::common::DOMElem::Ptr element);

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
    static std::string readString(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static double readDouble(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);


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
    static float readFloat(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static int readInt(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static std::vector<int> readIntList(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);
    
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
    static std::vector<double> readDoubleList(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);

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
    static bool readBool(rw::common::DOMElem::Ptr element, bool doCheckHeader = false);




    //------------------------ writing value of DOMElem

	/** 
	 * @brief Writes \bval to \belem.
	 * @param val [in] Value to write
	 * @param elem [in] Element to which to write
	 * @param addHeader [in] Whether or not to set the header of \belem
     * @return newly created DOMElem
	 */
	static rw::common::DOMElem::Ptr write(int val, rw::common::DOMElem::Ptr elem, bool addHeader=true);

	/** 
	 * @brief Writes \bval to \belem.
	 * @param val [in] Value to write
	 * @param elem [in] Element to which to write
	 * @param addHeader [in] Whether or not to set the header of \belem
	 * @return newly created DOMElem
     */
	static rw::common::DOMElem::Ptr write(double val, rw::common::DOMElem::Ptr elem, bool addHeader=true);


	/** 
	 * @brief Writes \bstr to \belem.
	 * @param val [in] Value to write
	 * @param elem [in] Element to which to write
	 * @param addHeader [in] Whether or not to set the header of \belem
	 * @return newly created DOMElem
     */
	static rw::common::DOMElem::Ptr write(const std::string& str, rw::common::DOMElem::Ptr elem, bool addHeader=true);


	/** 
	 * @brief Writes the content of \bval to \belem.
	 * @param val [in] Value to write
	 * @param elem [in] Element to which to write
	 * @param addHeader [in] Whether or not to set the header of \belem
	 * @return newly created DOMElem
     */
	static rw::common::DOMElem::Ptr write(const rw::math::Q& val, rw::common::DOMElem::Ptr elem, bool addHeader=true);

	/** 
	 * @brief Writes the content of \bval to \belem.
	 * @param val [in] Value to write
	 * @param elem [in] Element to which to write
	 * @param addHeader [in] Whether or not to set the header of \belem
	 * @return newly created DOMElem
     */
	static rw::common::DOMElem::Ptr write(const rw::math::Vector3D<>& val, rw::common::DOMElem::Ptr elem, bool addHeader=true);

	/** 
	 * @brief Writes the content of \bval to \belem.
	 * @param val [in] Value to write
	 * @param elem [in] Element to which to write
	 * @param addHeader [in] Whether or not to set the header of \belem
	 * @return newly created DOMElem
     */
	static rw::common::DOMElem::Ptr write(const rw::math::Vector2D<>& val, rw::common::DOMElem::Ptr elem, bool addHeader=true);


	/** 
	 * @brief Writes the content of \bval to \belem.
	 * @param val [in] Value to write
	 * @param elem [in] Element to which to write
	 * @param addHeader [in] Whether or not to set the header of \belem
	 * @return newly created DOMElem
     */
	static rw::common::DOMElem::Ptr write(const rw::math::Transform2D<>& val, rw::common::DOMElem::Ptr elem, bool addHeader=true);
    
	/** 
	 * @brief Writes the content of \bval to \belem.
	 * @param val [in] Value to write
	 * @param elem [in] Element to which to write
	 * @param addHeader [in] Whether or not to set the header of \belem
	 * @return newly created DOMElem
     */
	static rw::common::DOMElem::Ptr write(const rw::math::Transform3D<>& val, rw::common::DOMElem::Ptr elem, bool addHeader=true);
    
	/** 
	 * @brief Writes the content of \bval to \belem.
	 * @param val [in] Value to write
	 * @param elem [in] Element to which to write
	 * @param addHeader [in] Whether or not to set the header of \belem
	 * @return newly created DOMElem
     */
	static rw::common::DOMElem::Ptr write(const Eigen::MatrixXd& val, rw::common::DOMElem::Ptr elem, bool addHeader=true);

	
	/**
	 * @brief Reads in a matrix from \belement
	 * @param elem [in] The element from which to read
	 * @return Eigen matrix 
	 */
	static Eigen::MatrixXd readMatrix( rw::common::DOMElem::Ptr elem );


    //------------------------ creating and writing DOMElem
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
    static rw::common::DOMElem::Ptr createElement(const std::string& id, const std::string& value, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createQ(const rw::math::Q& q, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createVector3D(const rw::math::Vector3D<>& v, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createVector2D(const rw::math::Vector2D<>& v, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createRPY(const rw::math::RPY<>& rpy, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createEAA(const rw::math::EAA<>& eaa, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createQuaternion(const rw::math::Quaternion<>& quat, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createRotation3D(const rw::math::Rotation3D<>& rot, rw::common::DOMElem::Ptr doc);


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
    static rw::common::DOMElem::Ptr createRotation2D(const rw::math::Rotation2D<>& rot, rw::common::DOMElem::Ptr doc);


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
    static rw::common::DOMElem::Ptr createTransform3D(const rw::math::Transform3D<>& trans, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createVelocityScrew6D(const rw::math::VelocityScrew6D<>& vs, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createState(const rw::kinematics::State& state, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createQState(const rw::kinematics::State& state, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createTreeState(const rw::kinematics::State& state, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createBoolean(bool value, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createDouble(double value, rw::common::DOMElem::Ptr doc);


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
    static rw::common::DOMElem::Ptr createFloat(float value, rw::common::DOMElem::Ptr doc);


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
    static rw::common::DOMElem::Ptr createInteger(int value, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createString(const std::string& string, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createStringList(const std::vector<std::string>& strings, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createIntList(const std::vector<int>& ints, rw::common::DOMElem::Ptr doc);
    
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
    static rw::common::DOMElem::Ptr createDoubleList(const std::vector<double>& doubles, rw::common::DOMElem::Ptr doc);

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
    static rw::common::DOMElem::Ptr createStringPair(const std::string& first, const std::string& second, rw::common::DOMElem::Ptr doc);


private:
    DOMBasisTypes() {};

    /*
     * Map used for mapping unit identifiers o their corresponding values
     */


	

    //static const UnitMap _Units;
};

/** @} */

} //end namespace rw
}
#endif //end include guard
