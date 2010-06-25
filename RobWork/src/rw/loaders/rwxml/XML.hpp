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


#ifndef RW_LOADERS_XML_XML_HPP
#define RW_LOADERS_XML_XML_HPP

/**
   @file XML.hpp
*/

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <boost/property_tree/ptree.hpp>

namespace rw { namespace math { class Q; }}
namespace rw { namespace common { class PropertyMap; }}

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /*@{*/

    /**
       @brief XML is a collection of procedures for reading RobWork data
       structures stored in an XML format.

       The data structures are read from an XML file loaded into a Boost
       property tree.

       Eventually this parsing of XML structures should be shared between
       workcell and task files.

       The parser assumes that the node with the relevant name has been entered.
       For example:

\code
const Vector3D<> pos = XML::readVector3D(tree.get_child("Vector3D"));
\endcode
    */
    class XML
    {
    public:
        typedef boost::property_tree::ptree PTree;

        /**
\verbatim
<vector3d> ::= Vector3D <number> <number> <number>
\endverbatim
         */
        static math::Vector3D<> readVector3D(const PTree& tree);

        /**
\verbatim
<rotation3d> ::=
    RPY <number> <number> <number> |
    Rotation3D
        <number> <number> <number>
        <number> <number> <number>
        <number> <number> <number>
\endverbatim
         */
        static math::Rotation3D<> readRotation3D(const PTree& tree);

        /**
\verbatim
<transform3d> ::= Transform3D <vector3d> <rotation3d>
\endverbatim
        */
        static math::Transform3D<> readTransform3D(const PTree& tree);

        /**
\verbatim
<q> ::= Q <number> ... <number>
\endverbatim
        */
        static math::Q readQ(const PTree& tree);

        /**
\verbatim
<property-map> ::= PropertyMap <property> ...
<property> ::=
    Property
        (Key <string>)
        <description>?
        (<string> | <number> | <vector3d> | <rotation3d> | <transform3d> | <q>)
\endverbatim
        */
        static void readPropertyMap(
            const PTree& tree, common::PropertyMap& properties);

        /**
           @brief Print a formatted version of the tree to \b out.

           This utility is useful for figuring out how the XML representation
           corresponds to the property tree.
        */
        static
        void printTree(const PTree& tree, std::ostream& out);

    private:
        XML();
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
