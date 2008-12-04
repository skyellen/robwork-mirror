/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

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
