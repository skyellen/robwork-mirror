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

#include "TULLoader.hpp"

#include "Tag.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Constants.hpp>

#include <rw/models/Models.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/JointDevice.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/CompositeDevice.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/VirtualJoint.hpp>
#include <rw/models/DependentRevoluteJoint.hpp>
#include <rw/models/DependentPrismaticJoint.hpp>
#include <rw/models/DHParameterSet.hpp>

#include <rw/loaders/colsetup/CollisionSetupLoader.hpp>
#include <rw/proximity/CollisionSetup.hpp>

#include <rw/common/StringUtil.hpp>
#include <rw/common/macros.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FrameType.hpp>

#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/common/Property.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include <string>
#include <map>
#include <cfloat>
#include <vector>

using namespace rw;
using namespace rw::math;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::loaders;

using namespace std;



namespace rw { namespace kinematics {



/** @addtogroup kinematics */
/*@{*/

/**
 * @brief Interface for utilities for accessing a named property of a frame.
 *
 * FrameProperty is a utility for accessing the property of a frame as
 * painlessly as possible. The class may therefore happen to have a few
 * superflous methods that are abbreviations for combinations of the other
 * methods.
 *
 * FrameProperty hides the low-level manipulations of PropertyMap values
 * of frames.
 *
 * FrameProperty throws an exception to give good defaults for the common
 * case where your program has no option but to crash if some option is not
 * present.
 */
template <typename T>
class FrameProperty
{
public:
    /**
     * @brief The name of the property that is being accessed.
     *
     * @return The name of the property.
     */
    virtual const std::string& key() const = 0;

    /**
     * @brief The value of the property of a frame.
     *
     * @param frame [in] A frame containing properties.
     *
     * @return A pointer to the value of the property of the frame or NULL
     * if the property does not exist or if the value of the property is of
     * a wrong type.
     */
    virtual T* getPtr(Frame& frame) const = 0;

    /**
     * @copydoc getPtr
     */
    virtual const T* getPtr(const Frame& frame) const = 0;

    /**
     * @brief True iff the frame has the property.
     *
     * A call of the method is equivalent to
\code
getPtr(frame) != 0
\endcode
     *
     * @param frame [in] A frame containing properties.
     *
     * @return \b true iff the property exists in \b frame.
     */
    virtual bool has(const Frame& frame) const = 0;

    /**
     * @brief The value of the property of a frame.
     *
     * The method is superflous as it can be easily implemented in terms of
     * getPtr().
     *
     * @param frame [in] A frame containing properties.
     *
     * @return The value of the property of the frame.
     *
     * The property must be present in the frame or the program will throw. If
     * you don't like that policy you should use getPtr().
     */
    virtual const T& get(const Frame& frame) const = 0;

    /**
     * @brief The value of the property of a frame.
     *
     * The method is superflous as it can be easily implemented in terms of
     * getPtr().
     *
     * @param frame [in] A frame containing properties.
     *
     * @return The value of the property of the frame.
     *
     * The property must be present in the frame or the program will throw. If
     * you don't like that policy you should use getPtr().
     */
    virtual T& get(Frame& frame) const = 0;

    /**
     * @brief Assign a value to the property of the frame.
     *
     * @param frame [in] A frame containing properties.
     *
     * @param value [in] The value to assign to the property.
     *
     * [NB: We haven't quite decided yet what to do if a value is already
     * present (we could have single-assignment) or if the value present is
     * of a wrong type.]
     */
    virtual void set(Frame& frame, const T& value) const = 0;

    /**
     * @brief Remove a property of a frame.
     *
     * Following a call of erase() the has() will return false.
     *
     * If erase() is called on a frame that does not have the property, then
     * nothing is done.
     *
     * The property is removed also if its type does not match.
     *
     * @param frame [in] the frame from which to remove the property.
     */
    virtual void erase(Frame& frame) const = 0;

    /**
     * @brief Virtual destructor.
     */
    virtual ~FrameProperty() {}

protected:
    FrameProperty() {}

private:
    FrameProperty(const FrameProperty&);
    FrameProperty& operator=(const FrameProperty&);
};


    /**
     * @brief An implementation of the FrameProperty class.
     */
    template <typename T>
    class FramePropertyImpl : public FrameProperty<T>
    {
    public:
        /**
         * @brief An accessor for a named property of a frame.
         *
         * @param key [in] The name of property to access.
         *
         * @param description [in] The description of the property.
         */
        FramePropertyImpl(const std::string& key, const std::string& description) :
            _key(key),
            _description(description)
        {}

        /**
         * @brief The name of the property that is being accessed.
         *
         * @return The name of the property.
         */
        const std::string& key() const { return _key; }

        /**
         * @brief The value of the property of a frame.
         *
         * @param frame [in] A frame containing properties.
         *
         * @return A pointer to the value of the property of the frame or NULL
         * if the property does not exist or if the value of the property is of
         * a wrong type.
         */
        T* getPtr(Frame& frame) const
        {
            return frame.getPropertyMap().template getPtr<T>(_key);
        }

        /**
         * @brief The value of the property of a frame.
         *
         * @param frame [in] A frame containing properties.
         *
         * @return A pointer to the value of the property of the frame or NULL
         * if the property does not exist or if the value of the property is of
         * a wrong type.
         */
        const T* getPtr(const Frame& frame) const
        {
            // Forward to non-const version.
            return getPtr(const_cast<Frame&>(frame));
        }

        /**
         * @brief True iff the frame has the property.
         *
         * A call of the method is equivalent to
         \code
         getPtr(frame) != 0
         \endcode
         *
         * @param frame [in] A frame containing properties.
         *
         * @return \b true iff the property exists in \b frame.
         */
        bool has(const Frame& frame) const
        {
            return this->getPtr(frame) != NULL;
        }

        /**
         * @brief The value of the property of a frame.
         *
         * The method is superflous as it can be easily implemented in terms of
         * getPtr().
         *
         * @param frame [in] A frame containing properties.
         *
         * @return The value of the property of the frame.
         *
         * The property must be present in the frame or the program will throw.
         * If you don't like that policy you should use getPtr().
         */
        const T& get(const Frame& frame) const
        {
            // Forward to the non-const version
            return get(const_cast<Frame&>(frame));
        }

        /**
         * @brief The value of the property of a frame.
         *
         * The method is superflous as it can be easily implemented in terms of
         * getPtr().
         *
         * @param frame [in] A frame containing properties.
         *
         * @return The value of the property of the frame.
         *
         * The property must be present in the frame or the program will throw.
         * If you don't like that policy you should use getPtr().
         */
        T& get(Frame& frame) const
        {
            T* value = this->getPtr(frame);
            if (!value) {
                // Perhaps this error message should include the description of
                // the property also.
                RW_THROW(
                    "No key (of some type T) named "
                    << rw::common::StringUtil::quote(key())
                    << " in frame "
                    << rw::common::StringUtil::quote(frame.getName()));
            }
            return *value;
        }

        /**
         * @brief Assign a value to the property of the frame.
         *
         * @param frame [in] A frame containing properties.
         *
         * @param value [in] The value to assign to the property.
         */
        void set(Frame& frame, const T& value) const
        {
            frame.getPropertyMap().set(_key, value);
        }

        /**
           @brief Erase the property of the frame.
         */
        void erase(Frame& frame) const
        {
            frame.getPropertyMap().erase(_key);
        }

    private:
        std::string _key;
        std::string _description;
    };

    /*@}*/
}} // end namespaces

//----------------------------------------------------------------------
// Tag properties

// Tag properties are loaded from a file and are henceforth read-only (or least
// with respect to the structure and types).

namespace
{
    string quote(const string& str) { return StringUtil::quote(str); }

    // Formatting of error messages.
    string msgHeader(
        const Tag& tag, const string& attribute)
    {
        stringstream buf;
        buf
            << "Attribute "
            << quote(attribute)
            << " (frame: "
            << quote(tag.getName())
            << ", file: "
            << quote(tag.getFile())
            << "): ";
        return buf.str();
    }

    // Formatting of error messages.
    string msgHeader(const Tag& tag)
    {
        stringstream buf;
        buf
            << "(frame: "
            << quote(tag.getName())
            << ", file: "
            << quote(tag.getFile())
            << "): ";
        return buf.str();
    }

    const FrameProperty<Tag>& tagAccessor()
    {
        static FramePropertyImpl<Tag> accessor(
            "#TAG", "the tag of the frame");
        return accessor;
    }

    const FrameProperty<Transform3D<> >& movableFrameTransformAccessor()
    {
        static FramePropertyImpl<Transform3D<> > accessor(
            "#MOVABLE_FRAME_TRANSFORM",
            "the initial relative transform of the movable frame");
        return accessor;
    }

    // Nil is a class that cannot be constructed or copied.
    class Nil
    {
    private:
        Nil(const Nil&);
        Nil& operator=(const Nil&);
        Nil();
    };

    const Tag& getTag(const Frame& frame)
    {
        return tagAccessor().get(frame);
    }

    void setTag(Frame& frame, const Tag& tag)
    {
        return tagAccessor().set(frame, tag);
    }

    template <typename T>
    class TagProperty
    {
    public:
        TagProperty(const string& key) :
            _key(key)
        {}

        const string& key() const { return _key; }

        bool has(const Tag& tag) const
        { return hasAttribute(tag, _key); }

        bool has(const Frame& frame) const
        { return has(getTag(frame)); }

        const T& get(const Tag& tag, int pos) const
        { return getAttribute<T>(tag, _key, pos); }

        const T& get(const Frame& frame, int pos) const
        { return get(getTag(frame), pos); }

        // A templated version of get(). I wish I had a better name.
        template <class R>
        const R& getT(const Tag& tag, int pos) const
        { return getAttribute<R>(tag, _key, pos); }

        template <class R>
        const R& getT(const Frame& frame, int pos) const
        { return getT<R>(getTag(frame), pos); }

        int size(const Tag& tag) const
        { return getAttributeSize(tag, _key); }
        int size(const Frame& frame) const
        { return size(getTag(frame)); }

    private:
        string _key;
    };

    template <class T>
    vector<T> getAsVector(
        const TagProperty<T>& prop,
        const Tag& tag)
    {
        vector<T> result;
        const int len = prop.size(tag);
        for (int i = 0; i < len; i++)
            result.push_back(prop.get(tag, i));
        return result;
    }

    template <class T>
    vector<T> getAsVector(
        const TagProperty<T>& prop,
        const Frame& frame)
    {
        return getAsVector(prop, getTag(frame));
    }

    const TagProperty<double>& tagPropCraigDH()
    {
        static TagProperty<double> getter("CraigDH");
        return getter;
    }

    /**
       The TUL syntax is:

         PassiveRevolute <frame-name> <scale> <offset>

       where <frame-name> may be with or without a prefix.

       We may later want to allow scale to default to 1 and offset to 0.
    */
    const TagProperty<string>& tagPropPassiveRevolute()
    {
        static TagProperty<string> getter("PassiveRevolute");
        return getter;
    }

    const TagProperty<string>& tagPropPassivePrismatic()
    {
        static TagProperty<string> getter("PassivePrismatic");
        return getter;
    }

    const TagProperty<string>& tagPropReferenceFrame()
    {
        static TagProperty<string> getter("ReferenceFrame");
        return getter;
    }

    const TagProperty<string>& tagPropCollisionSetup()
    {
        static TagProperty<string> getter("CollisionSetup");
        return getter;
    }

    const TagProperty<string>& tagPropGeoID()
    {
        static TagProperty<string> getter("GeoID");
        return getter;
    }

    // Takes care of info only to be used for visualization
    const TagProperty<string>& tagPropDrawableID()
    {
        static TagProperty<string> getter("DrawableID");
        return getter;
    }

    const TagProperty<bool>& tagPropDrawableHighlight()
    {
        static TagProperty<bool> getter("DrawableHighlight");
        return getter;
    }

    const TagProperty<bool>& tagPropDrawableWireMode()
    {
        static TagProperty<bool> getter("DrawableWireMode");
        return getter;
    }

    // Takes care of info only to be used for collision checking
    const TagProperty<string>& tagPropCollisionModelID()
    {
        static TagProperty<string> getter("CollisionModelID");
        return getter;
    }

    const TagProperty<double>& tagPropGeoScale()
    {
        static TagProperty<double> getter("GeoScale");
        return getter;
    }

    const TagProperty<double>& tagPropJointPosLimit()
    {
        static TagProperty<double> getter("JointPosLimit");
        return getter;
    }

    const TagProperty<double>& tagPropJointVelLimit()
    {
        static TagProperty<double> getter("JointVelLimit");
        return getter;
    }

    const TagProperty<double>& tagPropJointAccLimit()
    {
        static TagProperty<double> getter("JointAccLimit");
        return getter;
    }

    const TagProperty<double>& tagPropJointHomePos()
    {
        static TagProperty<double> getter("JointHomePos");
        return getter;
    }

    const TagProperty<Q>& tagPropDeviceHomePos()
    {
        static TagProperty<Q> getter("DeviceHomePos");
        return getter;
    }

    const TagProperty<Nil>& tagPropDAF()
    {
        static TagProperty<Nil> getter("DAF");
        return getter;
    }

    const TagProperty<Nil>& tagPropRevolute()
    {
        static TagProperty<Nil> getter("Revolute");
        return getter;
    }

    const TagProperty<Nil>& tagPropPrismatic()
    {
        static TagProperty<Nil> getter("Prismatic");
        return getter;
    }

    const TagProperty<Nil>& tagPropFixed()
    {
        static TagProperty<Nil> getter("Fixed");
        return getter;
    }

    /*
      The TUL syntax is

        CompositeDevice <dev1> <dev2> ... <devN>

      where <dev1> ... <devN> are (frame) names of other devices. The composite
      devices are constructed only at the very end, so you can actually refer to
      devices that have not yet been loaded.

      Beware however that you can only refer to devices that are in the current
      Scope or below. I.e. you cannot refer to devices loaded from outer scopes.
    */
    const TagProperty<string>& tagPropCompositeDevice()
    {
        static TagProperty<string> getter("CompositeDevice");
        return getter;
    }

    const TagProperty<string>& tagPropDevice()
    {
        static TagProperty<string> getter("Device");
        return getter;
    }

    const TagProperty<Nil>& tagPropActiveJoint()
    {
        static TagProperty<Nil> getter("ActiveJoint");
        return getter;
    }

    const TagProperty<Nil>& tagPropMovable()
    {
        static TagProperty<Nil> getter("Movable");
        return getter;
    }

    const TagProperty<Nil>& tagPropMoveable()
    {
        static TagProperty<Nil> getter("Moveable");
        return getter;
    }

    const TagProperty<Nil>& tagPropLink()
    {
        static TagProperty<Nil> getter("Link");
        return getter;
    }

    const TagProperty<Nil>& tagPropObject()
    {
        static TagProperty<Nil> getter("Object");
        return getter;
    }

    const TagProperty<Nil>& tagPropCamera()
    {
        static TagProperty<Nil> getter("Camera");
        return getter;
    }

    // Transform values.

    const TagProperty<Q>& tagPropI()
    {
        static TagProperty<Q> getter("I");
        return getter;
    }

    const TagProperty<Q>& tagPropJ()
    {
        static TagProperty<Q> getter("J");
        return getter;
    }

    const TagProperty<Q>& tagPropK()
    {
        static TagProperty<Q> getter("K");
        return getter;
    }

    const TagProperty<Q>& tagPropRPY()
    {
        static TagProperty<Q> getter("RPY");
        return getter;
    }

    const TagProperty<Q>& tagPropPosition()
    {
        static TagProperty<Q> getter("Position");
        return getter;
    }

    Vector3D<> getVectorOrThrow(
        const TagProperty<Q>& getter,
        const Tag& tag)
    {
        const Q& q = getter.get(tag, 0);
        if (q.size() != 3)
            RW_THROW(
                msgHeader(tag, getter.key())
                << "Vector of length "
                << (int)q.size()
                << " should have been a 3D vector.");

        return Vector3D<>(q[0], q[1], q[2]);
    }

    // We default to the zero vector.
    Vector3D<> getPosition(const Tag& tag)
    {
        if (tagPropPosition().has(tag))
            return getVectorOrThrow(tagPropPosition(), tag);
        else
            return Vector3D<>(0, 0, 0);
    }

    // We default to the unit rotation.
    Rotation3D<> getRotation(const Tag& tag)
    {
        if (tagPropRPY().has(tag)) {
            const Vector3D<> angles =
                Deg2Rad * getVectorOrThrow(tagPropRPY(), tag);

            const RPY<> rpy(angles[0], angles[1], angles[2]);
            return rpy.toRotation3D();
        } else if (
            tagPropI().has(tag) ||
            tagPropJ().has(tag) ||
            tagPropK().has(tag))
        {
            const Vector3D<> i = getVectorOrThrow(tagPropI(), tag);
            const Vector3D<> j = getVectorOrThrow(tagPropJ(), tag);
            const Vector3D<> k = getVectorOrThrow(tagPropK(), tag);

            return Rotation3D<>(i, j, k);
        } else {
            return Rotation3D<>::identity();
        }
    }

    struct CraigDH
    {
        CraigDH(
            double alpha,
            double a,
            double d,
            double theta)
            :
            alpha(alpha),
            a(a),
            d(d),
            theta(theta)
        {}

        double alpha;
        double a;
        double d;
        double theta;
    };

    CraigDH getCraigDH(const Tag& tag)
    {
        RW_ASSERT(tagPropCraigDH().has(tag));

        const vector<double> vals = getAsVector(tagPropCraigDH(), tag);
        if (vals.size() != 4) {
            RW_THROW(
                msgHeader(tag)
                << "Exactly 4 Craig DH parameters expected. "
                << "The number of parameters is "
                << (int)vals.size());
        }

        const double alpha = vals[0] * Deg2Rad;
        const double a = vals[1];
        const double d = vals[2];
        const double theta = vals[3] * Deg2Rad;

        return CraigDH(alpha, a, d, theta);
    }

    CraigDH getCraigDH(const Frame& frame)
    {
        return getCraigDH(getTag(frame));
    }

    Transform3D<> getCraigDHTransform(const Tag& tag)
    {
        const CraigDH dh = getCraigDH(tag);
        return Transform3D<>::craigDH(dh.alpha, dh.a, dh.d, dh.theta);
    }

    // We default to the unit transform.
    Transform3D<> getTransform(const Tag& tag)
    {
        if (tagPropCraigDH().has(tag) &&
            (tagPropPosition().has(tag) ||
             tagPropI().has(tag) ||
             tagPropJ().has(tag) ||
             tagPropK().has(tag) ||
             tagPropRPY().has(tag)))
        {
            RW_THROW(
                msgHeader(tag)
                << "Attribute 'CraigDH' is in conflict "
                "with other transform attribute.");
        } else if (tagPropCraigDH().has(tag)) {
            return getCraigDHTransform(tag);
        } else {
            return Transform3D<>(
                getPosition(tag),
                getRotation(tag));
        }
    }

    // We default to the empty string.
    string getReferenceFrame(const Tag& tag)
    {
        if (tagPropReferenceFrame().has(tag))
            return tagPropReferenceFrame().get(tag, 0);
        else
            return string();
    }
}

//----------------------------------------------------------------------

namespace
{
    /**
     * @brief The file name (relative or absolute) given in the context of the
     * directory \a dir.
     */
    string getFileNameOfDirectory(
        const string& dir,
        const string& file)
    {
        const string& filename = StringUtil::replaceBackslash(file);

        if (StringUtil::isAbsoluteFileName(filename))
            return filename;
        else
            return dir + filename;
    }

    /**
     * @brief The file name \a file in the context of \a tag.
     */
    string getFileNameOfTag(
        const Tag& tag,
        const string& file)
    {
        return getFileNameOfDirectory(
            StringUtil::getDirectoryName(tag.getFile()), file);
    }

    /**
     * @brief The file name \a file in the context of \a frame.
     */
    string getFileNameOfFrame(
        const Frame& frame,
        const string& file)
    {
        return getFileNameOfTag(getTag(frame), file);
    }
}

//----------------------------------------------------------------------
// Special supported properties.

namespace
{
    // The scope within which a frame name is given.
    class Prefix
    {
        typedef vector<string> V;
        typedef V::const_iterator VI;

        // A mapping from frame name to frame.
        typedef std::map<string, Frame*> TULFrameMap;

    public:
        Prefix() :
            _frameMap(new TULFrameMap())
        {}

        // Enter and return the new scope.
        Prefix enter(const string& scope) const
        {
            Prefix newPrefix(*this);
            newPrefix._path.push_back(scope);
            return newPrefix;
        }

        // The frame referred to by \b name or NULL if no such frame exists.
        Frame* resolve(const string& name) const
        {
            const VI begin = _path.begin();
            VI end = _path.end();
            const int maxCnt = (int)_path.size() + 1;
            for (int cnt = 0; cnt < maxCnt; ++cnt, --end) {
                const string frameName =
                    getFrameName(begin, end, name);

                Frame* frame = lookupFrame(frameName);
                if (frame) return frame;
            }

            return NULL;
        }

        // Register \b frame in the scope.
        void insert(Frame* frame) const
        {
            _frameMap->insert(
                std::make_pair(
                    frame->getName(),
                    frame));
        }

        // The frame name to use for tag \b tagName.
        string getFrameName(const string& tagName) const
        {
            return getFrameName(_path.begin(), _path.end(), tagName);
        }

        // Prefix description to use for error messages.
        string getPrefixDescription() const
        {
            return getFrameName("");
        }

    private:
        /**
           A string of the form x0.x1.x2...xN.name.
        */
        static
        string getFrameName(
            VI from, VI to, const string& name)
        {
            stringstream buf;
            for (VI p = from; p != to; ++p) {
                buf << *p << ".";
            }
            buf << name;
            return buf.str();
        }

        /**
           The frame of identifier \b name or NULL if no such frame.
        */
        Frame* lookupFrame(const string& name) const
        {
            typedef TULFrameMap::const_iterator I;
            const I end = _frameMap->end();
            const I pos = _frameMap->find(name);
            if (pos != end) {
                RW_ASSERT(pos->second);
                return pos->second;
            }
            else
                return NULL;
        }

    private:
        V _path;

        // This is too tricky: The frame map is a global shared map whereas the
        // path is a local value that is created anew every time a new scope is
        // entered.
        boost::shared_ptr<TULFrameMap> _frameMap;
    };

    // This function implies that we can't have '.' in setup files. That
    // character we reserve for showing the hierarchy. The parser should enforce
    // this rule (!).
    string getPrefix(const Frame& frame)
    {
        const string& name = frame.getName();

        const string::size_type pos = name.find_last_of(".");
        if (pos != string::npos)
            return name.substr(0, pos + 1);
        else
            return string();
    }
    // The getPrefix() function is used for the loading of collision setups.
    // Probably we should integrate the loading of collision setups so that an
    // object of type Prefix was passed to the loader instead.

    /**
     * @brief Build a collision setup for a workcell.
     */
    CollisionSetup makeCollisionSetup(
        const WorkCell& workcell)
    {
        const vector<Frame*>& frames = Models::findAllFrames(workcell);

        const std::vector<std::pair<std::string,std::string> > empty_list;
        CollisionSetup result(empty_list);

        typedef vector<Frame*>::const_iterator I;
        for (I p = frames.begin(); p != frames.end(); ++p) {
            const Frame& frame = **p;

            if (tagPropCollisionSetup().has(frame)) {
                const string& setupFile =
                    tagPropCollisionSetup().get(frame, 0);

                // Remember to resolve the file name.
                const string& file = getFileNameOfFrame(frame, setupFile);

                // The string to prefix the frame values.
                const string& prefix = getPrefix(frame);

                // Load the file and merge into result.
                result =
                    CollisionSetup::merge(
                        result,
                        CollisionSetupLoader::load(prefix, file));
            }
        }

        return result;
    }

    void addCollisionSetupProperty(WorkCell& workcell)
    {
        const CollisionSetup& setup = makeCollisionSetup(workcell);
        CollisionSetup::set(setup, &workcell);
    }

    std::string getAccessorID(
        const Frame& frame,
        const TagProperty<string>& accessor,
        int pos)
    {
        const string& geo = accessor.get(frame, pos);

        if (geo.empty()) RW_THROW("Empty model identifier.");

        if (geo[0] != '#') {
            // Remember to resolve the file name.
            return getFileNameOfFrame(frame, geo);
        } else {
            return geo;
        }
    }

    // Model identifiers for the accessor given.
    std::vector<std::string> getAccessorIDs(
        const Frame& frame,
        const TagProperty<string>& accessor)
    {
        std::vector<std::string> result;
        if (accessor.has(frame)) {
            const int len = accessor.size(frame);
            for (int pos = 0; pos < len; pos++)
                result.push_back(getAccessorID(frame, accessor, pos));
        }

        return result;
    }

    // Model identifiers for the accessor given as well as for the GeoID attribute.
    std::vector<std::string> getGeoIDAndAccessorIDs(
        const Frame& frame,
        const TagProperty<string>& accessor)
    {
        std::vector<std::string> result =
            getAccessorIDs(frame, accessor);

        std::vector<std::string> other =
            getAccessorIDs(frame, tagPropGeoID());

        result.insert(result.end(), other.begin(), other.end());

        return result;
    }

    // The geo scale of the frame or 1 otherwise.
    double getOptionalGeoScale(const Frame& frame)
    {
    	if (tagPropGeoScale().has(frame)) {
            const double scale = tagPropGeoScale().get(frame, 0);
            if (scale < 0) RW_THROW(
                "Negative GeoScale parameter "
                << scale
                << " for frame "
                << quote(frame.getName()));
            return scale;
        } else {
            return 1;
        }
    }

    // We need to rewrite these functions so that they work for sequences of
    // model identifiers also.
    void addDrawableModelInfo(Frame& frame)
    {
    	//bool high = false;
        //if (tagPropDrawableHighlight().has(frame))
        //    high = true;

        //bool wiremode = false;
        //if (tagPropDrawableWireMode().has(frame))
        //    wiremode = true;

    	//const double geoScale = getOptionalGeoScale(frame);

        const std::vector<std::string> ids =
            getGeoIDAndAccessorIDs(frame, tagPropDrawableID());

        /*
        TODO: add drawables to the scene graph
        if (!ids.empty()) {
            DrawableModelInfo::set(std::vector<DrawableModelInfo>(), &frame);
        }

        std::vector<DrawableModelInfo> infos = DrawableModelInfo::get(&frame);
        BOOST_FOREACH(const std::string& id, ids) {
            infos.push_back(
                DrawableModelInfo(
                    id,
                    id,//todo: get name
                    Transform3D<>::identity(),
                    geoScale,
                    high,
                    wiremode));
        }
        DrawableModelInfo::set(infos, &frame);
*/
    }

    void addCollisionModelInfo(Frame& frame)
    {
        /*
         * TODO:collisionMODEL
    	const double geoScale = getOptionalGeoScale(frame);

        const std::vector<std::string> ids =
            getGeoIDAndAccessorIDs(frame, tagPropCollisionModelID());

        if (!ids.empty()) {
            CollisionModelInfo::set(std::vector<CollisionModelInfo>(), &frame);
        }

        std::vector<CollisionModelInfo> infos = CollisionModelInfo::get(&frame);
        BOOST_FOREACH(const std::string& id, ids) {
            infos.push_back(
                CollisionModelInfo(
                    id,
                    id,
                    Transform3D<>::identity(),
                    geoScale));
        }
        CollisionModelInfo::set(infos, &frame);
        */
    }

    void addFrameTypeProperty(Frame& frame)
    {
        /*
        if (dynamic_cast<RevoluteJoint*>(&frame)) {
            Accessor::frameType().set(frame, FrameType::RevoluteJoint);
        } else if (dynamic_cast<PrismaticJoint*>(&frame)) {
            Accessor::frameType().set(frame, FrameType::PrismaticJoint);
        } else if (dynamic_cast<FixedFrame*>(&frame)) {
            Accessor::frameType().set(frame, FrameType::FixedFrame);
        } else if (dynamic_cast<MovableFrame*>(&frame)) {
            Accessor::frameType().set(frame, FrameType::MovableFrame);
        } else {
            Accessor::frameType().set(frame, FrameType::Unknown);
        }
        */
    }

    void addActiveJointProperty(Frame& frame)
    {
        if (!tagPropActiveJoint().has(frame)){
            // set the joint to be passive
            Joint *j = dynamic_cast<Joint*>(&frame);
            if(j){
                j->setActive(false);
            }
        }
    }

    void addDHSetProperty(Frame& frame)
    {
        if (tagPropCraigDH().has(frame)) {
            const CraigDH dh = getCraigDH(frame);
            DHParameterSet::set(
                DHParameterSet(
                    dh.alpha,
                    dh.a,
                    dh.d,
                    dh.theta),
                    &frame);
        }
    }

    // Assign all special properties.
    void initProperties(Frame* world, const State& state)
    {
        BOOST_FOREACH(Frame* frame, Kinematics::findAllFrames(world, state)) {
            addFrameTypeProperty(*frame);
            addDrawableModelInfo(*frame);
            addCollisionModelInfo(*frame);
            addActiveJointProperty(*frame);
            addDHSetProperty(*frame);

            // We don't add any CollisionSetup property, currently, as that is
            // done only for the root. (See addCollisionSetupProperty()).
        }
    }

    void initCollisionSetup(WorkCell& workcell)
    {
        addCollisionSetupProperty(workcell);
    }
}

//----------------------------------------------------------------------

namespace
{
    struct DeviceStruct
    {
        DeviceStruct(
            const string& name,
            Frame* first,
            Frame* last,
            const vector<Joint*>& joints,
            const Q& q_home)
            :
            name(name),
            first(first),
            last(last),
            joints(joints),
            q_home(q_home)
        {}

        string name;
        Frame* first;
        Frame* last;
        vector<Joint*> joints;
        Q q_home;
    };

    struct CompositeDeviceStruct
    {
        CompositeDeviceStruct(
            const string& name,
            const vector<string>& device_names,
            const Q& q_home)
            :
            name(name),
            device_names(device_names),
            q_home(q_home)
        {
            if (device_names.empty())
                RW_THROW(
                    "Can't construct composite device "
                    << quote(name)
                    << " containing no subdevices.");
        }

        string name;
        vector<string> device_names;
        Q q_home;
    };

    struct WorkCellStruct
    {
        vector<DeviceStruct> devices;
        vector<CompositeDeviceStruct> composite_devices;
        StateStructure* tree;
        Frame* world_frame;

        // The most recently added frame.
        Frame* last_frame;

        WorkCellStruct() :
            tree(new StateStructure()),
            world_frame(0),
            last_frame(0)
        {}
    };

    void link(WorkCellStruct& workcell, Frame& frame, Frame& parent, bool isDaf)
    {
        if (isDaf) {
            workcell.tree->addDAF(&frame, &parent);
        } else {
            workcell.tree->addFrame(&frame, &parent);
        }
    }

    Joint* makeJoint(
        const Tag& tag,
        const string& frame_name,
        const Transform3D<>& transform)
    {
        if (tagPropRevolute().has(tag))
			return new RevoluteJoint(frame_name, transform);
        else if (tagPropPrismatic().has(tag))
			return new PrismaticJoint(frame_name, transform);
        else if (tagPropFixed().has(tag))
            return new VirtualJoint(frame_name, transform, 1);
        else {
            // We currently don't support other types of joints.
            RW_THROW(
                msgHeader(tag)
                << "The frame is neither revolute nor prismatic.");
            return 0;
        }
    }

    /**
       Build a frame for the attributes of \a tag and a frame
       transform of \a transform.

       If the frame is an active joint, it is added to \a activeJoints.
    */
    Frame* makeTagFrameHelper(
        Frame* parent,
        const Tag& tag,
        const string& frame_name,
        const Transform3D<>& transform,
        vector<Joint*>& activeJoints,
        const Prefix& prefix)
    {
        // This is kind of neat: If the DAF tag has been set, we simply set the
        // parent to NULL, and things will then just work.
        if (tagPropDAF().has(tag)) parent = 0;

        // If we have an active joint:
        if (tagPropActiveJoint().has(tag)) {

            Q minPos(1, -DBL_MAX);
            Q maxPos(1,DBL_MAX);
            if (tagPropJointPosLimit().has(tag)) {
                minPos(0) = tagPropJointPosLimit().get(tag, 0);
                maxPos(0) = tagPropJointPosLimit().get(tag, 1);
            }

            double maxVel = DBL_MAX;
            if (tagPropJointVelLimit().has(tag))
                maxVel = tagPropJointVelLimit().get(tag, 0);

            double maxAcc = DBL_MAX;
            if (tagPropJointAccLimit().has(tag))
                maxAcc = tagPropJointAccLimit().get(tag, 0);

            // If it is a revolute joint:
            if (tagPropRevolute().has(tag)) {
                minPos *= Deg2Rad;
                maxPos *= Deg2Rad;
                maxVel *= Deg2Rad;
                maxAcc *= Deg2Rad;
            }

            Joint *joint = makeJoint(tag, frame_name, transform);

            joint->setBounds(std::make_pair(minPos, maxPos));
            joint->setMaxVelocity(Q(1,maxVel));
            joint->setMaxAcceleration(Q(1,maxAcc));

            activeJoints.push_back(joint);

            return joint;
        }

        // If we have a movable joint:
        if (
            tagPropMovable().has(tag) ||
            tagPropMoveable().has(tag))
        {
            Frame* frame = new MovableFrame(frame_name);

            // This is a hack, but we need it now:
            movableFrameTransformAccessor().set(*frame, transform);
            // When we have constructed our state, we traverse the movable
            // frames and assign the initial transform appropriately.

            return frame;
        }

        // If we have a passive joint:
        const bool isPassiveRevolute = tagPropPassiveRevolute().has(tag);
        const bool isPassivePrismatic = tagPropPassivePrismatic().has(tag);
        if (isPassiveRevolute || isPassivePrismatic) {
            const TagProperty<string>& prop =
                isPassiveRevolute ?
                tagPropPassiveRevolute() :
                tagPropPassivePrismatic();

            const string ownerName = prop.getT<string>(tag, 0);
            const double scale = prop.getT<double>(tag, 1);
            const double offset = prop.getT<double>(tag, 2);

            // Lookup the owner.
            Frame* ownerFrame = prefix.resolve(ownerName);
            if (!ownerFrame)
                RW_THROW(
                    msgHeader(tag, prop.key())
                    << "No frame for context "
                    << quote(prefix.getPrefixDescription())
                    << " of name "
                    << quote(ownerName));

            // The owner should be a joint.
            Joint* owner = dynamic_cast<Joint*>(ownerFrame);
            if (!owner) {
                RW_THROW(
                    msgHeader(tag, prop.key())
                    << "The controlling frame "
                    << quote(ownerName)
                    << " must be a Joint.");
                // We don't support e.g. passive joints controlling other
                // passive joints.
            }

            // The passive joint:
            if (isPassiveRevolute)
                return new DependentRevoluteJoint(frame_name, transform, owner, scale, offset);
            else
                return new DependentPrismaticJoint(frame_name, transform, owner, scale, offset);
        }

        // All other joints are considered fixed:
        return new FixedFrame(frame_name, transform);
    }

    /**
       Build a single frame for \a tag.

       If the frame is an active joint, it is added to \a activeJoints.

       Property values of the frame are assigned, but the link to the parent
       frame is not made and no loading of referenced devices is done.
    */
    Frame* makeTagFrame(
        Frame* parent,
        const Tag& tag,
        vector<Joint*>& activeJoints,
        const Prefix& prefix)
    {
        // Set the name of the frame.
        const string& frame_name = prefix.getFrameName(tag.getName());

        // The frame transform.
        const Transform3D<>& frame_transform = getTransform(tag);

        // Make a single frame for the tag attributes.
        Frame* tag_frame = makeTagFrameHelper(
            parent,
            tag,
            frame_name,
            frame_transform,
            activeJoints,
            prefix);

        // Copy the attributes to this frame.
        setTag(*tag_frame, tag);

        // Return the frame.
        return tag_frame;
    }

    /**
       This function means that the WORLD tag is optional: A WORLD frame will be
       generated and registered no matter what.
    */
    void makeWorldFrame(
        WorkCellStruct& workcell,
        const Prefix& prefix,
        vector<Tag>& tags)
    {
        RW_ASSERT(!workcell.world_frame);

        // NB: We will have to build a check into attachFrame() so that the
        // world frame can't be treated as a DAF...

        Frame* world = workcell.tree->getRoot();

        // The default tag to use for the world frame.
        Tag world_tag(world->getName(), ""); // "" means no file.

        if (!tags.empty() && tags.front().getName() == world->getName()) {
            // Use this tag for the world instead.
            world_tag = tags.front();

            // Erase this tag now that we have processed it.
            tags.erase(tags.begin());
        }

        // Set the world frame tag.
        setTag(*world, world_tag);

        // We store the world frame.
        workcell.world_frame = world;

        // We register the world frame.
        prefix.insert(world);
    }

    Q getOptionalDeviceHomePos(const Tag& tag)
    {
        if (tagPropDeviceHomePos().has(tag))
            return tagPropDeviceHomePos().get(tag, 0);
        else
            return Q();
    }

    Q getOptionalDeviceHomePos(const Frame& frame)
    {
        return getOptionalDeviceHomePos(getTag(frame));
    }

    // 'frame' is the current frame containing the CompositeDevice attribute and
    // 'prefix' is the current scope.
    CompositeDeviceStruct makeCompositeDeviceStruct(
        const Frame& frame, const Prefix& prefix)
    {
        const vector<string> raw_names =
            getAsVector(tagPropCompositeDevice(), frame);

        // We have to place the names in the relative scope.
        vector<string> names;

        typedef vector<string>::const_iterator I;
        for (I p = raw_names.begin(); p != raw_names.end(); ++p) {
            names.push_back(prefix.getFrameName(*p));
        }

        return CompositeDeviceStruct(
            frame.getName(), names, getOptionalDeviceHomePos(frame));
    }

    void loadWorkCellStruct(
        const string& filename,
        Frame* root,
        const Prefix& prefix,
        WorkCellStruct& workcell,
        const Q& q_home)
    {
        // Read the tag file. We take a copy here so that makeWorldFrame() can
        // erase the first element.
        vector<Tag> tags = loadTagFile(filename);

        // Make sure we have a world frame.
        if (!workcell.world_frame) {
            makeWorldFrame(workcell, prefix, tags);
            RW_ASSERT(!root);
            root = workcell.world_frame;
        }

        // Collect the joints here.
        vector<Joint*> activeJoints;

        typedef vector<Tag>::const_iterator I;
        for (I p = tags.begin(); p != tags.end(); ++p) {
            const Tag& tag = *p;

            // The parent of the frame. All frames have a parent at load time,
            // even a DAF. It is just that DAFs have a parent assigned via
            // Tree::setDafParent().

            Frame* parent;

            const string& referenceFrame = getReferenceFrame(tag);

            if (referenceFrame == "") {
                parent = root;
            } else {
                parent = prefix.resolve(referenceFrame);
                if (!parent)
                    RW_THROW(
                        msgHeader(tag, tagPropReferenceFrame().key())
                        << "No frame for context "
                        << quote(prefix.getPrefixDescription())
                        << " of name "
                        << quote(referenceFrame));
            }

            // A tag.
            Frame* tag_frame = makeTagFrame(
                parent, tag, activeJoints, prefix);

            bool isDAF = false;
            if( tagPropDAF().has(tag) )
                isDAF = true;

            // Link the frame to the tree and also to the parent if it is a DAF
            // that does not yet have a parent.
            link(workcell, *tag_frame, *parent, isDAF);

            // This is the latest added active joint.
            if (tagPropLink().has(tag) || tagPropActiveJoint().has(tag))
                workcell.last_frame = tag_frame;

            // Register in the name to frame map.
            prefix.insert(tag_frame);

            // Load any linked device.
            if (tagPropDevice().has(tag)) {

                // Here we resolve the file name also.
                const string& deviceFile =
                    getFileNameOfTag(tag, tagPropDevice().get(tag, 0));

                // The device is loaded with a parent of tag_frame.
                loadWorkCellStruct(
                    deviceFile,
                    tag_frame,
                    // Enter a new scope.
                    prefix.enter(tag.getName()),
                    workcell,
                    getOptionalDeviceHomePos(tag));
            }

            // Or load a composite device instead.
            else if (tagPropCompositeDevice().has(tag)) {
                workcell.composite_devices.push_back(
                    makeCompositeDeviceStruct(*tag_frame, prefix));
            }
        }

        if (!activeJoints.empty()) {
            workcell.devices.push_back(
                DeviceStruct(
                    // The NameOfData field is not used as the device name in
                    // TUL, because the name is not unique. My cell for Grundfos
                    // for example uses the same device file in three different
                    // places in the workcell.
                    root->getName(),
                    root, // NB: This we define as the start of the device.
                    workcell.last_frame,
                    activeJoints,
                    q_home));
        }
    }
}

namespace
{
    // The initial work cell state when considering HomePos values.
    State homeState(Frame* world, StateStructure* tree)
    {
        State state = tree->getDefaultState();
        const vector<Frame*>& frames = Kinematics::findAllFrames(world, state);
        typedef vector<Frame*>::const_iterator I;
        for (I p = frames.begin(); p != frames.end(); ++p) {
            const Frame& frame = **p;
            if (tagPropJointHomePos().has(frame)) {

                // Here we should be using a Q value instead so that things
                // would work also for more advanced joints.
                const double val =
                    tagPropJointHomePos().get(frame, 0);

                const int dof = frame.getDOF();
                if (dof != 1) {
                    RW_THROW(
                        msgHeader(getTag(frame), tagPropJointHomePos().key())
                        << "DOF is "
                        << dof
                        << ". Only a DOF of one is supported.");
                }
                // Assign the value.
                frame.setData(state, &val);
            }
        }
        return state;
    }

    // The initial work cell state with configurations for the movable frames
    // initialized also.
    State movableFrameState(Frame* world, const State& initial_state)
    {
        State state = initial_state;
        const vector<Frame*>& frames = Kinematics::findAllFrames(world, state);
        typedef vector<Frame*>::const_iterator I;
        for (I p = frames.begin(); p != frames.end(); ++p) {
            MovableFrame* movable = dynamic_cast<MovableFrame*>(*p);
            if (movable) {
                const Transform3D<> transform =
                    movableFrameTransformAccessor().get(*movable);
                movableFrameTransformAccessor().erase(*movable);

                movable->setTransform(transform, state);
            }
        }
        return state;
    }

    // The initial work cell state.
    State initialState(Frame* world, StateStructure* tree)
    {
        // Here we combine all transformations that are needed for the work cell
        // state to be correct.
        return movableFrameState(
            world,
            homeState(world, tree));
    }

    // By the way: The use of JointHomePos values is nice, but it is nowhere
    // enough. For example it would be nice to be able to specify the initial
    // joint values in the Device tag where the device is loaded. That way the
    // same device can be loaded with different joint home positions.

    // Convert the temporary DeviceStruct values to real devices.
	vector<Device::Ptr> makeDevices(
        const vector<DeviceStruct>& devices,
        const State& state)
    {
		vector<Device::Ptr> result;
        typedef vector<DeviceStruct>::const_iterator I;
        for (I p = devices.begin(); p != devices.end(); ++p) {
            result.push_back(ownedPtr(new SerialDevice(
                    p->first,
                    p->last,
                    p->name,
                    state))
              );
        }
        return result;
    }

	Device::Ptr findDevice(
        const std::string& name,
		const vector<Device::Ptr>& devices)
    {
		BOOST_FOREACH(Device::Ptr device, devices)
            if (device->getName() == name)
                return device;
        return NULL;
    }

	Device::Ptr makeCompositeDevice(
        const Prefix& prefix,
		const vector<Device::Ptr>& standard_devices,
		const vector<Device::Ptr>& composite_devices,
        const CompositeDeviceStruct& composite,
        const State& state)
    {
		vector<Device::Ptr> devices;

        typedef vector<string>::const_iterator I;
        for (I p = composite.device_names.begin();
             p != composite.device_names.end();
             ++p)
        {
			Device::Ptr device = findDevice(*p, standard_devices);
            if (!device) device = findDevice(*p, composite_devices);
            if (!device) {
                RW_THROW(
                    "No device named "
                    << quote(*p)
                    << " for composite device "
                    << quote(composite.name));
            }
            devices.push_back(device);
        }

        Frame* base = prefix.resolve(composite.name);

        // And these really _are_ assertions and not exceptions.
        RW_ASSERT(base);
        RW_ASSERT(!devices.empty());

        // The end defaults to the end of the first device.
        Frame* end = devices.front()->getEnd();

        return new CompositeDevice(base, devices, end, composite.name, state);
    }

	vector<Device::Ptr> makeCompositeDevices(
        const Prefix& prefix,
		const vector<Device::Ptr>& standard_devices,
        const vector<CompositeDeviceStruct>& composite_devices,
        const State& state)
    {
		vector<Device::Ptr> result;

        typedef vector<CompositeDeviceStruct>::const_iterator I;
        for (I p = composite_devices.begin(); p != composite_devices.end(); ++p)
            result.push_back(
                makeCompositeDevice(
                    prefix,
                    standard_devices,
                    result, // Search also in the composite devices.
                    *p,
                    state));
        return result;
    }

    void addDevices(
		const vector<Device::Ptr>& devices,
        WorkCell& workcell)
    {
		typedef vector<Device::Ptr>::const_iterator I;
        for (I p = devices.begin(); p != devices.end(); ++p)
            workcell.addDevice(*p);
    }

    void setOptionalDeviceHomePos(
        const Device& device, const Q& q_home, State& state)
    {
        if (q_home.size() == 0) {}
        else if (q_home.size() != device.getDOF()) {
            RW_THROW(
                "'DeviceHomePos' configuration with "
                << q_home.size()
                << " DOF given for device "
                << quote(device.getName())
                << " with "
                << device.getDOF()
                << " DOF.");
        } else {
            device.setQ(q_home, state);
        }
    }
}

WorkCell::Ptr TULLoader::loadWorkCell(const std::string& filename) {
	return load(filename);
}

WorkCell::Ptr TULLoader::load(const string& filename)
{
    WorkCellStruct workcell;

    Prefix prefix;

    loadWorkCellStruct(
        filename, // The resolved file to load.
        0, // The currently unknown parent frame (the world frame).
        prefix, // Initially we use the empty prefix.
        workcell,
        Q());

    // Initial work cell state.
    State state = initialState(workcell.world_frame, workcell.tree);

    // Some extra initialization.
    initProperties(workcell.world_frame, state);

    // Construct the devices.
	vector<Device::Ptr> devices = makeDevices(workcell.devices, state);

    // Construct the composite devices.
	vector<Device::Ptr> composite_devices =
        makeCompositeDevices(
            prefix, devices, workcell.composite_devices, state);

    // Assign q_home values for the devices.
    for (int i = 0; i < (int)workcell.devices.size(); i++) {
        setOptionalDeviceHomePos(
            *devices[i],
            workcell.devices[i].q_home,
            state);
    }

    // Assign q_home values for the composite devices.
    for (int i = 0; i < (int)workcell.composite_devices.size(); i++) {
        setOptionalDeviceHomePos(
            *composite_devices[i],
            workcell.composite_devices[i].q_home,
            state);
    }

    // We know the state and the world frame, so we can create our workcell.
    workcell.tree->setDefaultState(state);
	WorkCell::Ptr result = ownedPtr(new WorkCell(ownedPtr( workcell.tree), filename));

    // Add the devices to the workcell.
    addDevices(devices, *result.get());

    // Add the composite devices to the workcell.
    addDevices(composite_devices, *result.get());

    // Some more initialization. It doesn't matter when this is done.
    initCollisionSetup(*result);

    // And that is all folks.
    return result;
}
