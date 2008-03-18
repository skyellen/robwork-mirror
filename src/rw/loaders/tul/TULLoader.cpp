/*********************************************************************
 * RobWork Version 0.2
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
#include "TULLoader.hpp"

#include "Tag.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Constants.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/JointDevice.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/CompositeDevice.hpp>
#include <rw/models/Accessor.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/FixedJoint.hpp>
#include <rw/models/PassiveRevoluteFrame.hpp>
#include <rw/models/PassivePrismaticFrame.hpp>

#include <rw/loaders/colsetup/CollisionSetupLoader.hpp>
#include <rw/proximity/CollisionSetup.hpp>

#include <rw/common/IOUtil.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/macros.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FrameType.hpp>
#include <rw/kinematics/FramePropertyImpl.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/Tree.hpp>
#include <rw/common/Property.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include <string>
#include <map>
#include <cfloat>
#include <fstream>
#include <vector>
#include <map>
#include <stack>
#include <list>

using namespace rw;
using namespace rw::math;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::loaders;

using namespace std;

//----------------------------------------------------------------------
// Tag properties

// Tag properties are loaded from a file and are henceforth read-only (or least
// with respect to the structure and types).

namespace
{
    string quote(const string& str) { return StringUtil::Quote(str); }

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
            return Rotation3D<>::Identity();
        }
    }

    // We default to the unit transform.
    Transform3D<> getTransform(const Tag& tag)
    {
        return Transform3D<>(
            getPosition(tag),
            getRotation(tag));
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
        const string& filename = StringUtil::ReplaceBackslash(file);

        if (StringUtil::IsAbsoluteFileName(filename))
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
            StringUtil::GetDirectoryName(tag.getFile()), file);
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
            const int maxCnt = _path.size() + 1;
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

    CollisionSetup defaultCollisionSetup(const WorkCell& workcell)
    {
        // We build a list of frames
        std::list<Frame*> frameList;
        std::stack<Frame*> frameStack;
        frameStack.push(workcell.getWorldFrame());
        while(0 != frameStack.size()){
            Frame* frame = frameStack.top();
            frameStack.pop();

            for (Frame::iterator it = frame->getChildren().first;
                 it != frame->getChildren().second;
                 ++it)
            {
                frameStack.push(&*it);
                frameList.push_back(&*it);
            }
        }

        // Add frames to exclude list
        ProximityPairList excludeList;
        std::list<Frame*>::reverse_iterator rit;
        std::list<Frame*>::iterator it;
        for(rit=frameList.rbegin(); rit!=frameList.rend();rit++ ){

            for(it = frameList.begin(); (*it) != (*rit); it++){

                // Do not check a child against a parent geometry
                Frame* parent1 = (*it)->getParent(); // Link N
                Frame* parent2 = (*rit)->getParent(); // Link N+1

                if(parent1 && parent2 && parent2->getParent()!=NULL){
                    if(parent2->getParent() == parent1){
                        excludeList.push_back(
                            ProximityPair((*rit)->getName(), (*it)->getName()));
                    }
                }

                // Do not check a child agains its parent
                if((*it)->getParent() == (*rit) || (*rit)->getParent() == (*it) ){
                    excludeList.push_back(
                        ProximityPair((*rit)->getName(), (*it)->getName()));
                }
            }
        }

        return CollisionSetup(excludeList);
    }

    /**
     * @brief Build a collision setup for a workcell.
     */
    CollisionSetup makeCollisionSetup(
        const WorkCell& workcell)
    {
        const vector<Frame*>& frames = Kinematics::FindAllFrames(
            workcell.getWorldFrame(),
            workcell.getDefaultState());

        const ProximityPairList empty_list;
        CollisionSetup result(empty_list);

        bool foundSetup = false;

        typedef vector<Frame*>::const_iterator I;
        for (I p = frames.begin(); p != frames.end(); ++p) {
            const Frame& frame = **p;

            if (tagPropCollisionSetup().has(frame)) {
                foundSetup = true;
                const string& setupFile =
                    tagPropCollisionSetup().get(frame, 0);

                // Remember to resolve the file name.
                const string& file = getFileNameOfFrame(frame, setupFile);

                // The string to prefix the frame values.
                const string& prefix = getPrefix(frame);

                // Load the file and merge into result.
                result =
                    CollisionSetup::Merge(
                        result,
                        CollisionSetupLoader::Load(prefix, file));
            }
        }

        // If no external collision setup is provided, we try to construct one
        if (!foundSetup) {
            RW_WARN("No collision setup given. Building default exclude list.");
            return defaultCollisionSetup(workcell);
        }

        return result;
    }

    void addCollisionSetupProperty(WorkCell& workcell)
    {
        const CollisionSetup& setup = makeCollisionSetup(workcell);
        Accessor::CollisionSetup().set(*workcell.getWorldFrame(), setup);
    }

    void addDrawableIDProperty(Frame& frame)
    {
        // Insert GeoID as Drawable
        if (tagPropDrawableID().has(frame)) {
            const string& geo = tagPropDrawableID().get(frame, 0);
            if (geo[0] != '#') {
                // Remember to resolve the file name.
                const string& file = getFileNameOfFrame(frame, geo);
                Accessor::DrawableID().set(frame, file);
            } else {
                Accessor::DrawableID().set(frame, geo);
            }
        } else if (tagPropGeoID().has(frame)) {
            const string& geo = tagPropGeoID().get(frame, 0);

            if (geo[0] != '#') {
                // Remember to resolve the file name.
                const string& file = getFileNameOfFrame(frame, geo);
                Accessor::DrawableID().set(frame, file);
            } else {
                Accessor::DrawableID().set(frame, geo);
            }
        }
    }

    void addCollisionModelIDProperty(Frame& frame)
    {
        // Insert GeoID as Collision Model
        if (tagPropCollisionModelID().has(frame)) {
            const string& geo = tagPropCollisionModelID().get(frame, 0);

            if (geo[0] != '#') {
                // Remember to resolve the file name.
                const string& file = getFileNameOfFrame(frame, geo);
                Accessor::CollisionModelID().set(frame, file);
            } else {
                Accessor::CollisionModelID().set(frame, geo);
            }
        } else if (tagPropGeoID().has(frame)) {
            const string& geo = tagPropGeoID().get(frame, 0);

            if (geo[0] != '#') {
                // Remember to resolve the file name.
                const string& file = getFileNameOfFrame(frame, geo);
                Accessor::CollisionModelID().set(frame, file);
            } else {
                Accessor::CollisionModelID().set(frame, geo);
            }
        }
    }

    void addGeoScaleProperty(Frame& frame)
    {
        if (tagPropGeoScale().has(frame)) {
            const double scale = tagPropGeoScale().get(frame, 0);
            Accessor::GeoScale().set(frame, scale);
        }
    }

    void addFrameTypeProperty(Frame& frame)
    {
        if (dynamic_cast<RevoluteJoint*>(&frame)) {
            Accessor::FrameType().set(frame, FrameType::RevoluteJoint);
        } else if (dynamic_cast<PrismaticJoint*>(&frame)) {
            Accessor::FrameType().set(frame, FrameType::PrismaticJoint);
        } else if (dynamic_cast<FixedFrame*>(&frame)) {
            Accessor::FrameType().set(frame, FrameType::FixedFrame);
        } else if (dynamic_cast<MovableFrame*>(&frame)) {
            Accessor::FrameType().set(frame, FrameType::MovableFrame);
        } else {
            Accessor::FrameType().set(frame, FrameType::Unknown);
        }
    }

    void addActiveJointProperty(Frame& frame)
    {
        if (tagPropActiveJoint().has(frame))
            Accessor::ActiveJoint().set(frame, true);

        // Otherwise do _not_ set a value: We want to use
        // Accessor::ActiveJoint().has() rather than get() to check if the joint
        // is active. The 'true' value is a dummy value.
    }

    void addAllProperties(Frame* frame)
    {
        addFrameTypeProperty(*frame);
        addGeoScaleProperty(*frame);
        //        addGeoIDProperty(*frame);
        addDrawableIDProperty(*frame);
        addCollisionModelIDProperty(*frame);
        addActiveJointProperty(*frame);
        // And we don't add any CollisionSetup property, currently, as that is
        // done only for the root. (See addCollisionSetupProperty()).
    }

    // Assign all special properties.
    void initProperties(Frame* world, const State& state)
    {
        const vector<Frame*>& frames = Kinematics::FindAllFrames(world, state);
        std::for_each(frames.begin(), frames.end(), addAllProperties);
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
        boost::shared_ptr<Tree> tree;
        Frame* world_frame;

        // The most recently added frame.
        Frame* last_frame;

        WorkCellStruct() :
            tree(new Tree()),
            world_frame(0),
            last_frame(0)
        {}
    };

    void link(WorkCellStruct& workcell, Frame& frame, Frame& parent)
    {
        workcell.tree->addFrame(&frame);

        // If it is a DAF then link the frame to 'parent'.
        if (!frame.getParent()) {
            workcell.tree->setDafParent(frame, parent);
        }
    }

    Joint* makeJoint(
        Frame* parent,
        const Tag& tag,
        const string& frame_name,
        const Transform3D<>& transform)
    {
        if (tagPropRevolute().has(tag))
            return new RevoluteJoint(
                parent, frame_name, transform);
        else if (tagPropPrismatic().has(tag))
            return new PrismaticJoint(
                parent, frame_name, transform);
        else if (tagPropFixed().has(tag))
            return new FixedJoint(
                parent, frame_name, transform);
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

            double minPos = -DBL_MAX;
            double maxPos = DBL_MAX;
            if (tagPropJointPosLimit().has(tag)) {
                minPos = tagPropJointPosLimit().get(tag, 0);
                maxPos = tagPropJointPosLimit().get(tag, 1);
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

            Joint *joint = makeJoint(parent, tag, frame_name, transform);

            joint->setBounds(std::make_pair(minPos, maxPos));
            joint->setMaxVelocity(maxVel);
            joint->setMaxAcceleration(maxAcc);

            activeJoints.push_back(joint);

            return joint;
        }

        // If we have a movable joint:
        if (
            tagPropMovable().has(tag) ||
            tagPropMoveable().has(tag))
        {
            Frame* frame = new MovableFrame(parent, frame_name);

            // This is a hack, but we need it now:
            movableFrameTransformAccessor().set(
                *frame, transform);
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
                return new PassiveRevoluteFrame(
                    parent, frame_name, transform, owner, scale, offset);
            else
                return new PassivePrismaticFrame(
                    parent, frame_name, transform, owner, scale, offset);
        }

        // All other joints are considered fixed:
        return new FixedFrame(parent, frame_name, transform);
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
        Frame* world = new FixedFrame(0, "WORLD", Transform3D<>::Identity());

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

        // Ownership of the world frame is taken.
        workcell.tree->addFrame(world);

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

            // Link the frame to the tree and also to the parent if it is a DAF
            // that does not yet have a parent.
            link(workcell, *tag_frame, *parent);

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
    State homeState(
        Frame* world, boost::shared_ptr<Tree> tree)
    {
        State state(tree);
        const vector<Frame*>& frames = Kinematics::FindAllFrames(world, state);

        typedef vector<Frame*>::const_iterator I;
        for (I p = frames.begin(); p != frames.end(); ++p) {
            const Frame& frame = **p;
            if (tagPropJointHomePos().has(frame)) {

                // Here we should be using a Q value instead so that things
                // would work also for more advanced joints.
                const double val =
                    tagPropJointHomePos().get(frame, 0);

                const int dof = frame.getDof();
                if (dof != 1) {
                    RW_THROW(
                        msgHeader(getTag(frame), tagPropJointHomePos().key())
                        << "DOF is "
                        << dof
                        << ". Only a DOF of one is supported.");
                }
                // Assign the value.
                frame.setQ(state, &val);
            }
        }

        return state;
    }

    // The initial work cell state with configurations for the movable frames
    // initialized also.
    State movableFrameState(Frame* world, const State& initial_state)
    {
        State state = initial_state;
        const vector<Frame*>& frames = Kinematics::FindAllFrames(world, state);
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
    State initialState(Frame* world, boost::shared_ptr<Tree> tree)
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
    vector<Device*> makeDevices(
        const vector<DeviceStruct>& devices,
        const State& state)
    {
        vector<Device*> result;
        typedef vector<DeviceStruct>::const_iterator I;
        for (I p = devices.begin(); p != devices.end(); ++p) {
            result.push_back(

                new SerialDevice(
                    p->first,
                    p->last,
                    p->name,
                    state)

                /*
                  In my opinion each TUL device should be of type JointDevice as
                  shown here. If you want a SerialDevice (whatever that is) you
                  should use a specific SerialDevice command or specifier for
                  that.

                new JointDevice(
                    p->name,
                    p->first,
                    p->last,
                    p->joints,
                    state)
                */
                );
        }
        return result;
    }

    Device* findDevice(
        const std::string& name,
        const vector<Device*>& devices)
    {
        BOOST_FOREACH(Device* device, devices)
            if (device->getName() == name)
                return device;
        return NULL;
    }

    Device* makeCompositeDevice(
        const Prefix& prefix,
        const vector<Device*>& standard_devices,
        const vector<Device*>& composite_devices,
        const CompositeDeviceStruct& composite,
        const State& state)
    {
        vector<Device*> devices;

        typedef vector<string>::const_iterator I;
        for (I p = composite.device_names.begin();
             p != composite.device_names.end();
             ++p)
        {
            Device* device = findDevice(*p, standard_devices);
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

    vector<Device*> makeCompositeDevices(
        const Prefix& prefix,
        const vector<Device*>& standard_devices,
        const vector<CompositeDeviceStruct>& composite_devices,
        const State& state)
    {
        vector<Device*> result;

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
        const vector<Device*>& devices,
        WorkCell& workcell)
    {
        typedef vector<Device*>::const_iterator I;
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

std::auto_ptr<WorkCell> TULLoader::LoadTUL(const string& filename)
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
    State state =
        initialState(workcell.world_frame, workcell.tree);

    // Some extra initialization.
    initProperties(workcell.world_frame, state);

    // Construct the devices.
    vector<Device*> devices = makeDevices(workcell.devices, state);

    // Construct the composite devices.
    vector<Device*> composite_devices =
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
    std::auto_ptr<WorkCell> result(
        new WorkCell(
            workcell.world_frame,
            state,
            filename));

    // Add the devices to the workcell.
    addDevices(devices, *result);

    // Add the composite devices to the workcell.
    addDevices(composite_devices, *result);

    // Some more initialization. It doesn't matter when this is done.
    initCollisionSetup(*result);

    // And that is all folks.
    return result;
}
