%module rw

%{
#include <rwlibs/lua/RemoteTypes.hpp>
#include <rw/common/Ptr.hpp>
using namespace rwlibs::rwr;
using namespace rw::common;
%}

%include <std_string.i>
%include <std_vector.i>

%include "carrays.i"
%array_class(double, doubleArray);

%constant double Pi = rw::math::Pi;
%constant double Inch2Meter = rw::math::Inch2Meter;
%constant double Meter2Inch = rw::math::Meter2Inch;
%constant double Deg2Rad = rw::math::Deg2Rad;
%constant double Rad2Deg = rw::math::Rad2Deg;

/********************************************
 * COMMON
 */


template<class T> class Ptr {
public:

    T *operator->() const;
};




/**************************
 * MATH
 */

class Q
{
public:
    // first we define functions that are native to Q
	Q();
	%feature("autodoc", "1")
    Q(int n, double vals[n]);
    int size() const;

    //
    %rename(elem) operator[];
    double& operator[](unsigned int i) ;

    const Q operator-() const;
    Q operator-(const Q& b) const;
    Q operator+(const Q& b) const;
    Q operator*(double s) const;
    Q operator/(double s) const;
    double norm2();
    double norm1();
    double normInf();

    %extend {
        std::string __tostring() { return toString(*$self); }
        double __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
    };

};



class Vector3D
{
public:
    Vector3D();
    %feature("autodoc", "1")
    Vector3D(double x,double y, double z);
    size_t size() const;
    Vector3D operator*(double scale) const;
    Vector3D operator+(const Vector3D& other) const;
    Vector3D operator-(const Vector3D& other) const;
    bool operator==(const Vector3D& q);

    double norm2();
    double norm1();
    double normInf();

    //double& operator[](unsigned int i) ;
    %rename(elem) operator[];

    %extend {
        std::string __tostring() { return toString<Vector3D>(*$self); }
        double __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
    };

};


class Rotation3D
{
public:
    // Lua methods:
    Rotation3D();
    %feature("autodoc", "1")
    Rotation3D(double v0,double v1,double v2,
    			double v3,double v4,double v5,
    			double v6,double v7,double v8);
    			
    explicit Rotation3D(const Rotation3D& R);

    Rotation3D operator*(const Rotation3D& other) const;
    Vector3D operator*(const Vector3D& vec) const;
    //Rotation3D inverse() const;

    static const Rotation3D& identity();
    static Rotation3D skew(const Vector3D& v);
    bool equal(const Rotation3D& rot, double precision);

    //EAA operator*(const EAA& other) const;

    bool operator==(const Rotation3D &rhs) const;
    //double& operator[](unsigned int i) ;

    // std::string __tostring() const;
};

%extend Rotation3D {
    std::string __tostring() { return toString<Rotation3D>(*$self); }

   double __getitem__(int x,int y)const {
       return (*$self)(x,y);
   }
   void __setitem__(int x, int y, double d)
   {
    (*$self)(x,y) = d;
    }
};


//Rotation3D inverse(const Rotation3D& val);

class EAA
{
public:
    // Lua methods:
    EAA();
    %feature("autodoc", "1")
    EAA(const EAA& eaa);
    %feature("autodoc", "1")
    EAA(const Rotation3D& rot);
    %feature("autodoc", "1")
    EAA(const Vector3D& axis, double angle);
    %feature("autodoc", "1")
    EAA(double thetakx, double thetaky, double thetakz);
    %feature("autodoc", "1")
    EAA(const Vector3D& v1, const Vector3D& v2);

    double angle() const;
    Vector3D axis() const;

    //const double& operator[](unsigned int i) const;
	//double& operator[](unsigned int i);
	%rename(elem) operator[];

    //EAA operator*(const Rotation3D& other) const;

    Rotation3D toRotation3D() const;

    //bool operator==(const EAA &rhs) const;
    // std::string __tostring() const;
};

%extend EAA {
    std::string __tostring() { return toString<EAA>(*$self); }
   double __getitem__(int i)const {
       return (*$self)[i];
   }
   void __setitem__(int i,double d)
   {
    (*$self)[i] = d;
    }

};


class RPY
{
public:
    // Lua methods:
    RPY();
    RPY(const RPY& eaa);
    RPY(const Rotation3D& rot);
    RPY(double roll, double pitch, double yaw);
    Rotation3D toRotation3D() const;
    %extend {
        std::string __tostring() { return toString<RPY>(*$self); }
        double __getitem__(int i)const { return (*$self)(i); }
        void __setitem__(int i,double d){ (*$self)(i) = d; }
    };
};


class Quaternion
{
public:
    // Lua methods:
    Quaternion();
    Quaternion(const Quaternion& eaa);
    Quaternion(const Rotation3D& rot);
    Quaternion operator*(double s);

    void normalize();

    Rotation3D toRotation3D() const;
    Quaternion slerp(const Quaternion& v, const double t) const;

    double getQx() const;
    double getQy() const;
    double getQz() const;
    double getQw() const;
    %extend {
        std::string __tostring() { return toString<Quaternion>(*$self); }
        double __getitem__(int i)const { return (*$self)(i); }
        void __setitem__(int i,double d) { (*$self)(i) = d; }
    };

};



//Quaternion operator*(double s, const Quaternion& v);

class Transform3D {
public:
	Transform3D();
    Transform3D(const Transform3D& t3d);
    Transform3D(const Vector3D& position,const Rotation3D& rotation);

    Transform3D operator*(const Transform3D& other) const;
    Vector3D operator*(const Vector3D& other) const;

    static Transform3D DH(double alpha, double a, double d, double theta);
    static Transform3D craigDH(double alpha, double a, double d, double theta);
	
    Vector3D& P();
    Rotation3D& R();

    %extend {
       std::string __tostring() { return toString<Transform3D>(*$self); }
       Transform3D inverse(){ return inverse(*$self); }
       //Transform3D inverse(const Transform3D& val);
    };
};


class Pose6D {
public:
	Pose6D(const Pose6D& p6d);
    Pose6D(const Vector3D& position,const EAA& rotation);
    Pose6D(const Transform3D& t3d);

    Transform3D toTransform3D();
    %extend {
       std::string __tostring() { return toString<Pose6D>(*$self); }
    };
};


class Jacobian
{
public:
    Jacobian(int m, int n);

    int size1() const ;
    int size2() const ;

    double& elem(int i, int j);

    %extend {
       std::string __tostring() { return toString<Jacobian>(*$self); }
    };
};



class VelocityScrew6D
{
public:
	VelocityScrew6D();
	VelocityScrew6D(const VelocityScrew6D& p6d);
    VelocityScrew6D(const Vector3D& position,const EAA& rotation);
    VelocityScrew6D(const Transform3D& t3d);

    // lua functions
    VelocityScrew6D operator*(double scale) const;
    VelocityScrew6D operator+(const VelocityScrew6D& other) const;
    VelocityScrew6D operator-(const VelocityScrew6D& other) const;
    //bool operator==(const VelocityScrew6D& q);

    double norm2();
    double norm1();
    double normInf();

    %extend {
       std::string __tostring() { return toString<VelocityScrew6D>(*$self); }
    };


    //Transform3D toTransform3D();
    // std::string __tostring() const;
};

/********************************************************************
// now we go on to geometry
*********************************************************************/


%template(GeometryDataPtr) Ptr<GeometryData>;
%template(TriMeshPtr) Ptr<TriMesh>;

class GeometryData {
    typedef enum {PlainTriMesh,
                  IdxTriMesh,
                  SpherePrim, BoxPrim, OBBPrim, AABBPrim,
                  LinePrim, PointPrim, PyramidPrim, ConePrim,
                  TrianglePrim, CylinderPrim, PlanePrim, RayPrim,
                  UserType} GeometryType;

    virtual GeometryType getType() const = 0;
    virtual Ptr<TriMesh> getTriMesh(bool forceCopy=true) = 0;
    static std::string toString(GeometryType type);
};



class TriMesh: public GeometryData {
public:
    virtual Triangle getTriangle(size_t idx) const = 0;
    virtual void getTriangle(size_t idx, Triangle& dst) const = 0;
    virtual void getTriangle(size_t idx, Trianglef& dst) const = 0;
    virtual size_t getSize() const = 0;
    virtual size_t size() const = 0;
    virtual Ptr<TriMesh> clone() const = 0;
    Ptr<TriMesh> getTriMesh(bool forceCopy=true);
    //Ptr<const TriMesh> getTriMesh(bool forceCopy=true) const;
};


class Primitive: public GeometryData {
public:
    Ptr<TriMesh> getTriMesh(bool forceCopy=true);
    virtual Ptr<TriMesh> createMesh(int resolution) const = 0;
    virtual Q getParameters() const = 0;
};

class Sphere: public Primitive {
public:
    //! constructor
    Sphere(const Q& initQ);
    Sphere(double radi):_radius(radi);
    double getRadius();
    TriMesh::Ptr createMesh(int resolution) const;
    Q getParameters() const;
    GeometryData::GeometryType getType() const;
};

class Box: public Primitive {
public:
    Box();
    Box(double x, double y, double z);
    Box(const Q& initQ);
    TriMesh::Ptr createMesh(int resolution) const;
    Q getParameters() const;
    GeometryType getType() const;
};

class Cone: public Primitive {
public:
    Cone(const Q& initQ);
    Cone(double height, double radiusTop, double radiusBot);
    double getHeight();
    double getTopRadius();
    double getBottomRadius();
    Ptr<TriMesh> createMesh(int resolution) const;
    Q getParameters() const;
    GeometryType getType() const;
};

class Plane: public Primitive {
public:
    Plane(const Q& q);
    Plane(const Vector3D& n, double d);
    Plane(const Vector3D& p1,
          const Vector3D& p2,
          const Vector3D& p3);

    Vector3D& normal();
    //const Vector3D& normal() const;
    double& d();
    //double d() const;
    double distance(const Vector3D& point);
    double refit( std::vector<Vector3D >& data );
    TriMesh::Ptr createMesh(int resolution) const ;
    Q getParameters() const;
    GeometryType getType() const{ return PlanePrim; };
};

class ConvexHull3D {
public:
    virtual void rebuild(const std::vector<Vector3D>& vertices) = 0;
    virtual bool isInside(const Vector3D& vertex) = 0;
    virtual double getMinDistInside(const Vector3D& vertex) = 0;
    virtual double getMinDistOutside(const Vector3D& vertex) = 0;
    virtual PlainTriMeshN1* toTriMesh() = 0;
};

%template(GeometryPtr) Ptr<Geometry>;
//typedef Ptr<Geometry> GeometryPtr;

class Geometry {
public:
    Geometry(Ptr<GeometryData> data, double scale=1.0);

    Geometry(Ptr<GeometryData> data,
             const Transform3D& t3d,
             double scale=1.0);

    double getScale() const;
    void setScale(double scale);
    void setTransform(const Transform3D& t3d);
    const Transform3D& getTransform() const;
    Ptr<GeometryData> getGeometryData();
    const Ptr<GeometryData> getGeometryData() const;
    void setGeometryData(Ptr<GeometryData> data);
    const std::string& getName() const;
    const std::string& getId() const;
    void setName(const std::string& name);
    void setId(const std::string& id);
    static Ptr<Geometry> makeSphere(double radi);
    static Ptr<Geometry> makeBox(double x, double y, double z);
    static Ptr<Geometry> makeCone(double height, double radiusTop, double radiusBot);
    static Ptr<Geometry> makeCylinder(float radius, float height);
};


class STLFile {
public:
    static void save(const TriMesh& mesh, const std::string& filename);
    static Ptr<PlainTriMeshN1f> load(const std::string& filename);
};

/**************************************************************************
 *  KINEMATICS
 *
 *************************************************************************/

class StateData {
protected:
    StateData(int size, const std::string& name);
public:
    const std::string& getName();
    int size() const;
    double* getData(State& state);
    void setData(State& state, const double* vals) const;
};

class Frame : public StateData
{
public:

    Transform3D getTransform(const State& state) const;
    const PropertyMap& getPropertyMap() const ;
    PropertyMap& getPropertyMap();
    int getDOF() const ;
    const Frame* getParent() const ;
    Frame* getParent() ;
    Frame* getParent(const State& state);
    const Frame* getParent(const State& state) const;
    const Frame* getDafParent(const State& state) const;
    Frame* getDafParent(const State& state);
/*
    typedef rw::common::ConcatVectorIterator<Frame> iterator;
    typedef rw::common::ConstConcatVectorIterator<Frame> const_iterator;

    typedef std::pair<iterator, iterator> iterator_pair;
    typedef std::pair<const_iterator, const_iterator> const_iterator_pair;

    const_iterator_pair getChildren() const;
    iterator_pair getChildren();
    const_iterator_pair getChildren(const State& state) const;
    iterator_pair getChildren(const State& state);
    const_iterator_pair getDafChildren(const State& state) const;
    iterator_pair getDafChildren(const State& state);
*/
    void attachTo(Frame* parent, State& state);
    bool isDAF();

private:
    // Frames should not be copied.
    Frame(const Frame&);
    Frame& operator=(const Frame&);
};

/**************************************************************************
 *  SENSOR
 *
 *************************************************************************/



/**************************************************************************
 *  MODELS
 *
 *************************************************************************/
class WorkCell {
public:
    std::string getName() const;
    Frame* getWorldFrame() const;
    void addDevice(Ptr<Device> device);
    const std::vector<Ptr<Device> >& getDevices() const;
    Frame* findFrame(const std::string& name) const;
    std::vector<Frame*> getFrames() const;
    Ptr<Device> findDevice(const std::string& name) const;
    State getDefaultState() const;

    //Ptr<StateStructure> getStateStructure();

    PropertyMap& getPropertyMap();
private:
    WorkCell(const WorkCell&);
    WorkCell& operator=(const WorkCell&);
};

class Device
{
public:
    typedef std::pair<Q, Q> QBox;

    Device(const std::string& name);
    //void registerStateData(rw::kinematics::StateStructure::Ptr sstruct);
    virtual void setQ(const Q& q, State& state) const = 0;
    virtual Q getQ(const State& state) const = 0;
    virtual QBox getBounds() const = 0;
    virtual Q getVelocityLimits() const = 0;
    virtual void setVelocityLimits(const Q& vellimits) = 0;
    virtual Q getAccelerationLimits() const = 0;
    virtual void setAccelerationLimits(const Q& acclimits) = 0;
    virtual size_t getDOF() const = 0;
    std::string getName() const;
    void setName(const std::string& name);
    virtual Frame* getBase() = 0;
    virtual const Frame* getBase() const = 0;
    virtual Frame* getEnd() = 0;
    virtual const Frame* getEnd() const = 0;
    Transform3D baseTframe(const Frame* f, const State& state) const;
    Transform3D baseTend(const State& state) const;
    Transform3D worldTbase(const State& state) const;
    virtual Jacobian baseJend(const State& state) const = 0;
    virtual Jacobian baseJframe(const Frame* frame,const State& state) const;
    virtual Jacobian baseJframes(const std::vector<Frame*>& frames,const State& state) const;
    //virtual Ptr<JacobianCalculator> baseJCend(const kinematics::State& state) const;
    //virtual JacobianCalculatorPtr baseJCframe(const kinematics::Frame* frame, const kinematics::State& state) const;
    //virtual JacobianCalculatorPtr baseJCframes(const std::vector<kinematics::Frame*>& frames, const kinematics::State& state) const = 0;
private:
    Device(const Device&);
    Device& operator=(const Device&);
};


class JointDevice: public Device
{
public:
    const std::vector<Joint*>& getJoints() const;
    void setQ(const Q& q, State& state) const;
    Q getQ(const State& state) const;
    size_t getDOF() const;
    std::pair<Q, Q> getBounds() const;
    void setBounds(const std::pair<Q, Q>& bounds);
    Q getVelocityLimits() const;
    void setVelocityLimits(const Q& vellimits);
    Q getAccelerationLimits() const;
    void setAccelerationLimits(const Q& acclimits);
    Jacobian baseJend(const State& state) const;

    //JacobianCalculatorPtr baseJCframes(const std::vector<kinematics::Frame*>& frames,
    //                                   const kinematics::State& state) const;

    Frame* getBase();
    const Frame* getBase() const;
    virtual Frame* getEnd();
    virtual const Frame* getEnd() const;

};

/******************************************************************************
 *  PROXIMITY
 *
 * *************************************************************************/


/******************************************************************************
 *  LOADERS
 *
 * *************************************************************************/

class WorkCellLoader{
public:
    static Ptr<WorkCell> load(const std::string& filename);
private:
    WorkCellLoader();
};

class ImageFactory{
public:
    static Ptr<Image> load(const std::string& filename);
private:
    ImageFactory();
};

class XMLTrajectoryLoader
{
public:
    XMLTrajectoryLoader(const std::string& filename, const std::string& schemaFileName = "");
    XMLTrajectoryLoader(std::istream& instream, const std::string& schemaFileName = "");

    enum Type { QType = 0, Vector3DType, Rotation3DType, Transform3DType};
    Type getType();
    rw::trajectory::QTrajectory::Ptr getQTrajectory();
    rw::trajectory::Vector3DTrajectory::Ptr getVector3DTrajectory();
    rw::trajectory::Rotation3DTrajectory::Ptr getRotation3DTrajectory();
    rw::trajectory::Transform3DTrajectory::Ptr getTransform3DTrajectory();
};

class XMLTrajectorySaver
{
public:
    static bool save(const rw::trajectory::QTrajectory& trajectory, const std::string& filename);
    static bool save(const rw::trajectory::Vector3DTrajectory& trajectory, const std::string& filename);
    static bool save(const rw::trajectory::Rotation3DTrajectory& trajectory, const std::string& filename);
    static bool save(const rw::trajectory::Transform3DTrajectory& trajectory, const std::string& filename);
    static bool write(const rw::trajectory::QTrajectory& trajectory, std::ostream& outstream);
    static bool write(const rw::trajectory::Vector3DTrajectory& trajectory, std::ostream& outstream);
    static bool write(const rw::trajectory::Rotation3DTrajectory& trajectory, std::ostream& outstream);
    static bool write(const rw::trajectory::Transform3DTrajectory& trajectory, std::ostream& outstream);
private:
    XMLTrajectorySaver();
};

