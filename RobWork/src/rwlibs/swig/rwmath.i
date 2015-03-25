
/********************************************
 * MATH
 ********************************************/
//! @copydoc rw::math::Matrix
class Matrix {
public:
	
	Matrix(int dimx, int dimy);
	
#if !defined(SWIGJAVA)
    double& operator()(size_t row, size_t column);
    const double& operator()(size_t row, size_t column) const;

    const Matrix operator+(const Matrix& wrench) const;    
    const Matrix operator-(const Matrix& wrench) const;
    const Matrix operator*(const Matrix& wrench) const;
#endif

#if defined(SWIGJAVA)
	%rename(subtract) operator-(const Matrix&) const;
	%rename(add) operator+(const Matrix&) const;
#endif
	
	%extend {
		Matrix pseudoinverse() {
			 return rw::math::LinearAlgebra::pseudoInverse( (*$self) );
		}
		
		double& elem(int x, int y){
			return (*$self)(x,y);
		}
		
		/* These accesors are neccesary because Python does not allow
		lvalues consisting of access operators calls (cannot assign to a function call).
		Moreover, it's not possible to dereference a pointer obtained from function returning a reference. */
		double get(int x, int y) {
			return (*$self)(x, y);
		}
		
		void set(int x, int y, double value) {
			(*$self)(x, y) = value;
		}

#if (defined(SWIGPYTHON))
		/* This typemap makes it possible to access Matrix class elements using following syntax:
		
		myMatrix[1, 1] = value
		print myMatrix[1, 1]
		
		-- using __getitem__ and __setitem__ methods. */
		%typemap(in) int[2](int temp[2]) {
			int i;
			if (PyTuple_Check($input)) {
				if (!PyArg_ParseTuple($input, "ii", temp, temp+1)) {
					PyErr_SetString(PyExc_TypeError, "tuple must have 2 elements");
					return NULL;
				}
				$1 = &temp[0];
			} else {
				PyErr_SetString(PyExc_TypeError, "expected a tuple.");
				return NULL;
			}
		}
		
        double __getitem__(int i[2])const {return (*$self)(i[0], i[1]); }
        void __setitem__(int i[2], double d){ (*$self)(i[0], i[1]) = d; }
#endif
	}
	
};

/**
 * @copydoc rw::math::Q 
 */
class Q
{
public:
    // first we define functions that are native to Q
	Q();
	//%feature("autodoc","1");
	Q(int n, double a0);
    Q(int n, double a0, double a1);
    Q(int n, double a0, double a1, double a2);
    Q(int n, double a0, double a1, double a2, double a3);
    Q(int n, double a0, double a1, double a2, double a3, double a4);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8, double a9);

    Q(const std::vector<double>& r);
#if defined(SWIGJAVA)
%apply double[] {double *};
#endif
    Q(int n, const double* values);

    int size() const;

#if !defined(SWIGJAVA)
    %rename(elem) operator[];
    double& operator[](unsigned int i) ;
#endif

#if defined(SWIGJAVA)
	%rename(subtract) operator-(const Q&) const;
	%rename(add) operator+(const Q&) const;
#endif

    const Q operator-() const;
    Q operator-(const Q& b) const;
    Q operator+(const Q& b) const;
    Q operator*(double s) const;
    Q operator/(double s) const;
    double norm2();
    double norm1();
    double normInf();

    %extend {
		
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Q>(*$self); }
        double __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Q>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
#endif
    };

};

%template (QVector) std::vector<Q>;
%template(QPair) std::pair<Q, Q>;

namespace rw {
namespace math {

template<class T> class Vector2D
{
public:
    Vector2D();
    %feature("autodoc","1");
    Vector2D(T x, T y);
    size_t size() const;

    T norm2();
    T norm1();
    T normInf();

    //double& operator[](unsigned int i) ;
    %rename(elem) operator[];

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<rw::math::Vector2D<T> >(*$self); }
        double __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<rw::math::Vector2D<T> >(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
#endif
    };

};

}}
%template (Vector2f) rw::math::Vector2D<float>;
%template (Vector2) rw::math::Vector2D<double>;
%template (Vector2Vector) std::vector<rw::math::Vector2D<double> >;

/**
 * @copydoc rw::math::Vector3D
 */
namespace rw { namespace math {
template<class T> class Vector3D
{
public:
    Vector3D();
    %feature("autodoc","1");
    Vector3D(T x, T y, T z);
    size_t size() const;
    Vector3D operator*(T scale) const;
#if defined(SWIGJAVA)
	%rename(subtract) operator-(const Vector3D&) const;
	%rename(add) operator+(const Vector3D&) const;
#endif
    Vector3D operator+(const Vector3D& other) const;
    Vector3D operator-(const Vector3D& other) const;
    bool operator==(const Vector3D& q);

    T norm2();
    T norm1();
    T normInf(); 

    //double& operator[](unsigned int i) ;
    //%rename(elem) operator[];
    
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<rw::math::Vector3D<T> >(*$self); }
        T __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<rw::math::Vector3D<T> >(*$self); }
        T get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
#endif
    };
};
}}
%template (Vector3) rw::math::Vector3D<double>;
%template (Vector3f) rw::math::Vector3D<float>;
%template (Vector3Vector) std::vector< rw::math::Vector3D<double> >;

/**
 * @copydoc rw::math::Rotation3D
 */
class Rotation3D
{
public:
    // Lua methods:
    Rotation3D();
    %feature("autodoc","1");
    Rotation3D(double v0,double v1,double v2,
    			double v3,double v4,double v5,
    			double v6,double v7,double v8);
    			
    explicit Rotation3D(const Rotation3D& R);

    Rotation3D operator*(const Rotation3D& other) const;
    rw::math::Vector3D<double> operator*(const rw::math::Vector3D<double>& vec) const;

    static const Rotation3D& identity();
    static Rotation3D skew(const rw::math::Vector3D<double>& v);
    
    void normalize();
    
    bool equal(const Rotation3D& rot, double precision) const;

    bool operator==(const Rotation3D &rhs) const;
    
    %extend {
    	const EAA operator*(const EAA& bTKc){
    		return *((rw::math::Rotation3D<>*)$self) * bTKc;
    	}
		Rotation3D inverse(){ return inverse(*$self); }       
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Rotation3D>(*$self); }
        double __getitem__(int x,int y)const {return (*$self)(x,y); }
        void __setitem__(int x,int y,double d){ (*$self)(x,y) = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Rotation3D>(*$self); }
        double get(std::size_t row, std::size_t column) const { return (*$self)(row, column); }
        void set(std::size_t row, std::size_t column, double d){ (*$self)(row, column) = d; }
#endif
    };
};

%template (Rotation3DVector) std::vector<Rotation3D>;


//! @copydoc rw::math::EAA
class EAA
{
public:
    // Lua methods:
    EAA();
     %feature("autodoc","1");
    EAA(const EAA& eaa);
     %feature("autodoc","1");
    EAA(const Rotation3D& rot);
     %feature("autodoc","1");
    EAA(const rw::math::Vector3D<double>& axis, double angle);
     %feature("autodoc","1");
    EAA(double thetakx, double thetaky, double thetakz);
     %feature("autodoc","1");
    EAA(const rw::math::Vector3D<double>& v1, const rw::math::Vector3D<double>& v2);

    double angle() const;
    rw::math::Vector3D<double> axis() const;

    Rotation3D toRotation3D() const;

	

    //bool operator==(const EAA &rhs) const;
    // std::string __tostring() const;
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<EAA>(*$self); }
        double __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
    	double& x() { return (*$self)[0]; }
    	double& y() { return (*$self)[1]; }
    	double& z() { return (*$self)[2]; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<EAA>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
    	double x() const { return (*$self)[0]; }
    	double y() const { return (*$self)[1]; }
    	double z() const { return (*$self)[2]; }
#endif
    };
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
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<RPY>(*$self); }
        double __getitem__(int i)const {
        	if(i<0 || i>2) throw("Index is outside bounds. Must be in range [0;2]");
        	return (*$self)[i]; 
        }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<RPY>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
#endif
    };
};

class ZYX
{
public:
    // Lua methods:
    ZYX();
    ZYX(const ZYX& eaa);
    ZYX(const Rotation3D& rot);
    ZYX(double roll, double pitch, double yaw);
    Rotation3D toRotation3D() const;
    
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<ZYX>(*$self); }
        double __getitem__(int i)const {
        	if(i<0 || i>2) throw("Index is outside bounds. Must be in range [0;2]");
        	return (*$self)[i]; 
        }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<ZYX>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
#endif
    };
};

//! @copydoc rw::math::Quaternion
class Quaternion
{
public:
    // Lua methods:
    Quaternion();
    Quaternion(double qx, double qy, double qz, double qw);
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
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Quaternion>(*$self); }
        double __getitem__(int i)const {return (*$self)(i); }
        void __setitem__(int i,double d){ (*$self)(i) = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Quaternion>(*$self); }
        double get(std::size_t i) const { return (*$self)(i); }
        void set(std::size_t i,double d){ (*$self)(i) = d; }
#endif
    };
};



//Quaternion operator*(double s, const Quaternion& v);

class Transform3D {
public:
	Transform3D();
    Transform3D(const Transform3D& t3d);
    Transform3D(const rw::math::Vector3D<double>& position,const Rotation3D& rotation);

    Transform3D operator*(const Transform3D& other) const;
    rw::math::Vector3D<double> operator*(const rw::math::Vector3D<double>& other) const;

    static Transform3D DH(double alpha, double a, double d, double theta);
    static Transform3D craigDH(double alpha, double a, double d, double theta);
	
    rw::math::Vector3D<double>& P();
    Rotation3D& R();

    %extend {
       Transform3D inverse(){ return inverse(*$self); }
       //Transform3D inverse(const Transform3D& val);
    };
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Transform3D>(*$self); }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Transform3D>(*$self); }
#endif
    };
};

%template (Transform3DVector) std::vector<Transform3D>;

/*
%inline %{

    Transform3D inverse(Transform3D t3d){ return rw::math::inverse(t3d); }

    Rotation3D inverse(Rotation3D t3d){
        std::cout << "CALLING LUA INVERSE.... DUNNO WHY" << std::endl;
        return rw::math::inverse(t3d);
    }

%}
*/

class Pose6D {
public:
	Pose6D(const Pose6D& p6d);
    Pose6D(const Vector3& position,const EAA& rotation);
    Pose6D(const Transform3D& t3d);

    Transform3D toTransform3D();
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Pose6D>(*$self); }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Pose6D>(*$self); }
#endif
    };
};

class Jacobian
{
public:
    Jacobian(int m, int n);

    int size1() const ;
    int size2() const ;

#if (defined(SWIGLUA) || defined(SWIGPYTHON))
    double& elem(int i, int j);

    %extend {
        char *__str__() { return printCString<Jacobian>(*$self); }
        double __getitem__(std::size_t row, std::size_t column)const {return (*$self)(row, column); }
        void __setitem__(std::size_t row, std::size_t column,double d){ (*$self)(row, column) = d; }
    };
#elif defined(SWIGJAVA)
    %extend {
        std::string toString() const { return toString<Jacobian>(*$self); }
        double elem(std::size_t row, std::size_t column) const { return (*$self)(row, column); }
        double get(std::size_t row, std::size_t column) const { return (*$self)(row, column); }
        void set(std::size_t row, std::size_t column, double d){ (*$self)(row, column) = d; }
    };
#endif
};



class VelocityScrew6D
{
public:
	VelocityScrew6D();
	VelocityScrew6D(const VelocityScrew6D& p6d);
    VelocityScrew6D(const rw::math::Vector3D<double>& position,const EAA& rotation);
    VelocityScrew6D(const Transform3D& t3d);

    // lua functions
    VelocityScrew6D operator*(double scale) const;
#if defined(SWIGJAVA)
	%rename(subtract) operator-(const VelocityScrew6D&) const;
	%rename(add) operator+(const VelocityScrew6D&) const;
#endif
    VelocityScrew6D operator+(const VelocityScrew6D& other) const;
    VelocityScrew6D operator-(const VelocityScrew6D& other) const;
    //bool operator==(const VelocityScrew6D& q);

    double norm2();
    double norm1();
    double normInf();

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<VelocityScrew6D>(*$self); }
        double __getitem__(std::size_t i)const {return (*$self)[i]; }
        void __setitem__(std::size_t i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<VelocityScrew6D>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i, double d){ (*$self)[i] = d; }
#endif
    };

    //Transform3D toTransform3D();
    // std::string __tostring() const;
};


class Wrench6D
{
public:		
    Wrench6D(double fx, double fy, double fz, double tx, double ty, double tz);

    // TODO: add constructor on vector

    Wrench6D();

    Wrench6D(const rw::math::Vector3D<double>& force, const rw::math::Vector3D<double>& torque);

    const rw::math::Vector3D<double> force() const;
    const rw::math::Vector3D<double> torque() const;

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Wrench6D>(*$self); }
        double __getitem__(std::size_t i)const {return (*$self)[i]; }
        void __setitem__(std::size_t i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Wrench6D>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i, double d){ (*$self)[i] = d; }
#endif
    };

    const Wrench6D operator*( double s) const;
    /*
    friend const Wrench6D<T> operator*(const Transform3D& aTb,
                                              const Wrench6D& bV);
    
    friend const Wrench6D operator*(const rw::math::Vector3D<double>& aPb,
                                       const Wrench6D& bV);


    friend const Wrench6D operator*(const Rotation3D& aRb, const Wrench6D& bV);
*/
#if defined(SWIGJAVA)
	%rename(subtract) operator-(const Wrench6D&) const;
	%rename(add) operator+(const Wrench6D&) const;
#endif
    const Wrench6D operator+(const Wrench6D& wrench) const;    
    const Wrench6D operator-(const Wrench6D& wrench) const;

    double norm1() const;
    
    double norm2() const ;
    double normInf() const ;
};


class InertiaMatrix{
public:
    InertiaMatrix(
        double r11, double r12, double r13,
        double r21, double r22, double r23,
        double r31, double r32, double r33);

    InertiaMatrix(
        const rw::math::Vector3D<double>& i,
        const rw::math::Vector3D<double>& j,
        const rw::math::Vector3D<double>& k);

    InertiaMatrix(
        double i = 0.0,
        double j = 0.0,
        double k = 0.0);

#if !defined(SWIGJAVA)
    double& operator()(size_t row, size_t column);
    const double& operator()(size_t row, size_t column) const;
#endif

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<InertiaMatrix>(*$self); }
        double __getitem__(std::size_t row, std::size_t column)const {return (*$self)(row,column); }
        void __setitem__(std::size_t row, std::size_t column, double d){ (*$self)(row,column) = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<InertiaMatrix>(*$self); }
        double get(std::size_t row, std::size_t column) const { return (*$self)(row,column); }
        void set(std::size_t row, std::size_t column, double val) { (*$self)(row,column) = val; }
#endif
    };

    //const Base& m() const;
    //Base& m();

    //friend InertiaMatrix operator*(const Rotation3D<T>& aRb, const InertiaMatrix& bRc);
    //friend InertiaMatrix operator*(const InertiaMatrix& aRb, const Rotation3D<T>& bRc);
    //friend InertiaMatrix operator+(const InertiaMatrix& I1, const InertiaMatrix& I2);
    //friend Vector3<T> operator*(const InertiaMatrix& aRb, const Vector3<T>& bVc);
    //friend InertiaMatrix inverse(const InertiaMatrix& aRb);
    //friend std::ostream& operator<<(std::ostream &os, const InertiaMatrix& r);

    static InertiaMatrix makeSolidSphereInertia(double mass, double radi);
    static InertiaMatrix makeHollowSphereInertia(double mass, double radi);
    static InertiaMatrix makeCuboidInertia(double mass, double x, double y, double z);
};

%nodefaultctor Metric;
template <class T>
class Metric
{
public:
    double distance(const T& q) const;

    double distance(const T& a, const T& b) const;

    int size() const;

};

%template (MetricQ) Metric<Q>;
%template (MetricQPtr) rw::common::Ptr<Metric<Q> >;
%template (MetricSE3) Metric<Transform3D>;
%template (MetricSE3Ptr) rw::common::Ptr<Metric<Transform3D> >;
