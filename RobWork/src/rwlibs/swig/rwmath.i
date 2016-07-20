
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
#endif

    const Matrix operator+(const Matrix& wrench) const;    
    const Matrix operator-(const Matrix& wrench) const;
    const Matrix operator*(const Matrix& wrench) const;
	
	%extend {
		Matrix pseudoinverse() {
			 return rw::math::LinearAlgebra::pseudoInverse( (*$self) );
		}
		
#if !defined(SWIGJAVA)
		double& elem(int x, int y){
			return (*$self)(x,y);
		}
#endif
		
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

namespace rw { namespace math {
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
        char *__str__() { return printCString<rw::math::Q>(*$self); }
        double __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<rw::math::Q>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
#endif
    };

};
} }

%template (QVector) std::vector<rw::math::Q>;
%template(QPair) std::pair<rw::math::Q, rw::math::Q>;

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
%template (Vector3d) rw::math::Vector3D<double>;
%template (Vector3f) rw::math::Vector3D<float>;
%template (Vector3Vector) std::vector< rw::math::Vector3D<double> >;

namespace rw { namespace math {
/**
 * @copydoc rw::math::Rotation3D
 */
template<class T> class Rotation3D
{
public:
    // Lua methods:
    Rotation3D();
    %feature("autodoc","1");
    Rotation3D(T v0,T v1,T v2,
    			T v3,T v4,T v5,
    			T v6,T v7,T v8);
    			
    explicit Rotation3D(const Rotation3D<T>& R);

    Rotation3D<T> operator*(const Rotation3D<T>& other) const;
    rw::math::Vector3D<T> operator*(const rw::math::Vector3D<T>& vec) const;

    static const Rotation3D<T>& identity();
    static Rotation3D<T> skew(const rw::math::Vector3D<T>& v);
    
    void normalize();
    
    bool equal(const Rotation3D<T>& rot, double precision) const;

    bool operator==(const Rotation3D<T> &rhs) const;
    
    %extend {
    	rw::math::EAA<T> toEAA(){ return rw::math::EAA<T>(*$self); }
    	rw::math::RPY<T> toRPY(){ return rw::math::RPY<T>(*$self); }
    	rw::math::Quaternion<T> toQuaternion(){ return rw::math::Quaternion<T>(*$self); }
    	
    	const rw::math::EAA<T> operator*(const rw::math::EAA<T>& bTKc){
    		return *((rw::math::Rotation3D<T>*)$self) * bTKc;
    	}
		Rotation3D<T> inverse(){ return inverse(*$self); }       
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Rotation3D<T> >(*$self); }
        T __getitem__(int x,int y)const {return (*$self)(x,y); }
        void __setitem__(int x,int y,double d){ (*$self)(x,y) = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Rotation3D<T> >(*$self); }
        T get(std::size_t row, std::size_t column) const { return (*$self)(row, column); }
        void set(std::size_t row, std::size_t column, double d){ (*$self)(row, column) = d; }
#endif
    };

#if defined(SWIGJAVA)    
    %extend {
    	rw::math::InertiaMatrix<T> multiply(const rw::math::InertiaMatrix<T>& bRc) { return (*$self)*bRc; }
    };
#endif
};
}}

%template (Rotation3d) rw::math::Rotation3D<double>;
%template (Rotation3f) rw::math::Rotation3D<float>;
%template (Rotation3Vector) std::vector< rw::math::Rotation3D<double> >;

namespace rw { namespace math {
//! @copydoc rw::math::EAA
template<class T> class EAA
{
public:
    // Lua methods:
    EAA();
     %feature("autodoc","1");
    EAA(const EAA<T>& eaa);
     %feature("autodoc","1");
    EAA(const Rotation3D<T>& rot);
     %feature("autodoc","1");
    EAA(const rw::math::Vector3D<T>& axis, T angle);
     %feature("autodoc","1");
    EAA(T thetakx, T thetaky, T thetakz);
     %feature("autodoc","1");
    EAA(const rw::math::Vector3D<T>& v1, const rw::math::Vector3D<T>& v2);

    double angle() const;
    rw::math::Vector3D<T> axis() const;

    Rotation3D<T> toRotation3D() const;

	

    //bool operator==(const EAA &rhs) const;
    // std::string __tostring() const;
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<EAA<T> >(*$self); }
        T __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
    	T& x() { return (*$self)[0]; }
    	T& y() { return (*$self)[1]; }
    	T& z() { return (*$self)[2]; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<EAA<T> >(*$self); }
        T get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
    	T x() const { return (*$self)[0]; }
    	T y() const { return (*$self)[1]; }
    	T z() const { return (*$self)[2]; }
#endif
    };
};
}}

%template (EAAd) rw::math::EAA<double>;
%template (EAAf) rw::math::EAA<float>;
%template (EAA3Vector) std::vector< rw::math::EAA<double> >;

namespace rw { namespace math {
//! @copydoc rw::math::RPY
template<class T> class RPY
{
public:
    // Lua methods:
    RPY();
    RPY(const RPY& eaa);
    RPY(const rw::math::Rotation3D<T>& rot);
    RPY(T roll, T pitch, T yaw);
    rw::math::Rotation3D<T> toRotation3D() const;
    
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<RPY<T> >(*$self); }
        T __getitem__(int i)const {
        	if(i<0 || i>2) throw("Index is outside bounds. Must be in range [0;2]");
        	return (*$self)[i]; 
        }
        void __setitem__(int i,T d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<RPY<T> >(*$self); }
        T get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,T d){ (*$self)[i] = d; }
#endif
    };
};
}}

%template (RPYd) rw::math::RPY<double>;
%template (RPYf) rw::math::RPY<float>;
%template (RPYdVector) std::vector< rw::math::RPY<double> >;

//! @copydoc rw::math::Quaternion
namespace rw { namespace math {
template<class T> class Quaternion
{
public:
    // Lua methods:
    Quaternion();
    Quaternion(T qx, T qy, T qz, T qw);
    Quaternion(const Quaternion& eaa);
    Quaternion(const rw::math::Rotation3D<T>& rot);
    Quaternion operator*(T s);

    void normalize();

    rw::math::Rotation3D<T> toRotation3D() const;
    Quaternion<T> slerp(const Quaternion<T>& v, const double t) const;

    T getQx() const;
    T getQy() const;
    T getQz() const;
    T getQw() const;

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<rw::math::Quaternion<T> >(*$self); }
        T __getitem__(int i)const {return (*$self)(i); }
        void __setitem__(int i,T d){ (*$self)(i) = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<rw::math::Quaternion<T> >(*$self); }
        T get(std::size_t i) const { return (*$self)(i); }
        void set(std::size_t i,T d){ (*$self)(i) = d; }
#endif
    };
};
}}

%template (Quaterniond) rw::math::Quaternion<double>;
%template (Quaternionf) rw::math::Quaternion<float>;
%template (QuaterniondVector) std::vector< rw::math::Quaternion<double> >;

namespace rw { namespace math {
template<class T> class Transform3D {
public:
	Transform3D();
    Transform3D(const Transform3D<T>& t3d);
    Transform3D(const rw::math::Vector3D<T>& position,const rw::math::Rotation3D<T>& rotation);

    Transform3D operator*(const Transform3D<T>& other) const;
    rw::math::Vector3D<T> operator*(const rw::math::Vector3D<T>& other) const;

    static Transform3D<T> DH(double alpha, double a, double d, double theta);
    static Transform3D<T> craigDH(double alpha, double a, double d, double theta);
	
    rw::math::Vector3D<T>& P();
    rw::math::Rotation3D<T>& R();

    %extend {
       Transform3D<T> inverse(){ return inverse(*$self); }
       //Transform3D<T> inverse(const Transform3D<T>& val);
    };
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Transform3D<T> >(*$self); }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Transform3D<T> >(*$self); }
#endif
    };
};
}}

%template (Transform3d) rw::math::Transform3D<double>;
%template (Transform3f) rw::math::Transform3D<float>;
%template (Transform3dVector) std::vector<rw::math::Transform3D<double> >;

namespace rw { namespace math {


template<class T> class Pose6D {
public:
	Pose6D(const Pose6D<T>& p6d);
    Pose6D(const Vector3D<T>& position,const EAA<T>& rotation);
    Pose6D(const Transform3D<T>& t3d);

    rw::math::Transform3D<T> toTransform3D();
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Pose6D<T> >(*$self); }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Pose6D<T> >(*$self); }
#endif
    };
};



template<class T> class VelocityScrew6D
{
public:
	VelocityScrew6D();
	VelocityScrew6D(const VelocityScrew6D<T>& p6d);
    VelocityScrew6D(const rw::math::Vector3D<T>& position,const EAA<T>& rotation);
    VelocityScrew6D(const Transform3D<T>& t3d);

    // lua functions
    VelocityScrew6D<T> operator*(T scale) const;
    
    VelocityScrew6D<T> operator+(const VelocityScrew6D<T>& other) const;
    VelocityScrew6D<T> operator-(const VelocityScrew6D<T>& other) const;
    //bool operator==(const VelocityScrew6D<T>& q);

    double norm2();
    double norm1();
    double normInf();

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<VelocityScrew6D<T> >(*$self); }
        T __getitem__(std::size_t i)const {return (*$self)[i]; }
        void __setitem__(std::size_t i,T d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<VelocityScrew6D<T> >(*$self); }
        T get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i, T d){ (*$self)[i] = d; }
#endif
    };

    //Transform3D toTransform3D();
    // std::string __tostring() const;
};


template<class T> class Wrench6D
{
public:		
    Wrench6D(T fx, T fy, T fz, T tx, T ty, T tz);

    // TODO: add constructor on vector

    Wrench6D();

    Wrench6D(const rw::math::Vector3D<T>& force, const rw::math::Vector3D<T>& torque);

    const rw::math::Vector3D<T> force() const;
    const rw::math::Vector3D<T> torque() const;

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Wrench6D<T> >(*$self); }
        T __getitem__(std::size_t i)const {return (*$self)[i]; }
        void __setitem__(std::size_t i,T d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Wrench6D<T> >(*$self); }
        T get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i, T d){ (*$self)[i] = d; }
#endif
    };

    const Wrench6D<T> operator*( double s) const;
    
    friend const Wrench6D<T> operator*(const Transform3D<T>& aTb,
                                              const Wrench6D<T>& bV);
    
    friend const Wrench6D operator*(const rw::math::Vector3D<T>& aPb, const Wrench6D<T>& bV);


    friend const Wrench6D operator*(const rw::math::Rotation3D<T>& aRb, const Wrench6D<T>& bV);

    const Wrench6D<T> operator+(const Wrench6D<T>& wrench) const;    
    const Wrench6D<T> operator-(const Wrench6D<T>& wrench) const;

    double norm1() const;
    
    double norm2() const ;
    double normInf() const ;
};

template<class T> class InertiaMatrix{
public:
    InertiaMatrix(
        T r11, T r12, T r13,
        T r21, T r22, T r23,
        T r31, T r32, T r33);

    InertiaMatrix(
        const rw::math::Vector3D<T>& i,
        const rw::math::Vector3D<T>& j,
        const rw::math::Vector3D<T>& k);

    InertiaMatrix(
        T i = 0.0,
        T j = 0.0,
        T k = 0.0);

#if !defined(SWIGJAVA)
    T& operator()(size_t row, size_t column);
    const T& operator()(size_t row, size_t column) const;
#endif

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<rw::math::InertiaMatrix<T> >(*$self); }
        T __getitem__(std::size_t row, std::size_t column)const {return (*$self)(row,column); }
        void __setitem__(std::size_t row, std::size_t column, T d){ (*$self)(row,column) = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<rw::math::InertiaMatrix<T> >(*$self); }
        T get(std::size_t row, std::size_t column) const { return (*$self)(row,column); }
        void set(std::size_t row, std::size_t column, T val) { (*$self)(row,column) = val; }
#endif
    };

    //const Base& m() const;
    //Base& m();

#if !defined(SWIGJAVA)
    friend InertiaMatrix<T> operator*(const rw::math::Rotation3D<T>& aRb, const InertiaMatrix<T>& bRc);
    friend InertiaMatrix<T> operator*(const InertiaMatrix<T>& aRb, const Rotation3D<T>& bRc);
    friend InertiaMatrix<T> operator+(const InertiaMatrix<T>& I1, const InertiaMatrix<T>& I2);
    friend Vector3D<T> operator*(const InertiaMatrix<T>& aRb, const Vector3D<T>& bVc);
    //friend InertiaMatrix<T> inverse(const InertiaMatrix<T>& aRb);
    //friend std::ostream& operator<<(std::ostream &os, const InertiaMatrix<T>& r);
#endif

#if defined(SWIGJAVA)    
    %extend {
    	InertiaMatrix<T> multiply(const Rotation3D<T>& bRc) { return (*$self)*bRc; }
    	InertiaMatrix<T> add(const InertiaMatrix<T>& I2) { return (*$self)+I2; }
    	Vector3D<T> multiply(const Vector3D<T>& bVc) { return (*$self)*bVc; }
    };
#endif

    static InertiaMatrix<T> makeSolidSphereInertia(double mass, double radi);
    static InertiaMatrix<T> makeHollowSphereInertia(double mass, double radi);
    static InertiaMatrix<T> makeCuboidInertia(double mass, double x, double y, double z);
};

}}

%template (Pose6d) rw::math::Pose6D<double>;
%template (Pose6f) rw::math::Pose6D<float>;
%template (Pose6dVector) std::vector<rw::math::Pose6D<double> >;

%template (Wrench6d) rw::math::Wrench6D<double>;
%template (Wrench6f) rw::math::Wrench6D<float>;
%template (Wrench6dVector) std::vector<rw::math::Wrench6D<double> >;

%template (Screw6d) rw::math::VelocityScrew6D<double>;
%template (Screw6f) rw::math::VelocityScrew6D<float>;
%template (Screw6dVector) std::vector<rw::math::VelocityScrew6D<double> >;

%template (InertiaMatrixd) rw::math::InertiaMatrix<double>;
%template (InertiaMatrixf) rw::math::InertiaMatrix<float>;
%template (InertiaMatrixdVector) std::vector<rw::math::InertiaMatrix<double> >;


namespace rw { namespace math {
class Jacobian
{
public:
    Jacobian(int m, int n);

    int size1() const ;
    int size2() const ;

#if (defined(SWIGLUA) || defined(SWIGPYTHON))
    double& elem(int i, int j);

    %extend {
        char *__str__() { return printCString<rw::math::Jacobian>(*$self); }
        double __getitem__(std::size_t row, std::size_t column)const {return (*$self)(row, column); }
        void __setitem__(std::size_t row, std::size_t column,double d){ (*$self)(row, column) = d; }
    };
#elif defined(SWIGJAVA)
    %extend {
        std::string toString() const { return toString<rw::math::Jacobian>(*$self); }
        double elem(std::size_t row, std::size_t column) const { return (*$self)(row, column); }
        double get(std::size_t row, std::size_t column) const { return (*$self)(row, column); }
        void set(std::size_t row, std::size_t column, double d){ (*$self)(row, column) = d; }
    };
#endif
};
} }




%nodefaultctor Metric;
template <class T>
class Metric
{
public:
    double distance(const T& q) const;

    double distance(const T& a, const T& b) const;

    int size() const;

};

%template (MetricQ) Metric<rw::math::Q>;
%template (MetricQPtr) rw::common::Ptr<Metric<rw::math::Q> >;
%template (MetricSE3) Metric<rw::math::Transform3D<double> >;
%template (MetricSE3Ptr) rw::common::Ptr<Metric<rw::math::Transform3D<double> > >;
