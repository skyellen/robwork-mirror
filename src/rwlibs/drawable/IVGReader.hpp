//
// IVGReader.h
//

#ifndef __IVGReader_H
#define __IVGReader_H

#include <list>
#include <string>
#include <cmath>
#include <cstring>

#define IVG_TYPE_BODY 0
#define IVG_TYPE_FACE 1
#define IVG_TYPE_EDGE 2
#define IVG_TYPE_VERTEX 3
#define IVG_TYPE_REFNODE 4
#define IVG_TYPE_AXIS 5
#define IVG_TYPE_COLLECTION 6
#define IVG_TYPE_NONE 7
#define IVG_TYPE_NOT_ACIS 9

#define IVG_TYPE_TESS 10
#define IVG_TYPE_POLY 11
#define IVG_TYPE_WIRE 12
#define IVG_TYPE_SHADED_WIRE 13
#define IVG_TYPE_SPHERE 14
#define IVG_TYPE_CONE 15
#define IVG_TYPE_POINT 16
#define IVG_TYPE_STL_MESS 17

namespace rwlibs { namespace drawable {

    typedef struct TriangleTag {
        int vInx[3];	// vertex index
        int nInx[3];	// normal index
    } Triangle;

    class IvgVec3d
    {
    public:

        IvgVec3d() {} // no operations done to maintain speed
        IvgVec3d(double x,double y,double z) { _v[0]=x; _v[1]=y; _v[2]=z; }

        double _v[3];

        inline const bool operator == (const IvgVec3d& v) const
        { return _v[0]==v._v[0] && _v[1]==v._v[1] && _v[2]==v._v[2]; }

        inline const bool operator <  (const IvgVec3d& v) const
        {
            if (_v[0]<v._v[0]) return true;
            else if (_v[0]>v._v[0]) return false;
            else if (_v[1]<v._v[1]) return true;
            else if (_v[1]>v._v[1]) return false;
            else return (_v[2]<v._v[2]);
        }

        inline double* ptr() { return _v; }
        inline const double* ptr() const { return _v; }

        inline void set( double x, double y, double z)
        {
            _v[0]=x; _v[1]=y; _v[2]=z;
        }

        inline double& operator [] (int i) { return _v[i]; }
        inline const double operator [] (int i) const { return _v[i]; }

        inline double& x() { return _v[0]; }
        inline double& y() { return _v[1]; }
        inline double& z() { return _v[2]; }

        inline const double x() const { return _v[0]; }
        inline const double y() const { return _v[1]; }
        inline const double z() const { return _v[2]; }

        /// dot product
        inline double operator * (const IvgVec3d& rhs) const
        {
            return _v[0]*rhs._v[0]+_v[1]*rhs._v[1]+_v[2]*rhs._v[2];
        }

        /// cross product
        inline const IvgVec3d operator ^ (const IvgVec3d& rhs) const
        {
            return IvgVec3d(_v[1]*rhs._v[2]-_v[2]*rhs._v[1],
                            _v[2]*rhs._v[0]-_v[0]*rhs._v[2] ,
                            _v[0]*rhs._v[1]-_v[1]*rhs._v[0]);
        }

        /// multiply by scalar
        inline const IvgVec3d operator * (const double& rhs) const
        {
            return IvgVec3d(_v[0]*rhs, _v[1]*rhs, _v[2]*rhs);
        }

        /// unary multiply by scalar
        inline IvgVec3d& operator *= (const double& rhs)
        {
            _v[0]*=rhs;
            _v[1]*=rhs;
            _v[2]*=rhs;
            return *this;
        }

        /// divide by scalar
        inline const IvgVec3d operator / (const double& rhs) const
        {
            return IvgVec3d(_v[0]/rhs, _v[1]/rhs, _v[2]/rhs);
        }

        /// unary divide by scalar
        inline IvgVec3d& operator /= (const double& rhs)
        {
            _v[0]/=rhs;
            _v[1]/=rhs;
            _v[2]/=rhs;
            return *this;
        }

        /// binary vector add
        inline const IvgVec3d operator + (const IvgVec3d& rhs) const
        {
            return IvgVec3d(_v[0]+rhs._v[0], _v[1]+rhs._v[1], _v[2]+rhs._v[2]);
        }

        /** unary vector add.  Slightly more efficient because no temporary
            intermediate object*/
        inline IvgVec3d& operator += (const IvgVec3d& rhs)
        {
            _v[0] += rhs._v[0];
            _v[1] += rhs._v[1];
            _v[2] += rhs._v[2];
            return *this;
        }

        /// binary vector subract
        inline const IvgVec3d operator - (const IvgVec3d& rhs) const
        {
            return IvgVec3d(_v[0]-rhs._v[0], _v[1]-rhs._v[1], _v[2]-rhs._v[2]);
        }

        /// unary vector subract
        inline IvgVec3d& operator -= (const IvgVec3d& rhs)
        {
            _v[0]-=rhs._v[0];
            _v[1]-=rhs._v[1];
            _v[2]-=rhs._v[2];
            return *this;
        }

        /// negation operator.  Returns the negative of the IvgVec3d
        inline const IvgVec3d operator - () const
        {
            return IvgVec3d (-_v[0], -_v[1], -_v[2]);
        }

        /// Length of the vector = sqrt( vec . vec )
        inline const double length() const
        {
            return sqrt( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] );
        }

        /// Length squared of the vector = vec . vec
        inline const double length2() const
        {
            return _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2];
        }

        /** normalize the vector so that it has length unity
            returns the previous length of the vector*/
        inline const double normalize()
        {
            double norm = IvgVec3d::length();
            _v[0] /= norm;
            _v[1] /= norm;
            _v[2] /= norm;
            return( norm );
        }

		//friend inline ostream& operator << (ostream& output, const IvgVec3d& vec);

    };	// end of class IvgVec3d

    class CIVGNode
    {

    public:
        CIVGNode() : nType(0), nId(0), nTypeMask(0), nColor(0), nPixelSize(0), pNext(0) { }

    private:
        // These are not used, so make them private.
        CIVGNode& operator= (const CIVGNode& t)
        {
            pNext = t.pNext;
            nType = t.nType;
            nId = t.nId;
            nTypeMask = t.nTypeMask;
            nColor = t.nColor;
            nPixelSize = t.nPixelSize;
            translation = t.translation;

            return *this;
        }

        CIVGNode(const CIVGNode& t)
        {
            pNext = t.pNext;
            nType = t.nType;
            nId = t.nId;
            nTypeMask = t.nTypeMask;
            nColor = t.nColor;
            nPixelSize = t.nPixelSize;
            translation = t.translation;
        }

    public:
        virtual ~CIVGNode() {}

        // Attributes that all entities share -------------------------------
        int nType;  // TYPE_TESS, TYPE_POLY, TYPE_WIRE, TYPE_SHADED_WIRE, TYPE_SPHERE, TYPE_CONE
        // TYPE_BODY, TYPE_FACE, TYPE_EDGE, TYPE_NOT_ACIS
        int nId;
        int nTypeMask;
        int nColor;
        int nPixelSize;
        IvgVec3d translation;
        // ------------------------------------------------------------------

        CIVGNode *pNext;	// to form a simple list of nodes

    }; // CIVGNode

    class CIVGBody : public CIVGNode
    {

    public:
        CIVGBody() : nSubCount(0) { }

        ~CIVGBody() {}

    private:
        // Not used, so make them private.
        CIVGBody(const CIVGBody& t)
        {
            nSubCount = t.nSubCount;
        }

        CIVGBody& operator= (const CIVGBody& t)
        {
            nSubCount = t.nSubCount;

            return *this;
        }

    public:
        // Attributes specific to BODY, FACE, EDGE and NOT_ACIS -------------
        int nSubCount;
        // ------------------------------------------------------------------

    }; // CIVGBody

    class CIVGPoly : public CIVGNode
    {

    public:
        CIVGPoly() : nBoundaries(0) { }

        ~CIVGPoly() {}

    private:
        // Prefer 'private'.
        CIVGPoly(const CIVGPoly& t)
        {
            nBoundaries = t.nBoundaries;
        }

        CIVGPoly& operator= (const CIVGPoly& t)
        {
            nBoundaries = t.nBoundaries;

            return *this;
        }

    public:
        // Attributes specific to POLY --------------------------------------
        int nBoundaries;
        int *pnVertices;
        IvgVec3d **ppBoundaries;
        // ------------------------------------------------------------------

    }; // CIVGPoly

    class CIVGTess : public CIVGNode
    {

    public:
        CIVGTess() : nVertices(0), pVertices(0), nTriangles(0), pTriangles(0),
                     nNormals(0), pNormals(0)  { }

        ~CIVGTess()
        {
            delete [] pVertices;
            delete [] pTriangles;
            delete [] pNormals;
        }

    private:
        // These are not correct (!), so make them private:
        CIVGTess(const CIVGTess& t)
        {
            nVertices = t.nVertices;
            nTriangles = t.nTriangles;
            nNormals = t.nNormals;
        }

        CIVGTess& operator= (const CIVGTess& t)
        {
            nVertices = t.nVertices;
            nTriangles = t.nTriangles;
            nNormals = t.nNormals;

            return *this;
        }

    public:
        // Attributes specific to TESS, WIRE and SHADED_WIRE ----------------
        int nVertices;
        IvgVec3d *pVertices;
        // ------------------------------------------------------------------

        // Attributes specific to TESS --------------------------------------
        int nTriangles;
        Triangle *pTriangles;

        int nNormals;
        IvgVec3d *pNormals;
        // ------------------------------------------------------------------

    }; // CIVGTess

    class CIVGCurve : public CIVGNode
    {

    public:
        CIVGCurve() : nVertices(0), pVertices(0), dLineThickness(0.0) { }

        ~CIVGCurve()
        {
            delete [] pVertices;
        }

    private:
        // These too are not correct, so make them private.
        CIVGCurve(const CIVGCurve& t)
        {
            nVertices = t.nVertices;
            dLineThickness = t.dLineThickness;
        }

        CIVGCurve& operator= (const CIVGCurve& t)
        {
            nVertices = t.nVertices;
            dLineThickness = t.dLineThickness;

            return *this;
        }

    public:
        // Attributes specific to TESS, WIRE and SHADED_WIRE ----------------
        int nVertices;
        IvgVec3d *pVertices;
        // ------------------------------------------------------------------

        // Attributes specific to WIRE and SHADED_WIRE ----------------------
        double dLineThickness;
        // ------------------------------------------------------------------

    }; // CIVGWire

    class CIVGSphere : public CIVGNode
    {

    public:
        CIVGSphere() : dRadius(0.0) { }

        ~CIVGSphere() {}

    private:
        // Prefer 'private'.
        CIVGSphere(const CIVGSphere& t)
        {
            dRadius = t.dRadius;
            vCenter = t.vCenter;
        }

        CIVGSphere& operator= (const CIVGSphere& t)
        {
            dRadius = t.dRadius;
            vCenter = t.vCenter;

            return *this;
        }

    public:
        // Attributes specific to SPHERE ------------------------------------
        IvgVec3d vCenter;
        double dRadius;
        // ------------------------------------------------------------------

    }; // CIVGSphere

    class CIVGCone : public CIVGNode
    {
    public:
        CIVGCone() : bShowBottom(false), dBottomRadius(0.0),
                     bShowTop(false), dTopRadius(0.0) { }

        ~CIVGCone() {}

    private:
        CIVGCone(const CIVGCone& t)
        {
            vBottom = t.vBottom;
            dBottomRadius = t.dBottomRadius;
            bShowBottom = t.bShowBottom;
            vTop = t.vTop;
            dTopRadius = t.dTopRadius;
            bShowTop = t.bShowTop;
        }

        CIVGCone& operator= (const CIVGCone& t)
        {
            vBottom = t.vBottom;
            dBottomRadius = t.dBottomRadius;
            bShowBottom = t.bShowBottom;
            vTop = t.vTop;
            dTopRadius = t.dTopRadius;
            bShowTop = t.bShowTop;

            return *this;
        }

    public:
        // Attributes specific to CONE --------------------------------------
        IvgVec3d vBottom;
        bool bShowBottom;
        double dBottomRadius;


        IvgVec3d vTop;
        bool bShowTop;
        double dTopRadius;

        // ------------------------------------------------------------------

    }; // CIVGCone

    class CIVGPoint : public CIVGNode
    {

    public:
        CIVGPoint() : dRadius(0.0) { }

        ~CIVGPoint() {}

    private:
        CIVGPoint(const CIVGPoint& t)
        {
            dRadius = t.dRadius;
            vCenter = t.vCenter;
        }

        CIVGPoint& operator= (const CIVGPoint& t)
        {
            dRadius = t.dRadius;
            vCenter = t.vCenter;

            return *this;
        }

    public:
        // Attributes specific to SPHERE ------------------------------------
        IvgVec3d vCenter;
        double dRadius;
        // ------------------------------------------------------------------

    }; // CIVGPoint

    class CIvgEntity
    {
    public:
        CIvgEntity() :
            cName(),
            nListSize(0),
            pFirst(0),
            pLast(0)
        {}

        ~CIvgEntity()
        {
            // note we don't dealloc the IVGNode list!!
        }

        CIvgEntity(const CIvgEntity& t)
        {
            cName = t.cName;
            nListSize = t.nListSize;
            pFirst = t.pFirst;	// just copy the pointers
            pLast = t.pLast;
        }

        CIvgEntity& operator= (const CIvgEntity& t)
        {
            cName = t.cName;
            nListSize = t.nListSize;
            pFirst = t.pFirst;	// just copy the pointers
            pLast = t.pLast;

            return *this;
        }

        void AddNode(CIVGNode *pNode);

		std::string cName;		// Geometry name
        int nListSize;
        CIVGNode *pFirst, *pLast;	// pointers to simple list
    }; // CIvgEntity

    /**
     * @brief Class for loading in IVG files
     */
    class IVGReader
    {
    public:
        IVGReader();
        virtual ~IVGReader();


        int ParseIVGGeometry(char *pArray, std::list<CIvgEntity> &pScene);

    private:

        inline char ReadChar(char * & pInx) {
            char _t;
            memcpy(&_t, pInx, sizeof(char));
            pInx += sizeof(char);
            return _t;
        }

        inline int ReadInt(char * & pInx) {
            int _t;
            memcpy(&_t, pInx, sizeof(int));
            pInx += sizeof(int);
            return _t;
        }

        inline double ReadDouble(char * & pInx) {
            double _t;
            memcpy(&_t, pInx, sizeof(double));
            pInx += sizeof(double);
            return _t;
        }

        int IVGCheckHeader(char **pArray);
        int IVGParseMaterial(char **ppIdx);
        CIVGSphere* AddSphere(char **ppIdx);
        CIVGCone* AddCone(char **ppIdx);
        IvgVec3d* MakeVertexArray(char **ppIdx, int nVertexCount);
        IvgVec3d* MakeNormalArray(char **ppIdx, int nVertexCount);
        CIVGTess* AddTess(char **ppIdx);
        CIVGCurve* AddCurve(char **ppIdx);
        CIVGPoint* AddPoint(char **ppIdx);
    };

}} // end namespaces

#endif
