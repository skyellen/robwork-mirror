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

#include <stdio.h>
#include <stdlib.h>
//#include <ctype.h>
//#include <sys/types.h>
#include <math.h>
#include <memory.h>
#include <string.h>

//#include "Vec3d.h"
#include "LoaderIVG.hpp"




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

namespace rw { namespace graphics {

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

    class IVGReader
    {
    public:
    	IVGReader(){};
        virtual ~IVGReader(){};

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


}
}



using namespace rw::graphics;

//////////////////////////////////////////////////////////////////////
// CIvgEntity Metods
//////////////////////////////////////////////////////////////////////

void CIvgEntity::AddNode(CIVGNode *pNode)
{
	// ASSERT(pNode)

	if (pFirst == NULL) // first time
	{
		pFirst = pLast = pNode;
		pNode->pNext = NULL;
	}
	else
	{
		pLast->pNext = pNode;
		pLast = pNode;
		pNode->pNext = NULL;
	}

	nListSize++;

} // AddNode

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

LoaderIVG::LoaderIVG()
{
}

LoaderIVG::~LoaderIVG()
{
}


Model3D::Ptr LoaderIVG::load(const std::string &filename)
{
	FILE *fp;
	if(! (fp = fopen(filename.c_str(), "rb")) )
	{
		RW_THROW("Couldn't open '" << filename << "'");
	}


   	//Start by storing the current locale. This is retrieved by passing NULL to setlocale	
	std::string locale = setlocale(LC_ALL, NULL); 
	setlocale(LC_ALL, "C");

	long lnStartPos = ftell(fp);
	fseek(fp, 0, SEEK_END);
	long lnEndPos = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	const long lnArraySize = lnEndPos - lnStartPos;
	char *pc_Buf = new char[lnArraySize];

	if (fread(pc_Buf, lnArraySize, 1, fp) == 0) {
	    fclose(fp);
        setlocale(LC_ALL, locale.c_str());
        delete[] pc_Buf;
        RW_THROW("Failed to read bytes from file "<<filename);
	    return NULL;
	}
	fclose(fp);

	IVGReader reader;
	std::list<CIvgEntity> scene;

	/*int n_Ent = */reader.ParseIVGGeometry(pc_Buf, scene);


	// TODO: convert the below stuff to a model3d format and return it
	Model3D::Ptr model = ownedPtr( new Model3D() );




    delete[] pc_Buf;
    setlocale(LC_ALL, locale.c_str());

	return model;
}


int IVGReader::ParseIVGGeometry(char *pArray, std::list<CIvgEntity> &pScene)
{
	//int retVal = -1;		// handle id to return
	char *pIdx = pArray;	// local poniter in pArray (like file pointer)
	int version;			// ivg file version

	if( (version = IVGCheckHeader(&pIdx) ) == -1)
		return -1; // no valid ivg file

	if (version == 30) // in version 3.0 we have to parse the material part first
	{
		if ( IVGParseMaterial(&pIdx) == -1 ) return -4;
	}


	// read on from stream

	int nEnts = ReadInt(pIdx); // read nr of entities from the file

	// make an array to hold the entities in the ivg-file

	for (int ent=0; ent<nEnts; ent++) // for each entity
	{
		CIvgEntity ivgEntity;

		char nAcisType = ReadChar(pIdx);

		int parentID = ReadInt(pIdx);

		unsigned int nTypeMask;
		memcpy(&nTypeMask, pIdx, sizeof(unsigned int));
		pIdx += sizeof(unsigned int);
		if(nTypeMask == 0) nTypeMask = (unsigned int)-1;

		int entityColor = ReadInt(pIdx);

		int viewDepPixSize = ReadInt(pIdx);

		double entVDPx = ReadDouble(pIdx);
		double entVDPy = ReadDouble(pIdx);
		double entVDPz = ReadDouble(pIdx);

		int nSubCount = ReadInt(pIdx);

		CIVGBody *ivgBody = new CIVGBody;

		ivgBody->nSubCount = nSubCount;
		ivgBody->nType = nAcisType;
		ivgBody->nId = parentID;
		ivgBody->nTypeMask = nTypeMask;
		ivgBody->nColor = entityColor;
		ivgBody->nPixelSize = viewDepPixSize;
		ivgBody->translation.set(entVDPx, entVDPy, entVDPz);

		ivgEntity.AddNode(ivgBody);

		for (int m=0; m<nSubCount; m++) // for each sub-entity
		{
			CIVGNode *ivgNode = NULL;

			char nSubType = ReadChar(pIdx);
			int childID = ReadInt(pIdx);

			int nSubMask = ReadInt(pIdx);
			if(nSubMask == 0)
			{
				// nSubMask = nTypeMask;
				nSubMask = 1 << nSubType;
			}

			int subColor = ReadInt(pIdx);

			// Note that all ponts are colleced into a list!
			if (nSubType == IVG_TYPE_POINT)
			{
				/*int pixSize = */ReadInt(pIdx);

				IvgVec3d p_dPt; // = new osg::Vec4;

				p_dPt[0] = ReadDouble(pIdx);
				p_dPt[1] = ReadDouble(pIdx);
				p_dPt[2] = ReadDouble(pIdx);
				// p_dPt[3] = childID;

				//m_vPointStack.push_back(p_dPt);
				// add x,y,z to a stl_vector _vPointStack
			}
			else // add the type information, for picking feedback etc..
			{
				int pixSize = ReadInt(pIdx);

				double subVDPx = ReadDouble(pIdx);
				double subVDPy = ReadDouble(pIdx);
				double subVDPz = ReadDouble(pIdx);

				if (nSubType == IVG_TYPE_POLY)
				{
	//				MessageBox(NULL,_T("Polyformat not supported "),_T("RView Warning"),MB_OK);
				}
				else if (nSubType == IVG_TYPE_TESS)
				{
					ivgNode = AddTess(&pIdx);
				}
				else if (nSubType == IVG_TYPE_WIRE)
				{
					ivgNode = AddCurve(&pIdx);
				}
				else if(nSubType == IVG_TYPE_SHADED_WIRE)
				{
					ivgNode = AddCurve(&pIdx);
				}
				else if(nSubType == IVG_TYPE_SPHERE)
				{
					ivgNode = AddSphere(&pIdx);
				}
				else if (nSubType == IVG_TYPE_CONE)
				{
					ivgNode = AddCone(&pIdx);
				}
				else
				{
	//				MessageBox(NULL,_T("Wrong ivg format"),_T("RView Error"),MB_OK);
					return -5;
				}

				// fill in the sub entity general data
				if ( ivgNode )
				{
					ivgNode->nType = nSubType;
					ivgNode->nId = childID;
					ivgNode->nTypeMask = nSubMask;
					ivgNode->nColor = subColor;
					ivgNode->nPixelSize = pixSize;
					ivgNode->translation.set(subVDPx, subVDPy, subVDPz);

					ivgEntity.AddNode(ivgNode);
				}
			}
		} // for each subentity

		// if (! m_vPointStack.empty()) // point list contains any data


		pScene.push_back(ivgEntity);

   } // for each entity

	return nEnts;

} // ParseIVGGeometry

int IVGReader::IVGCheckHeader(char **pArray)
{
  char header[8];
  char cVersion[3];
  int version;

  for(int i=0;i<7;i++){
	header[i] = (*pArray)[i];
  }
  cVersion[0] = (*pArray)[4];
  cVersion[1] = (*pArray)[6];
  cVersion[2] = 0;
  version = atoi(cVersion);
  header[4] = 0;
  header[7] = 0;
  if(!strcmp(header,"IVG#")){
	  if( ! ( (version == 20) || (version == 21) || (version == 30) ) )
	  {
		version = -1;
	  }
	  else
		*pArray += sizeof(char)*7;
  }
  else{
	version = -1;
  }

  return version;

} // IVGCheckHeader

int IVGReader::IVGParseMaterial(char **ppIdx)
{
	int nMaterials;

	// read nr. of user defined materials
	memcpy(&nMaterials, *ppIdx, sizeof(int));
	*ppIdx += sizeof(int);

	for (int i=0; i < nMaterials; i++) // for each material
	{
		char nRGBType;

		memcpy(&nRGBType, *ppIdx, sizeof(char));
		*ppIdx += sizeof(char);

		if ( nRGBType == 0 ) // RGBA definition
		{
			// read the four values
			double r, g, b, a;

			memcpy(&r, *ppIdx, sizeof(double));
			*ppIdx += sizeof(double);

			memcpy(&g, *ppIdx, sizeof(double));
			*ppIdx += sizeof(double);

			memcpy(&b, *ppIdx, sizeof(double));
			*ppIdx += sizeof(double);

			memcpy(&a, *ppIdx, sizeof(double));
			*ppIdx += sizeof(double);

			// this ensures that the RGB is in the material definition
			//pScene->pApperance->GetRGBMaterial(r, g, b);
			//materialIndex = pScene->pApperance->GetRGBIndex(r, g, b);

			// add the index to the MaterialMap
			//m_MaterialMap.insert(std::pair<long, long>(i+1, materialIndex));

		}
		else if ( nRGBType == 1 ) // Full material definition
		{
			// read 17 values, that gives the material
			*ppIdx += sizeof(double)*17;
		}

	} // for each material

	return 1;

} // IVGParseMaterial

CIVGSphere* IVGReader::AddSphere(char **ppIdx)
{
	CIVGSphere *ivgSphere = new CIVGSphere;

	double x,y,z,radius;

	memcpy(&x, *ppIdx, sizeof(double));
	*ppIdx += sizeof(double);

	memcpy(&y, *ppIdx, sizeof(double));
	*ppIdx += sizeof(double);

	memcpy(&z, *ppIdx, sizeof(double));
	*ppIdx += sizeof(double);

	memcpy(&radius, *ppIdx, sizeof(double));
	*ppIdx += sizeof(double);

	ivgSphere->dRadius = radius;
	ivgSphere->vCenter.set(x, y, z);

	return ivgSphere;

} // AddSphere


CIVGCone* IVGReader::AddCone(char **ppIdx)
{
	double bX,bY,bZ,bRadius;
	double tX,tY,tZ,tRadius;
	char showBottom,showTop;

	CIVGCone *ivgCone = new CIVGCone;

	memcpy(&bX, *ppIdx, sizeof(double));
	*ppIdx += sizeof(double);

	memcpy(&bY, *ppIdx, sizeof(double));
	*ppIdx += sizeof(double);

	memcpy(&bZ, *ppIdx, sizeof(double));
	*ppIdx += sizeof(double);

	memcpy(&bRadius, *ppIdx, sizeof(double));
	*ppIdx += sizeof(double);

	memcpy(&showBottom, *ppIdx, sizeof(char));
	*ppIdx += sizeof(char);

	memcpy(&tX, *ppIdx, sizeof(double));
	*ppIdx += sizeof(double);

	memcpy(&tY, *ppIdx, sizeof(double));
	*ppIdx += sizeof(double);

	memcpy(&tZ, *ppIdx, sizeof(double));
	*ppIdx += sizeof(double);

	memcpy(&tRadius, *ppIdx, sizeof(double));
	*ppIdx += sizeof(double);

	memcpy(&showTop, *ppIdx, sizeof(char));
	*ppIdx += sizeof(char);

	if (showBottom)
		ivgCone->bShowBottom = true;
	else
		ivgCone->bShowBottom = false;

	if (showTop)
		ivgCone->bShowTop = true;
	else
		ivgCone->bShowTop = false;

	ivgCone->dBottomRadius = bRadius;
	ivgCone->dTopRadius = tRadius;
	ivgCone->vBottom.set(bX, bY, bZ);
	ivgCone->vTop.set(tX, tY, tZ);

	return ivgCone;

} // AddCone

IvgVec3d* IVGReader::MakeVertexArray(char **ppIdx, int nVertexCount)
{

	IvgVec3d* cset = new IvgVec3d[nVertexCount];

	for (int j= 0;j<nVertexCount; j++)
	{
		double dX, dY, dZ;

		memcpy(&dX, *ppIdx, sizeof(double));
		*ppIdx += sizeof(double);

		memcpy(&dY, *ppIdx, sizeof(double));
		*ppIdx += sizeof(double);

		memcpy(&dZ, *ppIdx, sizeof(double));
		*ppIdx += sizeof(double);
		cset[j].set(dX,dY,dZ);
   	}

	return cset;
} // MakeVertexArray

IvgVec3d* IVGReader::MakeNormalArray(char **ppIdx, int nVertexCount)
{

	IvgVec3d* cset = new IvgVec3d[nVertexCount];

	for (int j=0; j<nVertexCount; j++){
		double dX, dY, dZ;

		memcpy(&dX, *ppIdx, sizeof(double));
		*ppIdx += sizeof(double);

		memcpy(&dY, *ppIdx, sizeof(double));
		*ppIdx += sizeof(double);

		memcpy(&dZ, *ppIdx, sizeof(double));
		*ppIdx += sizeof(double);

		cset[j].set(dX,dY,dZ);
	}
	return cset;
} // MakeNormalArray


CIVGTess* IVGReader::AddTess(char **ppIdx)
{
	int nVertexCount,nNormalCount, nTriangleCount;;

	CIVGTess *ivgTess = new CIVGTess;

	memcpy(&nVertexCount, *ppIdx, sizeof(int));
	*ppIdx += sizeof(int);

	memcpy(&nNormalCount, *ppIdx, sizeof(int));
	*ppIdx += sizeof(int);

	memcpy(&nTriangleCount, *ppIdx, sizeof(int));
	*ppIdx += sizeof(int);

	ivgTess->nVertices = nVertexCount;
	ivgTess->nNormals = nNormalCount;
	ivgTess->nTriangles = nTriangleCount;

	ivgTess->pVertices = MakeVertexArray(ppIdx, nVertexCount);

	ivgTess->pNormals = MakeNormalArray(ppIdx, nNormalCount);

	// make array's for the index reference
	Triangle *pTriangle = new Triangle[nTriangleCount];

	unsigned int idx[6];

	for (int i=0; i<nTriangleCount; i++){
		for (int k=0; k<6; k++) {
			memcpy(&(idx[k]), *ppIdx, sizeof(int));
			*ppIdx += sizeof(int);
		}

		pTriangle[i].vInx[0] = idx[0];
		pTriangle[i].vInx[1] = idx[2];
		pTriangle[i].vInx[2] = idx[4];

		pTriangle[i].nInx[0] = idx[1];
		pTriangle[i].nInx[1] = idx[3];
		pTriangle[i].nInx[2] = idx[5];
	}

	ivgTess->pTriangles = pTriangle;

	return ivgTess;
} // AddTess


CIVGCurve* IVGReader::AddCurve(char **ppIdx)
{
	int nVertexCount;
	double line_thickness;

	CIVGCurve *ivgCurve = new CIVGCurve;

	memcpy(&line_thickness, *ppIdx, sizeof(double));
	*ppIdx += sizeof(double);

	memcpy(&nVertexCount, *ppIdx, sizeof(int));
	*ppIdx += sizeof(int);

	if (nVertexCount == 0)
		return NULL;

	double X,Y,Z;

	IvgVec3d* cset = new IvgVec3d[nVertexCount];

	for (int j= 0; j<nVertexCount; j++) {

		memcpy(&X, *ppIdx, sizeof(double));
		*ppIdx += sizeof(double);

		memcpy(&Y, *ppIdx, sizeof(double));
		*ppIdx += sizeof(double);

		memcpy(&Z, *ppIdx, sizeof(double));
		*ppIdx += sizeof(double);

		cset[j].set((float)X,(float)Y,(float)Z);
	}

	ivgCurve->nVertices = nVertexCount;
	ivgCurve->pVertices = cset;
	ivgCurve->dLineThickness = line_thickness;

	return ivgCurve;

} // AddCurve

CIVGPoint* IVGReader::AddPoint(char **ppIdx)
{
//	int nVertexCount;

	CIVGPoint *ivgPoint = new CIVGPoint;

	return ivgPoint;

/*
	int nVertexCount = m_vPointStack.size();

	osg::Vec3* cset = new osg::Vec3[nVertexCount];
	data->m_pMem->ref_data(cset);	// add mem reference

	int* pointLengthList = new int[nVertexCount];
	data->m_pMem->ref_data(pointLengthList);	// add mem reference

	PointStack::iterator p;
	int j = 0;
	for (p = m_vPointStack.begin(); p != m_vPointStack.end(); p++, j++)
	{
		cset[j].set((float)p->x(),(float)p->y(),(float)p->z());
		pointLengthList[j] = (int)p->w();
	}

	osg::GeoSet* m_pntSet = new osg::GeoSet();
	m_pntSet->setSupportsDisplayList(false);

	if (pScene->m_bShowPoints)
		m_pntSet->setPrimType( osg::GeoSet::POINTS);
	else
		m_pntSet->setPrimType( osg::GeoSet::NO_TYPE);
	m_pntSet->setNumPrims(nVertexCount);
	m_pntSet->setCoords(cset);
	m_pntSet->setNormalBinding( osg::GeoSet::BIND_OFF  );
	m_pntSet->setPrimLengths( pointLengthList );

	m_pntSet->setStateSet( colorState );
	osg::Geode* geode = new osg::Geode();
	geode->addDrawable( m_pntSet );
	geode->setUserData(data);

	osg::Point *point = new osg::Point;
	point->setSize(2.0);
	colorState->setAttribute(point);

	osg::Switch *sw = new osg::Switch;

*/
} // AddPoint
