//
// IVGReader, class and struct to read ivg files
//

#include <stdio.h>
#include <stdlib.h>
//#include <ctype.h>
//#include <sys/types.h>
#include <math.h>
#include <memory.h>
#include <string.h>

//#include "Vec3d.h"
#include "IVGReader.hpp"

using namespace rwlibs::drawable;

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

IVGReader::IVGReader()
{

}

IVGReader::~IVGReader()
{

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
