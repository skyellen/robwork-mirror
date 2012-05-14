// RectangularSweptSphere.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "RectDist.hpp"			//New implementation
#include "RectDist.h"			//PQP implementation
#include "rw/math.hpp"
#include "rw/common/Timer.hpp"
#include "time.h"				//Used for seed in random
#include <iostream>				//Used to print errors to file
#include <fstream>

using namespace rw::common;

typedef rw::math::Vector3D<double> Vector;
typedef rw::math::Transform3D<double> Transform;
typedef rw::math::Rotation3D<double> Rotation;

void testRectDist()
{
	std::ofstream errorRectFile;
	errorRectFile.open("errorRect.txt");

	//Test boundaries
	const double positionBoundaries		= 1.1;
	const double minRectangleSize		= .1;
	const double maxRectangleSize		= .6;

	const double correctnessTestEps		= 1e-10;

	//<- Correctness tests ->
	Transform correctnessTestTransform;
	double correctnessTestRectangleSize[2][2];

	double correctnessTestPQPRotationMatrix[3][3];
	double correctnessTestPQPPositionVector[3];
	double correctnessTestPQPRectangleASize[2];
	double correctnessTestPQPRectangleBSize[2];

	int correctnessTestCorrectCounter = 0;

	const int nCorrectnessTests = 1000000;
	for(int i = 0;i<nCorrectnessTests;i++)
	{
		correctnessTestTransform.P() = Vector(Math::ran(-positionBoundaries,positionBoundaries),Math::ran(-positionBoundaries,positionBoundaries),Math::ran(-positionBoundaries,positionBoundaries));
		correctnessTestTransform.R() = Math::ranRotation3D<double>();

		correctnessTestRectangleSize[0][0]		= Math::ran(minRectangleSize,maxRectangleSize);
		correctnessTestRectangleSize[0][1]		= Math::ran(minRectangleSize,maxRectangleSize); 
		correctnessTestRectangleSize[1][0]		= Math::ran(minRectangleSize,maxRectangleSize); 
		correctnessTestRectangleSize[1][1]		= Math::ran(minRectangleSize,maxRectangleSize); 

		correctnessTestPQPRotationMatrix[0][0]	= correctnessTestTransform.R()(0,0);
		correctnessTestPQPRotationMatrix[1][0]	= correctnessTestTransform.R()(1,0);
		correctnessTestPQPRotationMatrix[2][0]	= correctnessTestTransform.R()(2,0);

		correctnessTestPQPRotationMatrix[0][1]	= correctnessTestTransform.R()(0,1);
		correctnessTestPQPRotationMatrix[1][1]	= correctnessTestTransform.R()(1,1);
		correctnessTestPQPRotationMatrix[2][1]	= correctnessTestTransform.R()(2,1);

		correctnessTestPQPRotationMatrix[0][2]	= correctnessTestTransform.R()(0,2);
		correctnessTestPQPRotationMatrix[1][2]	= correctnessTestTransform.R()(1,2);
		correctnessTestPQPRotationMatrix[2][2]	= correctnessTestTransform.R()(2,2);

		correctnessTestPQPPositionVector[0]		= correctnessTestTransform.P()(0);
		correctnessTestPQPPositionVector[1]		= correctnessTestTransform.P()(1);
		correctnessTestPQPPositionVector[2]		= correctnessTestTransform.P()(2);

		correctnessTestPQPRectangleASize[0]		= correctnessTestRectangleSize[0][0];
		correctnessTestPQPRectangleASize[1]		= correctnessTestRectangleSize[0][1];

		correctnessTestPQPRectangleBSize[0]		= correctnessTestRectangleSize[1][0];
		correctnessTestPQPRectangleBSize[1]		= correctnessTestRectangleSize[1][1];

		if(
			//Test if distance btw new implementation and PQP is more than epsilon
			abs(	distanceBetweenRectangles3(
						correctnessTestTransform.R(),
						correctnessTestTransform.P(),
						correctnessTestRectangleSize) 
					- 
					RectDist(
						correctnessTestPQPRotationMatrix,
						correctnessTestPQPPositionVector,
						correctnessTestPQPRectangleASize,
						correctnessTestPQPRectangleBSize)	) < correctnessTestEps
						)
		{
			correctnessTestCorrectCounter++;
		}
		else
		{
			//If error -> display size of error
			std::cout	<<	distanceBetweenRectangles3(
								correctnessTestTransform.R(),
								correctnessTestTransform.P(),
								correctnessTestRectangleSize) 
								- 
							RectDist(
								correctnessTestPQPRotationMatrix,
								correctnessTestPQPPositionVector,
								correctnessTestPQPRectangleASize,
								correctnessTestPQPRectangleBSize) 
						<< std::endl;
		}
	}

	std::cout << "Correctness tests" << std::endl;
	std::cout << correctnessTestCorrectCounter << " correct out of " <<  nCorrectnessTests << " tests" << std::endl << std::endl;

	//<- Time tests ->
	//Number of rectangles in test cycle
	const int nRectSets = 1000;

	double rectangleSizes[nRectSets][2][2];

	Transform transform;
	Transform transforms[nRectSets];

	for(int i = 0;i < nRectSets;i++)
	{
		rectangleSizes[i][0][0] = Math::ran(minRectangleSize,maxRectangleSize);
		rectangleSizes[i][0][1] = Math::ran(minRectangleSize,maxRectangleSize); 
		rectangleSizes[i][1][0] = Math::ran(minRectangleSize,maxRectangleSize); 
		rectangleSizes[i][1][1] = Math::ran(minRectangleSize,maxRectangleSize); 
	}

	for(int i = 0;i < nRectSets;i++)
	{
		transforms[i].P() = Vector(Math::ran(-positionBoundaries,positionBoundaries),Math::ran(-positionBoundaries,positionBoundaries),Math::ran(-positionBoundaries,positionBoundaries));
		transforms[i].R() = Math::ranRotation3D<double>();
		
	}

	//Adapt position vector and rotation matrix to PQP types
	double pqpRotation[nRectSets][3][3];
	double pqpPosition[nRectSets][3];
	double pqpASize[nRectSets][2];
	double pqpBSize[nRectSets][2];

	for(int i = 0;i<nRectSets;i++)
	{
		pqpRotation[i][0][0] = transforms[i].R()(0,0);
		pqpRotation[i][1][0] = transforms[i].R()(1,0);
		pqpRotation[i][2][0] = transforms[i].R()(2,0);

		pqpRotation[i][0][1] = transforms[i].R()(0,1);
		pqpRotation[i][1][1] = transforms[i].R()(1,1);
		pqpRotation[i][2][1] = transforms[i].R()(2,1);

		pqpRotation[i][0][2] = transforms[i].R()(0,2);
		pqpRotation[i][1][2] = transforms[i].R()(1,2);
		pqpRotation[i][2][2] = transforms[i].R()(2,2);

		pqpPosition[i][0] = transforms[i].P()(0);
		pqpPosition[i][1] = transforms[i].P()(1);
		pqpPosition[i][2] = transforms[i].P()(2);

		pqpASize[i][0] = rectangleSizes[i][0][0];
		pqpASize[i][1] = rectangleSizes[i][0][1];

		pqpBSize[i][0] = rectangleSizes[i][1][0];
		pqpBSize[i][1] = rectangleSizes[i][1][1];
	}

	//Number of tests cycles
	const int nTests = 10000;
	
	//Same timer for both implementations
	Timer time;

	//Result variables to make sure compiler optimizations does not remove code
	double pqpResult, newImplementationResult;

	//Time test of new implementation
	time.resetAndResume();
	for(int i = 0;i<nTests;i++)
		for(int j = 0;j<nRectSets;j++)
			newImplementationResult = distanceBetweenRectangles3(transforms[j].R(),transforms[j].P(),rectangleSizes[j]);

	time.pause();
	double testTimeNewImplementation = time.getTime();

	//Time test of PQP implementation
	time.resetAndResume();
	for(int i = 0;i<nTests;i++)
		for(int j = 0;j<nRectSets;j++)
			pqpResult = RectDist(pqpRotation[j],pqpPosition[j],pqpASize[j],pqpBSize[j]);

	time.pause();
	double testTimePQPImplementation = time.getTime();

	std::cout << "Time of new implementation:" << std::endl;
	std::cout << testTimeNewImplementation << std::endl << std::endl;

	std::cout << "Time of PQP implementation:" << std::endl;
	std::cout << testTimePQPImplementation << std::endl << std::endl;

	std::cout << "Difference in percent:" << std::endl;
	std::cout << testTimeNewImplementation/testTimePQPImplementation*100.0 << std::endl;
	
	//Print results of last result to avoid compiler optimization on time tests
	std::cout << std::endl << pqpResult << std::endl;
	std::cout << std::endl << newImplementationResult << std::endl;
}

int _tmain(int argc, _TCHAR* argv())
{
	//Choose new seed to get new test set for each run
	Math::seed(time(NULL));

	//Rectangle Distance Query Test
	testRectDist();
	
	return 0;
}

