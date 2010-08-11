/*
 * TRIDeviller.hpp
 *
 *  Created on: 18-02-2009
 *      Author: jimali
 */

#ifndef TRIDEVILLER_HPP_
#define TRIDEVILLER_HPP_

#include "DataTypes.hpp"

/**
 * @brief tests if two triangles are intersecting using devillers method.
 */
class TRIDeviller {
public:
	static bool inCollision(float *P1, float *P2, float *P3,
							float *Q1, float *Q2, float *Q3);

	static bool inCollision(const WpTriangle& P, const WpTriangle& Q);

};


#endif /* TRIDEVILLER_HPP_ */
