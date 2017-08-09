/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "HyperSphere.hpp"

#include <rw/math/Constants.hpp>
#include <rw/math/Math.hpp>

#include <cmath>

using rw::geometry::HyperSphere;
using namespace rw::math;

HyperSphere::HyperSphere(unsigned int dimensions):
	_dimensions(dimensions)
{
}

HyperSphere::~HyperSphere() {
}

std::vector<Eigen::VectorXd> HyperSphere::uniformDistributionCartesian(double delta) const {
	const std::vector<Eigen::VectorXd> vWec = uniformDistributionSpherical(delta);

	std::vector<Eigen::VectorXd> sphereVectors;
	for (std::size_t i = 0; i < vWec.size(); i++) {
		const Eigen::VectorXd& wec = vWec[i];
		Eigen::VectorXd SPvec(_dimensions);
		for(unsigned int i = 0; i < _dimensions; i++) {
			double val;
			if(i == _dimensions-1)
				val = 1;
			else
				val = std::cos(wec[i]);
			for(unsigned int j = 0; j < i; j++)
				val *= std::sin(wec[j]);
			SPvec[i] = val;
		}
		sphereVectors.push_back(SPvec);
	}

	return sphereVectors;
}

std::vector<Eigen::VectorXd> HyperSphere::uniformDistributionSpherical(double delta) const {
	std::vector<Eigen::VectorXd> vWec;

	switch (_dimensions) {
	case 2:
	{
		double dw12 = delta;
		double w12 = dw12/2;
		while(w12 <= 2*Pi) {
			Eigen::VectorXd wec(1);
			wec[0] = w12;
			vWec.push_back(wec);
			w12 += dw12;
		}
	}
	break;

	case 3:
	{
		double dw13 = delta;
		double w13 = dw13 / 2;
		while(w13 <= Pi) {
			double dw23 = dw13 / std::sin(w13);
			double w23 = dw23/2;
			while(w23 <= 2*Pi) {
				Eigen::VectorXd wec(2);
				wec[0] = w13;
				wec[1] = w23;
				vWec.push_back(wec);
				w23 += dw23;
			}
			w13 += dw13;
		}
	}
	break;

	case 4:
	{
		double dw14 = delta;
		double w14 = dw14/2;
		while(w14 <= Pi) {
			double dw24 = dw14 / std::sin(w14);
			double w24 = dw24/2;
			while(w24 <= Pi) {
				double dw34 = dw24 / std::sin(w24);
				double w34 = dw34/2;
				while(w34 <= 2*Pi) {
					Eigen::VectorXd wec(3);
					wec[0] = w14;
					wec[1] = w24;
					wec[2] = w34;
					vWec.push_back(wec);
					w34 += dw34;
				}
				w24 += dw24;
			}
			w14 += dw14;
		}
	}
	break;

	case 5:
	{
		double dw15 = delta;
		double w15 = dw15/2.;
		while(w15 <= Pi) {
			double dw25 = dw15 / std::sin(w15);
			double w25 = dw25/2.;
			while(w25 <= Pi) {
				double dw35 = dw25 / std::sin(w25);
				double w35 = dw35/2.;
				while(w35 <= Pi) {
					double dw45 = dw35 / std::sin(w35);
					double w45 = dw45/2.;
					while(w45 <= 2*Pi) {
						Eigen::VectorXd wec(4);
						wec[0] = w15;
						wec[1] = w25;
						wec[2] = w35;
						wec[3] = w45;
						vWec.push_back(wec);
						w45 += dw45;
					}
					w35 += dw35;
				}
				w25 += dw25;
			}
			w15 += dw15;
		}
	}
	break;

	case 6:
	{
		double dw16 = delta;
		double w16 = dw16/2;
		while(w16 <= Pi) {
			double dw26 = dw16 / std::sin(w16);
			double w26 = dw26/2;
			while(w26 <= Pi) {
				double dw36 = dw26 / std::sin(w26);
				double w36 = dw36/2;

				while(w36 <= Pi) {
					double dw46 = dw36 / std::sin(w36);
					double w46 = dw46/2;
					while(w46 <= Pi) {
						double dw56 = dw46 / std::sin(w46);
						double w56 = dw56/2;
						while(w56 <= 2 * Pi) {
							Eigen::VectorXd wec(5);
							wec[0] = w16;
							wec[1] = w26;
							wec[2] = w36;
							wec[3] = w46;
							wec[4] = w56;
							vWec.push_back(wec);
							w56 += dw56;
						}
						w46 += dw46;
					}
					w36 += dw36;
				}
				w26 += dw26;
			}
			w16 += dw16;
		}
	}
	break;

	default:
		break;
	}
	return vWec;
}

unsigned int HyperSphere::getDimensions() const {
	return _dimensions;
}

double HyperSphere::area() const {
	if (_dimensions%2 == 0) {
		return std::pow(Pi,_dimensions/2)*_dimensions/Math::factorial(_dimensions/2);
	} else {
		return std::pow(2.,_dimensions)*std::pow(Pi,(_dimensions-1)/2)*Math::factorial((_dimensions-1)/2)*_dimensions/Math::factorial(_dimensions);
	}
}

double HyperSphere::volume() const {
	if (_dimensions%2 == 0) {
		return std::pow(Pi,_dimensions/2)/Math::factorial(_dimensions/2);
	} else {
		unsigned int doubleFactorial = _dimensions;
		for (unsigned int i = 3; i < _dimensions; i += 2) {
			doubleFactorial *= i;
		}
		return std::pow(2.*Pi,(_dimensions-1)/2)*2./doubleFactorial;
	}
}
