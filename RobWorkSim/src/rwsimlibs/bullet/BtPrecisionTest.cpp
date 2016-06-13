/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <iostream>

#include <LinearMath/btVector3.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>

int main() {
	const btSphereShape sphere(1);
	btVector3 res;
	sphere.calculateLocalInertia(1,res); // linking fails with undefined symbol if BT_USE_DOUBLE_PRECISION is wrong
#ifdef BT_USE_DOUBLE_PRECISION
	std::cout << "DOUBLE" << std::endl;
#else
	std::cout << "SINGLE" << std::endl;
#endif
	return 0;
}
