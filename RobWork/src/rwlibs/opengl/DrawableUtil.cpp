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


#include "DrawableUtil.hpp"

using namespace rwlibs::opengl;

void DrawableUtil::transform3DToGLTransform(
    const rw::math::Transform3D<float>& transform,
    GLfloat* gltrans)
{
    for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++)
            gltrans[j + 4 * k] =
                transform(j,k);

        gltrans[12 + j] =
            transform(j, 3);
    }

    gltrans[3] = gltrans[7] = gltrans[11] = 0;
    gltrans[15] = 1;
}

void DrawableUtil::transform3DToGLTransform(
            const rw::math::Transform3D<double>& transform,
            GLfloat* gltrans)
{
    for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++)
            gltrans[j + 4 * k] =
                (float)transform(j,k);

        gltrans[12 + j] =
            (float)transform(j, 3);
    }

    gltrans[3] = gltrans[7] = gltrans[11] = 0;
    gltrans[15] = 1;
}


void DrawableUtil::transform3DToGLTransform(
            const rw::math::Transform3D<double>& transform,
            GLdouble* gltrans)
{
    for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++)
            gltrans[j + 4 * k] = transform(j,k);

        gltrans[12 + j] = transform(j, 3);
    }

    gltrans[3] = gltrans[7] = gltrans[11] = 0;
    gltrans[15] = 1;
}
