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

#ifndef RW_MATH_PROJECTIONMATRIX_HPP_
#define RW_MATH_PROJECTIONMATRIX_HPP_

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>

#include <rw/common/PropertyMap.hpp>

namespace rw {
namespace math {

    /**
     * @brief projection matrix
     */
    class ProjectionMatrix {
    private:
        boost::numeric::ublas::bounded_matrix<double, 4, 4> _matrix;
    public:

        //! @brief constructor
        ProjectionMatrix(){};
        
        const boost::numeric::ublas::bounded_matrix<double, 4, 4> m() const {
           return _matrix;
        }


        bool isPerspectiveProjection(){ return _matrix(2,3)<-0.5; }
        bool isOrtographicProjection(){ return _matrix(3,3)>0.5; }

        /**
         * @brief set the projection matrix to a ortho graphic projection
         * @param left
         * @param right
         * @param bottom
         * @param top
         * @param zNear
         * @param zFar
         */
        void setOrtho(double left, double right,
                      double bottom, double top,
                      double zNear, double zFar);

        bool getOrtho(double& left, double& right,
                              double& bottom, double& top,
                              double& zNear, double& zFar) const;


        /**
         * @brief set the projection matrix to the viewing frustum
         * @param left
         * @param right
         * @param bottom
         * @param top
         * @param zNear
         * @param zFar
         */
        void setFrustum(double left, double right,
                                 double bottom, double top,
                                 double zNear, double zFar);

        bool getFrustum(double& left, double& right,
                                               double& bottom, double& top,
                                               double& zNear, double& zFar) const;

        /**
         * @brief set the projection matrix to perspective projection
         * @param fovy
         * @param aspectRatio
         * @param zNear
         * @param zFar
         */
        void setPerspective(double fovy, double aspectRatio, double zNear, double zFar);

        bool getPerspective(double& fovy, double& aspectRatio, double& zNear, double& zFar) const;


        /**
         * @brief convert the projection matrix to an OpenGL compatible matrix
         * @param arr
         */
        template<class T>
        void toOpenGLMatrix(T arr[16]){
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++)
                    arr[j + 4 * k] = static_cast<T>(_matrix(j,k));
            }
        }

        /**
         * @brief creates a projection matrix with a perspective projection
         * @param fovy
         * @param aspectRatio
         * @param zNear
         * @param zFar
         * @return
         */
        static ProjectionMatrix makePerspective(double fovy, double aspectRatio, double zNear, double zFar);

        /**
         * @brief creates a projection matrix with a orthographic projection
         * @param left
         * @param right
         * @param bottom
         * @param top
         * @param zNear
         * @param zFar
         * @return
         */
        static ProjectionMatrix makeOrtho(double left, double right,
                                          double bottom, double top,
                                          double zNear, double zFar);

    };

}
}

#endif /* PROJECTIONMATRIX_HPP_ */
