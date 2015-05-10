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

#include <boost/numeric/ublas/matrix.hpp>

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
        
        //! @brief get the boost matrix corresponding to this projection
        const boost::numeric::ublas::bounded_matrix<double, 4, 4> m() const {
           return _matrix;
        }

        //! @brief test if this is a perspective projection
        bool isPerspectiveProjection(){ return _matrix(2,3)<-0.5; }

        //! @brief test if this is a ortographic projection
        bool isOrtographicProjection(){ return _matrix(3,3)>0.5; }

        /**
         * @brief set the projection matrix to an ortographic projection by defining
         * the box with length to all sides (left, right, bottom, top, near and far)
         * @param left [in] length in m to left edge of image
         * @param right [in] length in m to right edge of image
         * @param bottom [in] length in m to bottom edge of image
         * @param top [in] length in m to top edge of image
         * @param zNear [in] length in m to near clipping plane
         * @param zFar [in] length in m to far clipping plane
         */
        void setOrtho(double left, double right,
                        double bottom, double top,
                        double zNear, double zFar);

        //! get ortographic projection. Onli valid if isOrtographicProjection is true
        bool getOrtho(double& left, double& right,
                        double& bottom, double& top,
                        double& zNear, double& zFar) const;

        /**
         * @brief set the projection matrix to the viewing frustum
         * @param left [in] distance in m near cutting plane from center to left edge
         * @param right [in] distance in m near cutting plane from center to right edge
         * @param bottom [in] distance in m near cutting plane from center to bottom edge
         * @param top [in] distance in m near cutting plane from center to top edge
         * @param zNear [in] distance in m along z-axis to near cutting plane
         * @param zFar [in] distance in m along z-axis to far cutting plane
         */
        void setFrustum(double left, double right,
                          double bottom, double top,
                          double zNear, double zFar);

        /**
         * @brief get the projection matrix to the viewing frustum
         * @param left [out] distance in m near cutting plane from center to left edge
         * @param right [out] distance in m near cutting plane from center to right edge
         * @param bottom [out] distance in m near cutting plane from center to bottom edge
         * @param top [out] distance in m near cutting plane from center to top edge
         * @param zNear [out] distance in m along z-axis to near cutting plane
         * @param zFar [out] distance in m along z-axis to far cutting plane
         */
        bool getFrustum(double& left, double& right,
                           double& bottom, double& top,
                           double& zNear, double& zFar) const;

        /**
         * @brief set the projection matrix to perspective projection
         * @param fovy [in] vertical field of view
         * @param aspectRatio [in] aspect ratio between width and height of image
         * @param zNear [in] distance to near cutting plane
         * @param zFar [in] distance to far cutting plane
         */
        void setPerspective(double fovy, double aspectRatio, double zNear, double zFar);

        /**
         * @brief set the projection matrix to perspective projection
         * @param fovy [in] vertical field of view
         * @param width [in] width of image
         * @param height [in] height of image
         * @param zNear [in] distance to near cutting plane
         * @param zFar [in] distance to far cutting plane
         */
        void setPerspective(double fovy, double width, double height, double zNear, double zFar){
            return setPerspective(fovy, (width*1.0)/height, zNear, zFar);
        }

        /**
         * @brief set the projection matrix to perspective projection
         * @param fovy [in] vertical field of view
         * @param aspectRatio [in] aspect ratio between width and height of image
         * @param zNear [in] distance to near cutting plane
         * @param zFar [in] distance to far cutting plane
         */
        bool getPerspective(double& fovy, double& aspectRatio, double& zNear, double& zFar) const;

        /**
         * @brief convert the projection matrix to an OpenGL compatible matrix
         * @param arr [out] array of 16*sizeof(T) with the opengl matrix
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
         * @brief creates a projection matrix with a perspective projection
         * @param fovy [in]
         * @param width [in] of image
         * @param height [in] of image
         * @param zNear [in]
         * @param zFar [in]
         * @return
         */
        static ProjectionMatrix makePerspective(double fovy, double width, double height, double zNear, double zFar);


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

        /**
         * get near and far clipping plane
         * @return
         */
        std::pair<double,double> getClipPlanes() const;
    };

}
}

#endif /* PROJECTIONMATRIX_HPP_ */
