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

#ifndef RW_DRAWABLE_RENDERSMOOTHSKIN_HPP_
#define RW_DRAWABLE_RENDERSMOOTHSKIN_HPP_

#include <rw/graphics/Render.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace kinematics { class State; } }

namespace rwlibs { namespace opengl {

    /** @addtogroup opengl */
    /*@{*/
	//! @file RenderSmoothSkin.hpp

	/**
	 * @brief renders skin using vertice weights in relation to the bone structure.
	 *
	 * Each vertice has a number of bones attached. These attached bones influence the position of
	 * the vertice.
	 *
	 * The transformation of each vertice is weighted against the transformation of each attached bone. Such
	 * that the vertice position is defined by
	 * \f$ v' = \sum w_i v . M_{[i]} \f$
	 * where M is the transform from \b Base frame to the the i'th attached bone (which is not the i'th bone).
	 *
	 * @note The number of attached bones to a single vertice should not be too large <5.
	 */
	class RenderSmoothSkin: public rw::graphics::Render {
		//! @brief Type for vertice weights.
	    typedef std::pair<int, float> VerticeWeight;

	    //! @brief Type for bone weights.
	    typedef std::vector<VerticeWeight> BoneWeights;

	    //! @brief constructor
		RenderSmoothSkin(rw::geometry::IndexedTriMesh<>::Ptr mesh,
		                 rw::kinematics::Frame* base,
		                 std::vector<rw::kinematics::Frame*>& bones,
		                 std::vector<BoneWeights>& weights);

		//! @brief destructor	
		virtual ~RenderSmoothSkin();

		//! @copydoc RenderSmoothSkin()
		void init(rw::geometry::IndexedTriMesh<>::Ptr mesh,
		     rw::kinematics::Frame* base,
		     std::vector<rw::kinematics::Frame*>& bones,
		     std::vector<BoneWeights>& weights);

        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
		void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                  rw::graphics::DrawableNode::DrawType type,
                  double alpha) const;

		/**
		 * @brief Update the mesh.
		 * @param state [in] new state.
		 */
        void update(const rw::kinematics::State& state);

	private:
		struct VerticeWeightINL {
		    uint8_t boneIdx;
		    float weight;
		};

		std::vector< uint8_t > _weights;
		std::vector<rw::math::Transform3D<> > _transforms;

        rw::kinematics::Frame* _base;
        std::vector<rw::kinematics::Frame*> _bones;

        rw::geometry::IndexedTriMesh<>::Ptr _mesh;
        std::vector<rw::math::Vector3D<> > _vertices;



	};


    /*@}*/
}}

#endif /* RENDERSKIN_HPP_ */
