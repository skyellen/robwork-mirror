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

#ifndef RW_MODELS_DEFORMABLEOBJECT_HPP_
#define RW_MODELS_DEFORMABLEOBJECT_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/kinematics/Stateless.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/graphics/Model3D.hpp>

#include "Object.hpp"

#include <vector>

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
       @brief The deformable object is an object that contain a deformable mesh. Deformations
       are part of the state object and they are modeled/controlled through control nodes.
       each control node correspond to a vertice in the mesh. All vertices are described relative to the base
       frame of the object.


    */
    class DeformableObject: public Object
    {
    public:
    	//! smart pointer type
        typedef rw::common::Ptr<DeformableObject> Ptr;

        /**
         * @brief constructor - constructs a deformable mesh with a specific number of control nodes
         * and without any faces. Both geometry and model are created based on nodes.
         * @param baseframe [in] base frame of object
         * @param nr_of_nodes [in] the number of controlling nodes in the deformable object
         */
        DeformableObject(rw::kinematics::Frame* baseframe, int nr_of_nodes);

        /**
         * @brief constructor - control nodes are taken as vertices in the Model3D. Vertices that
         * are equal are merged into the same control node. All faces of the model are used to
         * define faces of the deformable object.
         *
         * geometry will be created based on model information
         *
         * @note only triangle faces are currently supported.
         *
         * @param baseframe [in] base frame of object
         * @param model [in]
         */
        DeformableObject(rw::kinematics::Frame* baseframe, rw::graphics::Model3D::Ptr model);

        /**
         * @brief constructor - control nodes are taken from a triangle mesh generated from triangulating the
         * geometry. Vertices that
         * are equal are merged into the same control node. All faces of the geometry are used to
         * define faces of the deformable object.
         *
         * model will be created based on geometry information
         *
         * @param baseframe [in] base frame of object
         * @param geom [in] geometry to define the faces and nodes
         */
        DeformableObject(rw::kinematics::Frame* baseframe, rw::geometry::Geometry::Ptr geom);

        //! destructor
        virtual ~DeformableObject();

        /**
         * @brief get a specific node from the state
         * @param id [in] id of the node to fetch
         * @param state [in] current state
         * @return handle to manipulate a node in the given state.
         */
        rw::math::Vector3D<float>& getNode(int id, rw::kinematics::State& state) const;
        //! @copydoc getNode
        const rw::math::Vector3D<float>& getNode(int id, const rw::kinematics::State& state) const;

        /**
         * @brief set the value of a specific node in the state.
         * @param id [in] id of the node
         * @param v [in] value to set.
         * @param state [in] state in which to set the value.
         */
        void setNode(int id, const rw::math::Vector3D<float>& v, rw::kinematics::State& state);

        /**
         * get the number of controlling nodes of this deformable object.
         * @param state [in]
         * @return
         */
        size_t getNrNodes(const rw::kinematics::State& state) const ;
        size_t getNrNodes() const;

        /**
         * @brief get all faces of this soft body
         * @return list of indexed triangles - indeces point to vertices/nodes
         */
        const std::vector<rw::geometry::IndexedTriangle<> >& getFaces() const;

        /**
         * @brief add a face to three existing nodes
         * @param node1 [in] idx of node 1
         * @param node2 [in] idx of node 2
         * @param node3 [in] idx of node 3
         */
        void addFace(unsigned int node1, unsigned int node2, unsigned int node3);

        /**
         * @brief return a triangle mesh representing the softbody in the current state
         * \b cstate
         * @param cstate
         */
        rw::geometry::IndexedTriMesh<float>::Ptr getMesh(rw::kinematics::State& cstate);


 	    /**
 	     * @brief get mass in Kg of this object
 	     * @return mass in kilo grams
 	     */
 	    double getMass(rw::kinematics::State& state) const;

 	    /**
 	     * @brief get center of mass of this object
 	     * @param state [in] the state in which to get center of mass
 	     * @return
 	     */
 	    rw::math::Vector3D<> getCOM(rw::kinematics::State& state) const;

 	    /**
 	     * @brief returns the inertia matrix of this body calculated around COM with the orientation
 	     * of the base frame.
 	     */
 	    rw::math::InertiaMatrix<> getInertia(rw::kinematics::State& state) const;

 	    /**
 	     * @brief updates the model with the current state of the deformable model
 	     * @param model [in/out] model to be updated
 	     * @param state
 	     */
 	    void update(rw::graphics::Model3D::Ptr model, const rw::kinematics::State& state);

 	    //void update(rw::geometry::Geometry::Ptr geom, const rw::kinematics::State& state);

    protected:
        friend class WorkCell;

        const std::vector<rw::geometry::Geometry::Ptr>& doGetGeometry(const rw::kinematics::State& state) const;
        const std::vector<rw::graphics::Model3D::Ptr>& doGetModels(const rw::kinematics::State& state) const;

        class DeformableObjectCache: public rw::kinematics::StateCache {
        public:
        	typedef rw::common::Ptr<DeformableObjectCache> Ptr;
        	std::vector<rw::math::Vector3D<float> > _nodes;
        	std::vector<rw::graphics::Model3D::Ptr> _models;
        	std::vector<rw::geometry::Geometry::Ptr> _geoms;

        	DeformableObjectCache(int nr_of_nodes):
        		_nodes(nr_of_nodes, rw::math::Vector3D<float>(0,0,0) )
        	{
        	};

        	size_t size() const{ return _nodes.size()*3*sizeof(float); };

            virtual rw::common::Ptr<StateCache> clone() const{
            	DeformableObjectCache::Ptr cache = rw::common::ownedPtr( new DeformableObjectCache(*this) );
            	cache->_models.resize(0);
            	return cache;
            };
        };


    private:

        rw::kinematics::StatelessData<int> _rstate;
        rw::common::Ptr<rw::geometry::IndexedTriMeshN0<float> > _mesh;
        std::vector< std::pair<int,rw::kinematics::MovableFrame*> > _frames;
        rw::graphics::Model3D::Ptr _model;
        rw::geometry::Geometry::Ptr _geom;
    };

    /*@}*/
}}

#endif /* OBJECT_HPP_ */
