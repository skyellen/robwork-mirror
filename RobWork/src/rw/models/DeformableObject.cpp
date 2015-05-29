#include "DeformableObject.hpp"

#include <rw/geometry/TriangleUtil.hpp>

using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::models;
using namespace rw::math;


DeformableObject::DeformableObject(rw::kinematics::Frame* baseframe, int nr_of_nodes):
	Object(baseframe),_rstate(1, rw::common::ownedPtr( new DeformableObjectCache(nr_of_nodes)).cast<StateCache>() )
{
	add(_rstate);
	_model = rw::common::ownedPtr( new rw::graphics::Model3D(baseframe->getName()+"_M"));

	Model3D::Material mat("gray",0.7,0.7,0.7);
	int matId = _model->addMaterial( mat );
	_mesh = rw::common::ownedPtr( new IndexedTriMeshN0<float>() );
	_mesh->getVertices().resize(nr_of_nodes, Vector3D<float>(0,0,0));
	_mesh->getNormals().resize(nr_of_nodes, Vector3D<float>(0,0,0));
	Model3D::Object3D::Ptr obj = rw::common::ownedPtr( new Model3D::Object3D("Obj1") );
	obj->_vertices.resize(nr_of_nodes, Vector3D<float>(0,0,0));
	obj->_normals.resize(nr_of_nodes, Vector3D<float>(0,0,1));
	obj->_faces.resize(0);
	obj->_materialMap.push_back( Model3D::Object3D::MaterialMapData((uint16_t)matId,0,(uint16_t)0) );
	_model->addObject( obj );
	_geom = rw::common::ownedPtr( new rw::geometry::Geometry(_mesh, std::string(baseframe->getName()+"_G")) );
}

DeformableObject::DeformableObject(rw::kinematics::Frame* baseframe, rw::graphics::Model3D::Ptr model):
		Object(baseframe),_rstate(1,NULL )
{
	_mesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<float> >( *(model->toGeometryData()->getTriMesh(false)) ,0.000001 );
	_model = model;
	_rstate = rw::kinematics::StatelessData<int>(1,rw::common::ownedPtr( new DeformableObjectCache(_mesh->getVertices().size())).cast<StateCache>() );
	add(_rstate);

	_geom = rw::common::ownedPtr( new rw::geometry::Geometry(_mesh, std::string(baseframe->getName()+"_G")) );
}

DeformableObject::DeformableObject(rw::kinematics::Frame* baseframe, rw::geometry::Geometry::Ptr geom):
		Object(baseframe),_rstate(1, NULL ) // we change that imidiately below
{
	TriMesh::Ptr mesh = geom->getGeometryData()->getTriMesh(false);
	// create IndexedTriMesh from the data
	_mesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<float> >(*mesh,0.000001);
   _rstate = rw::kinematics::StatelessData<int>(1,rw::common::ownedPtr( new DeformableObjectCache(_mesh->getVertices().size())).cast<StateCache>() );
   add(_rstate);
   _geom = rw::common::ownedPtr( new rw::geometry::Geometry(_mesh, std::string(baseframe->getName()+"_G")) );
   _model = rw::common::ownedPtr( new rw::graphics::Model3D(baseframe->getName()+"_M"));
   Model3D::Material mat("gray",0.7,0.7,0.7);
   _model->addGeometry(mat, _geom);
}

DeformableObject::~DeformableObject(){

}


rw::math::Vector3D<float>& DeformableObject::getNode(int id, rw::kinematics::State& state) const
{
	return _rstate.getStateCache<DeformableObjectCache>(state)->_nodes[id];
}

const rw::math::Vector3D<float>& DeformableObject::getNode(int id, const rw::kinematics::State& state) const{
	return _rstate.getStateCache<DeformableObjectCache>(state)->_nodes[id];
}

size_t DeformableObject::getNrNodes(const rw::kinematics::State& state) const
{
	return _rstate.getStateCache<DeformableObjectCache>(state)->_nodes.size();
}

size_t DeformableObject::getNrNodes() const
{
	return _mesh->getVertices().size();
}

const std::vector<rw::geometry::IndexedTriangle<> >& DeformableObject::getFaces() const {
	return _mesh->getTriangles();
}

void DeformableObject::addFace(unsigned int node1, unsigned int node2, unsigned int node3){
	_mesh->add( rw::geometry::IndexedTriangle<>(node1,node2,node3) );
}

rw::geometry::IndexedTriMesh<float>::Ptr DeformableObject::getMesh(rw::kinematics::State& cstate){
	return _mesh;
}

void DeformableObject::setNode(int id, const rw::math::Vector3D<float>& v, rw::kinematics::State& state){
	_rstate.getStateCache<DeformableObjectCache>(state)->_nodes[id] = v;
}

 const std::vector<rw::geometry::Geometry::Ptr>& DeformableObject::doGetGeometry(const rw::kinematics::State& state) const{
	 if(_rstate.getStateCache<DeformableObjectCache>(state)->_geoms.size()>0)
		 return _rstate.getStateCache<DeformableObjectCache>(state)->_geoms;
	 std::vector<rw::geometry::Geometry::Ptr> geoms;
	 // get a copy of the models with the configuration from the state
	 BOOST_FOREACH(rw::geometry::Geometry::Ptr geom, getGeometry()){
	 	 geoms.push_back( rw::common::ownedPtr( new rw::geometry::Geometry(*geom)) );
	 }
	 _rstate.getStateCache<DeformableObjectCache>(state)->_geoms = geoms;
	 return _rstate.getStateCache<DeformableObjectCache>(state)->_geoms;
 }

 const std::vector<rw::graphics::Model3D::Ptr>& DeformableObject::doGetModels(const rw::kinematics::State& state) const
 {
	 if(_rstate.getStateCache<DeformableObjectCache>(state)->_models.size()>0)
		 return _rstate.getStateCache<DeformableObjectCache>(state)->_models;
	 std::vector<rw::graphics::Model3D::Ptr> models(0);
	 models.push_back(_model);
	_rstate.getStateCache<DeformableObjectCache>(state)->_models = models;
	return _rstate.getStateCache<DeformableObjectCache>(state)->_models;

	 // get a copy of the models with the configuration from the state
	 BOOST_FOREACH(rw::graphics::Model3D::Ptr model, getModels()){
	 	 models.push_back( rw::common::ownedPtr( new rw::graphics::Model3D(*model)) );
	 }
	 _rstate.getStateCache<DeformableObjectCache>(state)->_models = models;
	 return _rstate.getStateCache<DeformableObjectCache>(state)->_models;
 }

 double DeformableObject::getMass(rw::kinematics::State& state) const{
	 return 1.0;
 }

 rw::math::Vector3D<> DeformableObject::getCOM(rw::kinematics::State& state) const{
	 return rw::math::Vector3D<>(0,0,0);
 }

 rw::math::InertiaMatrix<> DeformableObject::getInertia(rw::kinematics::State& state) const{
	 return rw::math::InertiaMatrix<>::makeSolidSphereInertia(1.0,0.1);
 }


 void DeformableObject::update(rw::graphics::Model3D::Ptr model, const rw::kinematics::State& state){
	 // check what model it is and update it with the nodes.
	 // for now we assume only one model.
	 typedef Vector3D<float> Vector3Df;
	 const std::vector<Vector3D<float> > &nodes = _rstate.getStateCache<DeformableObjectCache>(state)->_nodes;
	 std::vector<Model3D::Object3D::Ptr> objects = model->getObjects();
	 int objIndex=0, offset=0;
	 for( int i = 0; i<nodes.size(); i++){
		 if(objects[objIndex]->_vertices.size()>(i-offset)){
			 objects[objIndex]->_vertices[i - offset] = nodes[i];
			 std::cout << objects[objIndex]->_vertices[i - offset] << std::endl;
		 } else {
			 // check to see if the object should be changed
			 if(objects.size()<objIndex+1){
				 objIndex++;
				 offset = i;
			 }
		 }
	 }

	 // check if the number of faces has changed
	 if( objects[0]->_faces.size() != _mesh->getNrTris() ){
		 // copy all ids
		 objects[0]->_faces.resize( _mesh->getNrTris() );
		 for(int i=0;i<_mesh->getNrTris();i++){
			 objects[0]->_faces[i][0] = _mesh->getIndexedTriangle(i)[0];
			 objects[0]->_faces[i][1] = _mesh->getIndexedTriangle(i)[1];
			 objects[0]->_faces[i][2] = _mesh->getIndexedTriangle(i)[2];
		 }
		 // remember to also set size of last material thingy
		 objects[0]->_materialMap.back().size = _mesh->getNrTris();
	 }
 }
