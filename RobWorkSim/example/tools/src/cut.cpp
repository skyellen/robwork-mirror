
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>

#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/lexical_cast.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/geometry/IntersectUtil.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/TriMesh.hpp>
#include <rw/proximity/ProximityData.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

#include <sstream>
#include <fstream>
#include <string>

#include "CutActionParam.hpp"

using namespace boost::program_options;
using namespace std;
using namespace rw::common;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::trajectory;
using rwlibs::proximitystrategies::ProximityStrategyPQP;

rw::common::Ptr<TriMesh> offsetSurface(const TriMesh& triMesh, double dist);
bool isDebugEnabled = false;

struct Knife {
	// the direction in which the knife is sharp, this is an interval
	// we can only cut when moving in this direction
	rw::math::Vector3D<> from,to;

	ProximityModel::Ptr blade, handle;
	Transform3D<> bladeOffset, handleOffset;

	// blade edge is key points on the sharp edge of the blade,
	// all points should be in the cutting plane
	// described relative to blade frame
	std::vector<Vector3D<> > bladeedge;
	// todo how do we handle double bladed stuff
};

struct CutResult {
	CutResult():success(false),failType(NONE){}
	bool success;

	typedef enum {NONE, NoContact, BadDirection, InitialCollision, HandleCollision, GripperCollision,SizeOfStatusArray} Failure;
	Failure failType;
	// the pose of the knife at each timestep
	std::vector<Transform3D<> > path;
	// the force at each timestep
	std::vector< std::pair<Vector3D<>,Vector3D<> > > forcepath;
	// the two resulting meshes (if successfull)
	std::vector<TriMesh::Ptr> meshes;
	//PlainTriMeshF::Ptr mesh1, mesh2;
};


Ptr<CutResult> simulateCut(Knife& knife, ProximityModel::Ptr obj, Object::Ptr objPtr, ProximityStrategyPQP& colstrat, CutActionParam& param);
std::vector<TriMesh::Ptr> knifeMesh(TriMesh::Ptr knife, const Transform3D<>& wTknife,
								 TriMesh::Ptr object, const Transform3D<>& wTobject);


Vector3D<> parseVector3D(const std::string& str){

	stringstream sstr(str);
	Vector3D<> val;
	sstr >> val[0] >> val[1] >> val[2];
	//std::cout << val << std::endl;
	return val;
}

std::vector<Vector3D<> > parseBladeEdge(const std::string& str){
	//std::cout << "parseBladeEdge" << std::endl;
	std::vector<Vector3D<> > result;
	stringstream sstr(str);
	while(!sstr.eof()){
		Vector3D<> val;
		sstr >> val[0] >> val[1] >> val[2];
		result.push_back(val);
		//std::cout << val << std::endl;
	}
	return result;
}


int main(int argc, char** argv){
    // we need
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("knife,k", value<string>(), "the knife name.")
        ("object,o", value<string>(), "the object name.")
        ("output-dir", value<string>()->default_value("out"), "The output directory.")
        ("output-geom", value<bool>()->default_value(false), "The geometries of cutted object will be saved.")
        ("output-statepath", value<bool>()->default_value(false), "State path for visually checking cutting trajectory.")
        ("debug", value<bool>()->default_value(false), "Debug stuff.")
        ("wc,w", value<string>(), "the workcell file.")
        ("input", value<string>(), "input of action parameters.")

        ("gripper", value<string>(), "the gripper.")
        ("gripper-tcp", value<string>()->default_value("GRIPPER_TCP"), "the gripper tcp.")
        ("gripper-base", value<string>()->default_value("GRIPPER_MOVER"), "the frame that moves the gripper.")

        //("batchsize", value<int>()->default_value(5000), "The max nr grasp of output, files will be split into multiple files.")
        //("useGraspTarget",value<bool>()->default_value(false),"Enable to copy ObjTTcpTarget to target.pose")
    ;
    positional_options_description optionDesc;
    optionDesc.add("input",-1);
    variables_map vm;
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);

    if (vm.count("help")) {
        cout << "Usage:\n\n"
                  << "\t" << argv[0] <<" [options] -w<workcellFile> -o<objectName> -k<knifeName> <actionInput> \n"
                  << "\n";
        cout << desc << "\n";
        return 1;
    }

    std::string wcFile = vm["wc"].as<string>();
    std::string knifeName = vm["knife"].as<string>();
    std::string objName = vm["object"].as<string>();
    std::string input = vm["input"].as<string>();

    string out_dir = vm["output-dir"].as<string>();
    if(!boost::filesystem::exists(out_dir)){
    	boost::filesystem::create_directory(out_dir);

    }
    bool output_statepath = vm["output-statepath"].as<bool>();
    bool output_geom = vm["output-geom"].as<bool>();
    bool debug = vm["debug"].as<bool>();
    isDebugEnabled = debug;
    std::vector<CutActionParam> params = CutActionParam::load(input);

    WorkCell::Ptr wc = WorkCellFactory::load(wcFile);

    MovableFrame* knife_frame = wc->findFrame<MovableFrame>(knifeName);
    std::string bladeName = knife_frame->getPropertyMap().get<std::string>("blade");
    std::string handleName = knife_frame->getPropertyMap().get<std::string>("handle");
    std::string bladeedge_desc = knife_frame->getPropertyMap().get<std::string>("bladeedge");
    std::string from_desc = knife_frame->getPropertyMap().get<std::string>("dir_from");
    std::string to_desc = knife_frame->getPropertyMap().get<std::string>("dir_to");

    Object::Ptr blade = wc->findObject(bladeName);
    Object::Ptr handle = wc->findObject(handleName);
    Object::Ptr obj = wc->findObject(objName);

    State state = wc->getDefaultState();

    // parse gripper stuff
    std::string gripper_name = vm["gripper"].as<string>();
    std::string gripper_tcp_name = vm["gripper-tcp"].as<string>();
    std::string gripper_base_name = vm["gripper-base"].as<string>();

    Device::Ptr gripper = wc->findDevice(gripper_name);
    MovableFrame* gripper_base = wc->findFrame<MovableFrame>(gripper_base_name);
    Frame* gripper_tcp = wc->findFrame(gripper_tcp_name);

    if(gripper==NULL)
        RW_THROW("Gripper \""<<gripper_name<<"\" could not be found in workcell!");
    if(gripper_base==NULL)
        RW_THROW("Gripper base \""<<gripper_base_name<<"\" could not be found in workcell!");
    if(gripper_tcp==NULL)
        RW_THROW("Gripper \""<<gripper_tcp_name<<"\" could not be found in workcell!");


    // describe everything relative to knife_frame
    Knife cutter;
    cutter.bladeedge = parseBladeEdge( bladeedge_desc );
    cutter.from = parseVector3D( from_desc );
    cutter.to = parseVector3D( to_desc );
    cutter.bladeOffset = Kinematics::frameTframe(knife_frame, blade->getBase(), state);
    cutter.handleOffset = Kinematics::frameTframe(knife_frame, handle->getBase(), state);

    ofstream resultFile;
    resultFile.open(std::string(out_dir+"/result.txt").c_str());
    ofstream resultCompactFile;
    resultCompactFile.open(std::string(out_dir+"/resultCompact.txt").c_str());
    // make proximity model
	TimedStatePath spath_global;
	int time_global = 0;

	// prepare the collision strategy
    ProximityStrategyPQP::Ptr strategy = ownedPtr(new ProximityStrategyPQP());
    strategy->addModel(blade);
    strategy->addModel(handle);
    strategy->addModel(obj);
    ProximityModel::Ptr objectModel = strategy->getModel(obj->getBase());

    // gripper collision detector - create a collision detector that only
    // test collision between gripper and object
    CollisionDetector detector(wc, ownedPtr(new ProximityStrategyPQP()) );
    ProximitySetupRule ruleExcludeAll("*", "*", ProximitySetupRule::EXCLUDE_RULE);
    ProximitySetupRule ruleIncludeObject(gripper_name+".*", objName, ProximitySetupRule::INCLUDE_RULE);

    detector.addRule( ruleExcludeAll );
    detector.addRule( ruleIncludeObject );
    rw::proximity::ProximityData pdata;
    pdata.setCollisionQueryType( CollisionDetector::FirstContactNoInfo );
    int successes=0;
    // int totalSims=params.size(); // Unused
    std::vector<int> failTypes( CutResult::SizeOfStatusArray , 0);
	for(size_t pi=0;pi<params.size(); pi++){

        cutter.blade = strategy->getModel(blade->getBase());
        cutter.handle = strategy->getModel(handle->getBase());

		// we simulate
		if(isDebugEnabled)
		    std::cout << "Simulate cute: " << std::endl;
		Ptr<CutResult> result = simulateCut(cutter, objectModel, obj, *strategy, params[pi]);


		// before saving result, test if gripper collides with object during motion
		if(result->success){
		    bool gripperCollideWithObj = false;
	        for(size_t i=0;i<result->path.size();i++){
	            // set configuration of object and knife and save state.
	            Transform3D<> objPose( params[0].posObj, params[0].rpyObj );
	            Transform3D<> knifet3d = result->path[i];
	            ((MovableFrame*)obj->getBase())->setTransform(objPose, state);
	            knife_frame->setTransform(knifet3d, state);

	            // calculate position of gripper base
	            Transform3D<> knifeTgripper(params[pi].posGripper,params[pi].rpyGripper);
                Transform3D<> knifeTgripperbase = knifeTgripper * Kinematics::frameTframe(gripper_tcp,gripper_base, state);

	            gripper->setQ( params[pi].gripperQ, state);
	            gripper_base->moveTo(knifeTgripperbase,knife_frame,state);
	            // test for collision
	            gripperCollideWithObj |= detector.inCollision(state, pdata);
	        }
	        if(gripperCollideWithObj){
	            //std::cout << "GRIPPER COLLIDES WITH OBJECT!" << std::endl;
	            result->success = false;
	            result->failType = CutResult::GripperCollision;
	        } else {
	            //std::cout << "GRIPPER DOES NOT COLLIDE WITH OBJECT!" << std::endl;
	        }
		}
		if(result->success){
		    successes++;
		} else {
		    failTypes[result->failType]++;
		}

		std::cout << (pi+1) << "\t"
                        << successes << "\t"
                        << (successes*100.0)/(pi+1) <<"%" << "\t";
		for(std::size_t errType=0;errType<failTypes.size();errType++)
		    std::cout << failTypes[errType] << "\t";
        std::cout << "        \r";

		resultCompactFile << pi << " "
		                  << result->success << " "
		                  << result->failType << " "
		                  << result->meshes.size() << " "
		                  << "\n";


		resultFile << "\nexp " << pi
				   << "\n success " << result->success
				   << "\n failtype " << result->failType
				   << "\n pathlen "  << result->path.size()
				   << "\n forcepathlen "  << result->forcepath.size()
				   << "\n geoms "  << result->meshes.size()
				   ;


		resultFile << "\n ftdata \n";
		for(std::size_t i=0;i<result->forcepath.size();i++){
			Vector3D<> f = result->forcepath[i].first;
			Vector3D<> t = result->forcepath[i].second;
			resultFile << f[0] << " " << f[1] << " " << f[2] << " "
			           << t[0] << " " << t[1] << " " << t[2] << "\n";
		}

		if(output_statepath)
			resultFile << "\n statepath "  << boost::lexical_cast<std::string>(pi) << "_statepath.rwplay";

		if(output_geom){
			TriMesh::Ptr mesh = obj->getGeometry()[0]->getGeometryData()->getTriMesh() ;
			STLFile::save( *mesh,
					out_dir+"/"+ boost::lexical_cast<std::string>(pi) + "_geo" +".stl");

			for(std::size_t i=0;i<result->meshes.size();i++){
				resultFile << "\n geo "  << pi<< "_geo" << i <<".stl";
				STLFile::save( *result->meshes[i] ,
						out_dir+"/"+ boost::lexical_cast<std::string>(pi) + "_geo" + boost::lexical_cast<std::string>(i) +".stl");
			}
		}


		// create state path
		if(output_statepath){
            TimedStatePath spath;
            for(size_t i=0;i<result->path.size();i++){
                // set configuration of object and knife and save state.
                Transform3D<> objPose( params[0].posObj, params[0].rpyObj );
                Transform3D<> knifet3d = result->path[i];
                knife_frame->setTransform(knifet3d, state);
                ((MovableFrame*)obj->getBase())->setTransform(objPose, state);

                // calculate position of gripper base
                Transform3D<> knifeTgripper(params[pi].posGripper,params[pi].rpyGripper);
                gripper->setQ( params[pi].gripperQ, state);
                Transform3D<> knifeTgripperbase = knifeTgripper * Kinematics::frameTframe(gripper_tcp,gripper_base, state);

                gripper_base->moveTo(knifeTgripperbase,knife_frame,state);

                spath.push_back( TimedState(i*0.01, state) );

                spath_global.push_back( TimedState(time_global*0.01, state) );
                time_global++;
            }
			PathLoader::storeTimedStatePath(*wc, spath, out_dir+"/"+ boost::lexical_cast<std::string>(pi) + "_statepath.rwplay");
		}
		/*
		if(result->success){
			RW_WARN("");
			// change objectModel
		    //strategy = new ProximityStrategyPQP();
		    //strategy->addModel(blade);
		    //strategy->addModel(handle);

		    //obj = ownedPtr( new Object(obj->getBase(), ownedPtr( new Geometry( result->meshes[1] ) )));


			Geometry::Ptr geom = obj->getGeometry()[0];

			geom->setGeometryData( result->meshes[1] );
			obj = ownedPtr( new Object(obj->getBase(), geom ));

			//objectModel->removeGeometry( objectModel->getGeometryIDs()[0] );
			//obj->removeGeometry( obj->getGeometry()[0] );
			std::cout << "nr meshes: " << result->meshes.size() << std::endl;
			//Geometry::Ptr geom = ownedPtr( new Geometry( result->meshes[1] ) );
			//geom->setFrame(obj->getBase());
			//obj->addGeometry( geom );
			RW_WARN("");
		    //strategy->addModel(obj);
		    //objectModel = strategy->getModel(obj->getBase());

			//objectModel->addGeometry( *(obj->getGeometry()[0]) );

		}
		*/

    }

	if(output_statepath)
		PathLoader::storeTimedStatePath(*wc, spath_global, out_dir+"/global_statepath.rwplay");

    // save summary

    return 1;
}

rw::common::Ptr<CutResult> simulateCut(Knife& knife, ProximityModel::Ptr obj, Object::Ptr objPtr, ProximityStrategyPQP& colstrat, CutActionParam& param){
	ProximityStrategyData data;

	Ptr<CutResult> res = ownedPtr( new CutResult() );

	Transform3D<> knife_pose_init(param.pos, param.rpy );
	// calculate the new move direction
	Vector3D<> knifeNormal = normalize( cross( knife_pose_init.R()*knife.from, knife_pose_init.R()*knife.to) );
	// project knife dir onto plane
	Vector3D<> ndir = normalize(param.dir - (dot(param.dir, knifeNormal) * knifeNormal) );
	// make ndir as long as param.dir
	ndir = ndir*param.dir.norm2();

	if(isDebugEnabled){
        std::cout << "KnifeDIR: " << ndir << std::endl;
        std::cout << "paramDIR: " << param.dir << std::endl;
	}

	// if ndir is not within [from;to] then the direction is bad
	double ang1 = angle(knife_pose_init.R()*knife.from, ndir, knifeNormal);
	double ang2 = angle(knife_pose_init.R()*knife.from, knife_pose_init.R()*knife.to, knifeNormal);
	if(ang2<0 && (ang1>0 || ang1<ang2) ){
		res->failType = CutResult::BadDirection;
		return res;
	}
	if(ang2>0 && (ang1<0 || ang1>ang2) ){
		res->failType = CutResult::BadDirection;
		return res;
	}

	int timesteps = 100;
	double len_dt = param.len/timesteps;
	Transform3D<> objPose( param.posObj, param.rpyObj );
	// the simulation runs in two steps:
	// first the position of the knife is translated in the cutting direction until
	// collision with object is detected
	int i = 0;
	data.setCollisionQueryType( CollisionStrategy::FirstContact);
	for(i=0;i<timesteps;i++){
		// calculate new position of knife
		Transform3D<> t3d_knife(param.pos + param.dir*(len_dt*i), param.rpy );
		res->path.push_back(t3d_knife);
		res->forcepath.push_back(std::make_pair(Vector3D<>(0,0,0),Vector3D<>(0,0,0)));

		// check if knife is in collision
		// if handle is in collision the we failed
		if( colstrat.inCollision(knife.handle, t3d_knife * knife.handleOffset, obj, objPose, data) ){
			res->success = false;
			res->failType = CutResult::HandleCollision;
			return res;
		}

		if( colstrat.inCollision(knife.blade, t3d_knife * knife.bladeOffset, obj, objPose, data) ){
			// TODO: make sure its the sharp end of the blade that hit the object
			res->success = true;
			break;
		}
	}

	// check if we hit the object
	if( i>=timesteps ){
		res->failType = CutResult::NoContact;
		return res;
	}

	int cuttingStartIndex = res->path.size()-2;
	int cuttingEndIndex = res->path.size();

	data.setCollisionQueryType( CollisionStrategy::AllContacts );
	// the second step is to continue moving the knife in the direction of cutting
	// until it is through the object
	Vector3D<> cpos = res->path.back().P();
	bool lastCollide = true;
	int j = 1;
	for(;i<timesteps;i++){
		// calculate new position of knife
		Transform3D<> t3d_knife(cpos + ndir*(len_dt*j), param.rpy );
		j++;
		//Transform3D<> t3d_knife(param.pos + param.dir*(len_dt*i), param.rpy );

		res->path.push_back(t3d_knife);
		// translate contact force -ndir into object coordinates
		res->forcepath.push_back(std::make_pair(-ndir,Vector3D<>(0,0,0)));


		// check if knife is in collision
		// if handle is in collision the we failed
		if( colstrat.inCollision(knife.handle, t3d_knife * knife.handleOffset, obj, objPose, data) ){
			res->success = false;
			res->failType = CutResult::HandleCollision;
			return res;
		}

		if( colstrat.inCollision(knife.blade, t3d_knife * knife.bladeOffset, obj, objPose, data) ){
			// TODO: calculate force path from contact information
			// the force is scaled by the area which is in contact
			//data.getCollisionData()._collisionPairs
			lastCollide = true;
		} else {
			// knife released hold of object so we stop...
			if(lastCollide)
				cuttingEndIndex = i+1;
			lastCollide = false;
		}
	}

	// now create the cutting mesh
	PlainTriMeshF cuttingMesh;
	if(isDebugEnabled)
	    std::cout << "Cutting start: " << cuttingStartIndex << std::endl;
	for(int j=cuttingStartIndex+1;j<cuttingEndIndex;j++){
		Transform3D<> knifeTrans_a = res->path[j-1];
		Transform3D<> knifeTrans_b = res->path[j] ;
		for(std::size_t x=1;x<knife.bladeedge.size();x++){
			Vector3D<float> a1 = cast<float>( knifeTrans_a * knife.bladeedge[x-1] );
			Vector3D<float> a2 = cast<float>( knifeTrans_a * knife.bladeedge[x] );

			Vector3D<float> b1 = cast<float>( knifeTrans_b * knife.bladeedge[x-1] );
			Vector3D<float> b2 = cast<float>( knifeTrans_b * knife.bladeedge[x] );

			cuttingMesh.add( Triangle<float>(a1,b1,b2) );
			cuttingMesh.add( Triangle<float>(a1,b2,a2) );
		}
	}
    if(isDebugEnabled){
    	STLFile::save(cuttingMesh, "tests_mesh.stl");
    }
	// perform actual cutting

	TriMesh::Ptr objTriMesh = objPtr->getGeometry()[0]->getGeometryData()->getTriMesh();

	std::vector<TriMesh::Ptr> cutobjs =
			knifeMesh(Ptr<TriMesh>(&cuttingMesh), Transform3D<>::identity(), objTriMesh, objPose);

	res->meshes = cutobjs;
	return res;
}


std::vector<TriMesh::Ptr> knifeMesh(TriMesh::Ptr knife, const Transform3D<>& wTknife,
		TriMesh::Ptr object, const Transform3D<>& wTobject){
    if(isDebugEnabled){
        std::cout << "Knifing mesh" << std::endl;

        std::cout << "to indexed mesh" << std::endl;
    }
	// convert object to indexed trimesh
	IndexedTriMeshN0<float>::Ptr imesh_object =
			TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<float> >( *object );

	// now create a mesh representation of edges
    ProximityStrategyPQP::Ptr strategy = ownedPtr( new ProximityStrategyPQP() );
    Geometry knifeGeom(knife);
    Geometry objectGeom(imesh_object);
    ProximityStrategyData data;
    data.setCollisionQueryType( CollisionStrategy::AllContacts);
    ProximityModel::Ptr knifeModel = strategy->createModel();
    knifeModel->addGeometry(knifeGeom);
    ProximityModel::Ptr objectModel = strategy->createModel();
    objectModel->addGeometry(objectGeom);
    if( ! strategy->inCollision(knifeModel, wTknife, objectModel, wTobject, data) )
    	return std::vector<TriMesh::Ptr>();
    // now comes the fun part
    // find all edges/triangles that are to be changed
    PlainTriMeshD::Ptr knifeMeshResult_left = ownedPtr( new PlainTriMeshD() );
    PlainTriMeshD::Ptr knifeMeshResult_right = ownedPtr( new PlainTriMeshD() );
    PlainTriMeshD knifeMeshCols;

    /*
    std::map<int, int> knifeMap, objectMap;
    typedef std::pair<int, int> IDs;
    BOOST_FOREACH( IDs prims, data.getCollisionData()._geomPrimIds ){
    	int knife_tri = prims.first;
    	int object_tri = prims.second;
    	knifeMap[knife_tri]++;
    	objectMap[object_tri]++;
    }

    std::cout << "KNIFE MAP" << std::endl;
    BOOST_FOREACH( IDs data, knifeMap ){
    	std::cout << data.first << " = " << data.second << "\n";
    }

    std::cout << "OBJECT MAP" << std::endl;
    BOOST_FOREACH( IDs data, objectMap ){
    	std::cout << data.first << " = " << data.second << "\n";
    }
*/

    std::vector<int> collidingTrisObject;
    std::stack<int> vertsLeft, vertsRight;


    typedef std::pair<int, int> IDs;
    BOOST_FOREACH( IDs prims, data.getCollisionData()._geomPrimIds ){
    	int knife_tri = prims.first;
    	int object_tri = prims.second;
    	collidingTrisObject.push_back(object_tri);
    	//
    	// now create a cutted triangle on the knife mesh which is on the
    	// negative side of the object surface (inside the object)
    	Triangle<> ktri, otri;
    	knife->getTriangle(knife_tri, ktri);
    	imesh_object->getTriangle(object_tri, otri);

    	IndexedTriangle<uint32_t> otri_idx = imesh_object->getIndexedTriangle( object_tri );

    	ktri = ktri.transform(wTknife);
    	otri = otri.transform(wTobject);
    	knifeMeshCols.add(otri);
    	// determine contact points
    	Vector3D<> p1,p2;
    	if( !IntersectUtil::intersetPtTriTri(ktri,otri,p1,p2) ){
    		//RW_WARN("COULD NOT FIND INTERSECTION");
    		continue;
    	}
    	if( std::isnan(p1[0]) || std::isnan(p2[0])){

    		std::cout << p1 <<"\n"<<  p2 << "\n\n";
    		std::cout << ktri << std::endl;
    		std::cout << otri << std::endl;
    	}
    	// create new triangles
    	// 1. determine on which side of the line (p1,p2) the vertices of otri.
    	Vector3D<> normal = otri.calcFaceNormal();
    	Vector3D<> knormal = ktri.calcFaceNormal();
    	bool side0 = dot(cross((p2-p1),(otri[0]-p1)), normal ) <0;
    	bool side1 = dot(cross((p2-p1),(otri[1]-p1)), normal ) <0;
    	bool side2 = dot(cross((p2-p1),(otri[2]-p1)), normal ) <0;

    	if(dot(otri[0]-ktri[0], knormal ) <0) vertsLeft.push( otri_idx[0] );
    	else vertsRight.push( otri_idx[0] );
    	if(dot(otri[1]-ktri[0], knormal ) <0 ) vertsLeft.push( otri_idx[1] );
    	else vertsRight.push( otri_idx[1] );
    	if(dot(otri[2]-ktri[0], knormal ) <0 ) vertsLeft.push( otri_idx[2] );
    	else vertsRight.push( otri_idx[2] );


    	// we rotate triangle such that s1 and s2 are allways on the same side
    	if(side0==side2){
    		//side = side0;
    		std::swap( otri[0],otri[1] );
    		std::swap( otri[2],otri[0] );
    	} else if(side1==side2 ){
    		//side = side1;
    		std::swap( otri[0],otri[1] );
    		std::swap( otri[1],otri[2] );
    	}
    	// check which side of knife the tri0 is on

    	// project  tri0 onto p1,p2
    	Vector3D<> p = IntersectUtil::closestPtPointRay(otri[0],p1,p2);
    	bool knife_side = !(dot( otri[0]-p , knormal )>=0);

    	/*
    	Vector3D<> p_tri1 = IntersectUtil::closestPtPointLine(otri[1],p1,p2);
    	if(MetricUtil::dist2(otri[0], p )<0.0001)
    		continue;
    	if(MetricUtil::dist2(otri[0], p )<MetricUtil::dist2(otri[1], p_tri1 )){
        	knife_side = !(dot( otri[1]-p_tri1 , knormal )>=0);

    	}
    	*/
    	///std::cout << MetricUtil::dist2(otri[0], p ) << std::endl;


    	if(side0==side1 && side0==side2){
        	//std::cout << side0 << " " << side1 << " " << side2 << std::endl;

    		//std::cout << p1 <<"\n"<<  p2 << "\n\n";
    		//std::cout << ktri << std::endl;
    		//std::cout << otri << std::endl;

			if(!knife_side)
			{
				//knifeMeshResult.add(otri);
			}

    		continue;
    	}



    	Transform3D<> objTw = inverse( wTobject );
		if(dot( cross(p2-otri[0],p1-otri[0]) , normal )>0){
			// then p2 comes before p1
			if(knife_side){
				knifeMeshResult_left->add(Triangle<>(otri[0],otri[1],p2).transform(objTw) );
				knifeMeshResult_left->add(Triangle<>(otri[0],p2,p1).transform(objTw) );

				knifeMeshResult_right->add(Triangle<>(otri[2],p1,p2).transform(objTw) );
			} else {
				knifeMeshResult_right->add(Triangle<>(otri[0],otri[1],p2).transform(objTw));
				knifeMeshResult_right->add(Triangle<>(otri[0],p2,p1).transform(objTw) );

				knifeMeshResult_left->add(Triangle<>(otri[2],p1,p2).transform(objTw) );
			}

			//} else {
			//	knifeMeshResult_right.add(Triangle<>(otri[2],p1,p2) );
			//}
		} else {
			// then p1 comes before p2
			if(knife_side){
				knifeMeshResult_left->add(Triangle<>(otri[0],otri[1],p1).transform(objTw));
				knifeMeshResult_left->add(Triangle<>(otri[0],p1,p2).transform(objTw) );
			//} else {
				knifeMeshResult_right->add(Triangle<>(otri[2],p2,p1).transform(objTw) );
			} else {
				knifeMeshResult_right->add(Triangle<>(otri[0],otri[1],p1).transform(objTw));
				knifeMeshResult_right->add(Triangle<>(otri[0],p1,p2).transform(objTw) );
			//} else {
				knifeMeshResult_left->add(Triangle<>(otri[2],p2,p1).transform(objTw) );
			}
		}

    }

    if(isDebugEnabled){
		STLFile::save(*knifeMeshResult_left,"knifemesh_result_left.stl");
		STLFile::save(*knifeMeshResult_right,"knifemesh_result_right.stl");
    }

    // now come the last part, determine the new meshes by computing connectivity between
    // the cutted mesh and the remaining mesh.


    // create vertice-nighbour map
    std::vector<int> triColor(imesh_object->size(), 0);
    std::vector<int> vertColor(imesh_object->size(), 0);
    std::vector<std::vector<int> > verticeToTris(imesh_object->getVertices().size());

    for(std::size_t i=0; i<imesh_object->size(); i++ ){
    	IndexedTriangle<uint32_t> itri = imesh_object->getIndexedTriangle(i);
    	verticeToTris[itri.getVertexIdx(0)].push_back(i);
    	verticeToTris[itri.getVertexIdx(1)].push_back(i);
    	verticeToTris[itri.getVertexIdx(2)].push_back(i);
    }
    enum{NONE=0, COLLIDING_TRI=1, LEFT_TRI, RIGHT_TRI};
    // now color all triangles on either side of the knife
    BOOST_FOREACH(int colTri, collidingTrisObject){
    	triColor[colTri] = COLLIDING_TRI;
    }
    // next do floodfill and color all left and right triangles
    // std::stack<int> verticeFrontLeft, verticeFrontRight;
    while(!vertsLeft.empty()){
    	int vert = vertsLeft.top();
    	vertsLeft.pop();
    	// get triangles that are not colored
    	BOOST_FOREACH(int tri, verticeToTris[vert]){
    		if(triColor[tri]!=NONE)
    			continue;
    		triColor[tri] = LEFT_TRI;
    		// add vertices to vertsLeft
    		IndexedTriangle<uint32_t> itri = imesh_object->getIndexedTriangle(tri);
    		vertsLeft.push( itri[0] );
    		vertsLeft.push( itri[1] );
    		vertsLeft.push( itri[2] );
    	}
    	//
    }

    while(!vertsRight.empty()){
    	int vert = vertsRight.top();
    	vertsRight.pop();
    	// get triangles that are not colored
    	BOOST_FOREACH(int tri, verticeToTris[vert]){
    		if(triColor[tri]!=NONE)
    			continue;
    		triColor[tri] = RIGHT_TRI;
    		// add vertices to vertsLeft
    		IndexedTriangle<uint32_t> itri = imesh_object->getIndexedTriangle(tri);
    		vertsRight.push( itri[0] );
    		vertsRight.push( itri[1] );
    		vertsRight.push( itri[2] );
    	}
    	//
    }


    // now add all triangles that are colored left to left trimesh
    for(std::size_t tri=0; tri<triColor.size(); tri++){

    	if(triColor[tri]==LEFT_TRI){
        	Triangle<> otri;
        	imesh_object->getTriangle(tri, otri);
        	//otri = otri.transform(wTobject);

    		knifeMeshResult_left->add( otri );
    	}

    	if(triColor[tri]==RIGHT_TRI){
        	Triangle<> otri;
        	imesh_object->getTriangle(tri, otri);
        	//otri = otri.transform(wTobject);

    		knifeMeshResult_right->add( otri );
    	}
    }

    if(isDebugEnabled){
		STLFile::save(*knifeMeshResult_left,"knifemesh_result_left_comp.stl");
		STLFile::save(*knifeMeshResult_right,"knifemesh_result_right_comp.stl");
		STLFile::save(knifeMeshCols,"knifemesh_cols.stl");
    }
    // the inefficient approach here is to
    // 1. color all colliding triangles


    std::vector<TriMesh::Ptr> result;
    result.push_back(knifeMeshResult_left);
    result.push_back(knifeMeshResult_right);
    return result;
}

