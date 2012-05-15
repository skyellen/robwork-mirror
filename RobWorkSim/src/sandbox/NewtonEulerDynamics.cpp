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

#include "NewtonEulerDynamics.hpp"

using namespace rw::core::kinematics;
using namespace rw::core::math;
using namespace rw::core::models;
using namespace boost::numeric::ublas;
using namespace std;

NewtonEulerDynamics::NewtonEulerDynamics(const SerialDevice &rob,bool print) :
Z(0,0,1),
R(0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0)
{
	bodies.resize(0);
	if(print){
		cout<<"[NewtonEulerDynamics] Adding RigidBodies:"<<endl;
	}
	robot = &rob;
	base = robot->getBase();
	const Frame *end = robot->getEnd();
	
	bool endisnotbase = true;
	
	while( endisnotbase ){
		if (end == base)
		{
			endisnotbase = false;
		}
		
		Frame::const_iterator_pair iter_pair = end->getChildren();
		while( iter_pair.first != iter_pair.second ){
			const Frame *tmp = &(*iter_pair.first);
			// check if frame is a RigidBody
			const RigidBody *body = dynamic_cast<const RigidBody*>(tmp);
			if( body != NULL ){
				if(print){
					cout<<"adding \""<<body->getName()<<"\""<<endl;
				}
				bodies.push_back( (RigidBody*)body );
			}
			iter_pair.first++;	
		}
		end = end->getParent();	
	}
	if(print){
		cout<<"[NewtonEulerDynamics] List of Rigidbodies: "<<endl;
	
		for (unsigned int h=0; h<bodies.size(); h++)
		{
			//cur_body = 
			cout<<h<<" : "<<bodies[h]->getName()<<endl;
		}
		cout<<"[NewtonEulerDynamics] Finished adding RigidBodies"<<endl;
	}
	
	links = bodies.size();
	w.resize(links+1);
	wd.resize(links+1);
	vd.resize(links+1);
	vdC.resize(links+1);
	F.resize(links+1);
	Nout.resize(links+1);

	f.resize(links+1);
	n.resize(links+1);
	tau.resize(links);
}

void NewtonEulerDynamics::execute(State &state,
						    const Q &q,
						    const Q &qd,
			 			    const Q &qdd,
						    const Vector3D<double >& w0,
						    const Vector3D<double >& wd0,
						    const Vector3D<double >& vd0,
						    const Vector3D<double >& f_end,
						    const Vector3D<double >& n_end,
						    bool print, bool printres)
{
	int i, j;

	w[0]	= w0;
	wd[0]	= wd0;
	vd[0] 	= vd0;
	vdC[0]	= Z*0.0;

	robot->setQ(q,state);
	base = robot->getBase();

	Frame::const_iterator_pair iter_pair = base->getChildren();

	if(print){
		cout<<"[NewtonEulerDynamics] Executing N-E Algorithm "<<endl;
		cout<<"[NewtonEulerDynamics] Outward iterations"<<endl;
	}
	bool first = true;
	/*outward iterations*/
	for ( i=0; i<links; i++)
	{
		if(print){
			cout<<"--------------------------------------------"<<endl;
			cout<<"i : "<<i<<endl;
		}
		j        = links-1-i;
		
		iter_pair = base->getChildren();
		base = &(*iter_pair.first);
		
		Ti	 = base->getTransform(state);
		//cout<<"[NewtonEulerDynamics] OI - Ti :"<<endl;
		if(print)
			printT(Ti, 1.0e-8);
		
		cur_body = bodies[j];
		
		pci      = (cur_body->getTransform(state)).P();
				
		m        = cur_body->getMass();
		
		//cI			= cur_body->getInertia();
		cI			= prod(cur_body->getTransform(state).R().m(),cur_body->getInertia());
		//cI			= cur_body->getTransform(state).R() * cI;
		
		if(print)
		{
			cout<<inverse(Ti).R()<<" * "<<w[i]<<" + "<<qd[i]*Z<<endl;
		}
		first = false;
		
		w[i+1]	 = inverse(Ti).R() * w[i] + qd[i] * Z;
		if(print)
			cout<<w[i+1]<<endl<<endl;
		
		if(print){
			cout<<inverse(Ti).R()<<" * "<<wd[i]<<" + "<<inverse(Ti).R()<<" * "<<w[i]<<" x "<<qd[i]*Z<<" + "<<qdd[i]*Z<<endl;
			cout<<inverse(Ti).R() * wd[i]<<" + "<<inverse(Ti).R()*w[i]<<" x "<<qd[i]*Z<<" + "<<qdd[i]*Z<<endl;
			cout<<inverse(Ti).R() * wd[i]<<" + "<<cross(inverse(Ti).R() * w[i],qd[i] * Z)<<" + "<<qdd[i]*Z<<endl;
		}
			
		wd[i+1]	 = inverse(Ti).R() * wd[i] + cross(inverse(Ti).R() * w[i],qd[i] * Z) + qdd[i] * Z;
		if(print)
			cout<<wd[i+1]<<endl<<endl;
		
		temp	 = (cross(wd[i],Ti.P()) + cross(w[i],cross(w[i],Ti.P()))+vd[i]);
		
		vd[i+1]	 = inverse(Ti).R() * temp;
		
		vdC[i+1] = cross(wd[i+1], pci) + cross(w[i+1],cross(w[i+1],pci)) + vd[i+1];

		F[i+1]   = m * vdC[i+1];

		temp     = prod(cI,w[i+1]);

		Nout[i+1]   = prod(cI,wd[i+1]) + cross(w[i+1],prod(cI,w[i+1]));
	}
	
	if(print){
		cout<<"[NewtonEulerDynamics] Finished outward iterations "<<endl;
	}
	if(print||printres){
		printout();
	}
	if(print){
		cout<<"[NewtonEulerDynamics] Inward iterations "<<endl;
	}
	
	f[links]   = f_end;
	n[links]   = n_end;
	iter_pair = base->getChildren();
	base = &(*iter_pair.first);
	/*inward iterations*/
	for (i=links; i>0; i--){
		if(print){
			cout<<"-------------------------"<<endl<<"i : "<<i<<endl;
		}		
		j        = links-i;
		cur_body = bodies[j];
		pci      = (cur_body->getTransform(state)).P();
		Ti = base->getTransform(state);
		base = base->getParent();
		
		if(print){
			cout<<"f"<<endl;
			cout<<Ti.R()<<" * "<<f[i]<<" + "<<F[i]<<endl;
			cout<<Ti.R() * f[i] <<" + "<<F[i]<<endl;
		}
		
		f[i-1]	= Ti.R() * f[i] + F[i];
		if(print){
			cout<<f[i-1]<<endl;
		}
		
		n[i-1]	= Nout[i] + (Ti.R() * n[i]) + cross(pci,F[i])
				  + cross(Ti.P(),Ti.R() * f[i]);

		if(print){
			cout<<"n"<<endl;
			cout<<Nout[i]<<" + "<<Ti.R()<<" * "<<n[i]<<" + "<<pci<<" x "<<F[i]<<" + "<<Ti.P()<<" x ("<<Ti.R()<<" * "<<f[i]<<")"<<endl;
			cout<<Nout[i]<<" + "<<(Ti.R() * n[i])<<" + "<<cross(pci,F[i])<<" + "<<cross(Ti.P(),Ti.R() * f[i])<<endl;
			cout<<n[i-1]<<endl;
		}
		//tau[i]	= ((inverse(Ti).R())*n[i-1])(2);
		tau[i-1]	= dot(n[i-1],Z);
		if(print)
			cout<<"tau[i]: "<<tau[i]<<endl;
	}
	if(print||printres){
		printin();
	}
	if(print){
		cout<<"[NewtonEulerDynamics] Finished inward iterations "<<endl;
		cout<<"[NewtonEulerDynamics] Finished executing N-E algorithm"<<endl;
	}
}

const std::vector<double >* NewtonEulerDynamics::readTau(){
	//cout<<"[NewtonEulerDynamics] Reading results"<<endl;
	return &tau;
}

void NewtonEulerDynamics::printout()
{
	unsigned int i;
	
	// w
	cout<<"  \t";
	for (i=0; i<w.size(); i++){
		printf("%+2.4f  \t",w[i](0));
	}cout<<endl;
	
	cout<<"w:\t";
	for (i=0; i<w.size(); i++){
		printf("%+2.4f  \t",w[i](1));
	}cout<<endl;
	
	cout<<"  \t";
	for (i=0; i<w.size(); i++){
		printf("%+2.4f  \t",w[i](2));
	}cout<<endl;
	cout<<endl;
	
	//wd
	cout<<"  \t";
	for (i=0; i<wd.size(); i++){
		printf("%+2.4f  \t",wd[i](0));
	}cout<<endl;
	
	cout<<"wd:\t";
	for (i=0; i<wd.size(); i++){
		printf("%+2.4f  \t",wd[i](1));
	}cout<<endl;
	
	cout<<"  \t";
	for (i=0; i<wd.size(); i++){
		printf("%+2.4f  \t",wd[i](2));
	}cout<<endl;
	cout<<endl;
	
	//vd
	cout<<"  \t";
	for (i=0; i<vd.size(); i++){
		printf("%+2.4f  \t",vd[i](0));
	}cout<<endl;
	
	cout<<"vd:\t";
	for (i=0; i<vd.size(); i++){
		printf("%+2.4f  \t",vd[i](1));
	}cout<<endl;
	
	cout<<"  \t";
	for (i=0; i<vd.size(); i++){
		printf("%+2.4f  \t",vd[i](2));
	}cout<<endl;
	cout<<endl;
	
	//vdC
	cout<<"  \t";
	for (i=0; i<vdC.size(); i++){
		printf("%+2.4f  \t",vdC[i](0));
	}cout<<endl;
	
	cout<<"vdC:\t";
	for (i=0; i<vdC.size(); i++){
		printf("%+2.4f  \t",vdC[i](1));
	}cout<<endl;
	
	cout<<"  \t";
	for (i=0; i<vdC.size(); i++){
		printf("%+2.4f  \t",vdC[i](2));
	}cout<<endl;
	cout<<endl;
	
	// F
	
	cout<<"  \t";
	for (i=0; i<F.size(); i++){
		printf("%+2.4f  \t",F[i](0));
	}cout<<endl;
	
	cout<<"F:\t";
	for (i=0; i<F.size(); i++){
		printf("%+2.4f  \t",F[i](1));
	}cout<<endl;
	
	cout<<"  \t";
	for (i=0; i<F.size(); i++){
		printf("%+2.4f  \t",F[i](2));
	}cout<<endl;
	cout<<endl;
	
	// N
	cout<<"  \t";
	for (i=0; i<Nout.size(); i++){
		printf("%+2.4f  \t",Nout[i](0));
	}cout<<endl;
	
	cout<<"N:\t";
	for (i=0; i<Nout.size(); i++){
		printf("%+2.4f  \t",Nout[i](1));
	}cout<<endl;
	
	cout<<"  \t";
	for (i=0; i<Nout.size(); i++){
		printf("%+2.4f  \t",Nout[i](2));
	}cout<<endl;
	cout<<endl;
}

void NewtonEulerDynamics::printin(){
	unsigned int i;
	
	// f
	cout<<"  \t";
	for (i=0; i<f.size(); i++){
		printf("%+2.4f  \t",f[i](0));
	}cout<<endl;
	
	cout<<"f:\t";
	for (i=0; i<f.size(); i++){
		printf("%+2.4f  \t",f[i](1));
	}cout<<endl;
	
	cout<<"  \t";
	for (i=0; i<f.size(); i++){
		printf("%+2.4f  \t",f[i](2));
	}cout<<endl;
	cout<<endl;
	
	// n
	cout<<"  \t";
	for (i=0; i<n.size(); i++){
		printf("%+2.4f  \t",n[i](0));
	}cout<<endl;
	
	cout<<"n:\t";
	for (i=0; i<n.size(); i++){
		printf("%+2.4f  \t",n[i](1));
	}cout<<endl;
	
	cout<<"  \t";
	for (i=0; i<n.size(); i++){
		printf("%+2.4f  \t",n[i](2));
	}cout<<endl;
	cout<<endl;
	
	// tau
	cout<<"tau:\t";
	for (i=0; i<tau.size(); i++){
		printf("%+2.4f  \t",tau[i]);
	}cout<<endl;
	cout<<endl;
}

void NewtonEulerDynamics::printT(const Transform3D<double> &t, double b){
	int i,j;
	double a;
	
	for(i=0; i<3; i++){
		for (j=0; j<4; j++){
			if(j!=3){
				a = (t.R())(i,j);
				if (fabs(a)<b){
					a = 0;
				}
				cout<<a<<"\t";
			}
			else{
				cout<<(t.P())(i);
			}
		}
		cout<<endl;
	}
	cout<<"0\t0\t0\t1"<<endl;
	cout<<endl;
}
