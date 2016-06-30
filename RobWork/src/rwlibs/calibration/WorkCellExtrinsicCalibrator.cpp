#include "WorkCellExtrinsicCalibrator.hpp"
#include "CalibrationMeasurement.hpp"
#include "WorkCellCalibration.hpp"

#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/EAA.hpp>
#include <rw/models/SerialDevice.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>

//#include <iomanip>


using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

using namespace rwlibs::calibration;

WorkCellExtrinsicCalibrator::WorkCellExtrinsicCalibrator(rw::models::WorkCell::Ptr workcell) :
	_workcell(workcell) 
{

}

WorkCellExtrinsicCalibrator::~WorkCellExtrinsicCalibrator() {

}

rw::models::WorkCell::Ptr WorkCellExtrinsicCalibrator::getWorkCell() const {
	return _workcell;
}

void WorkCellExtrinsicCalibrator::setMeasurements(const std::vector<CalibrationMeasurement::Ptr>& measurements) {
	_measurements = measurements;
}

void WorkCellExtrinsicCalibrator::calibrate(WorkCellCalibration::Ptr workcellCalibration) {
	//Initially we need to sort the data according to device and frame
	typedef std::map<std::string, std::vector<CalibrationMeasurement::Ptr> > StringMeasurementMap;
	StringMeasurementMap sortedMeasurements;

	BOOST_FOREACH(CalibrationMeasurement::Ptr measurement, _measurements) {
		sortedMeasurements[measurement->getDeviceName()].push_back(measurement);
	}

	std::vector<Device2SensorResult> results;	
	for (StringMeasurementMap::iterator it = sortedMeasurements.begin(); it != sortedMeasurements.end(); ++it) {
		//std::cout<<"Calibrate "<<(*it).first<<" which has "<<(*it).second.size()<<" measurements "<<std::endl;		
		calibrateForSingleDevice((*it).first, (*it).second, results);
	}

	//Extract the needed information from the results

	//Find the tool offset for each device
	std::map<std::string, Vector3D<> > positions;
	std::map<std::string, Vector3D<> > eaas;
	std::map<std::string, int> cnts;
	BOOST_FOREACH(Device2SensorResult res, results) {
		EAA<> eaa(res.tool2marker.R());
		Vector3D<> eaaVector(eaa(0), eaa(1), eaa(2));
		if (positions.find(res.device) != positions.end()) {
			positions[res.device] += res.tool2marker.P() * res.cnt;
			eaas[res.device] += eaaVector * res.cnt;
			cnts[res.device] += res.cnt;
		}
		else {
			positions[res.device] = res.tool2marker.P() * res.cnt;
			eaas[res.device] = eaaVector * res.cnt;
			cnts[res.device] = res.cnt;
		}
	}

	for (std::map<std::string, Vector3D<> >::iterator it = positions.begin(); it != positions.end(); ++it) {
		const std::string name = (*it).first;
		Vector3D<> pos = positions[name] /= (double)cnts[name];
		Vector3D<> eaavec = eaas[name] /= (double)cnts[name];
		//std::cout<<"Marker "<<(*it).first<<" Correction Pos = "<<pos<<"  EAA="<<eaavec<<std::endl;
		workcellCalibration->getFixedFrameCalibrationForMarker((*it).first)->setCorrectionTransform(Transform3D<>(pos, EAA<>(eaavec(0), eaavec(1), eaavec(2)))); 		
	}


	//Find the offset of the sensors
	std::string primaryDevice = workcellCalibration->getDeviceMarkerPairs().front().first->getName();
	//std::cout<<"Primary Device = "<<primaryDevice<<std::endl;	
	std::map<std::string, Transform3D<> > primaryDevice2Sensor;
	std::map<std::pair<std::string, std::string>, Transform3D<> > secondaryDeviceTransforms;
	BOOST_FOREACH(Device2SensorResult res, results) {		
		//std::cout<<"Result: "<<res.device<<std::endl;
		//std::cout<<" base2sensor = "<<res.sensor2base<<std::endl;
		//std::cout<<" end2marker = "<<res.tool2marker<<std::endl;

		if (res.device == primaryDevice) {
			Frame* baseFrame = _workcell->findDevice(res.device)->getBase();
			Frame* sensorFrame = _workcell->findFrame(res.sensor);
			Transform3D<> Tsensor2base = Kinematics::frameTframe(sensorFrame, baseFrame, _workcell->getDefaultState());
			Transform3D<> Tcorrection = Tsensor2base * inverse(res.sensor2base);
			workcellCalibration->getFixedFrameCalibrationForSensor(res.sensor)->setCorrectionTransform(Tcorrection);
			//std::cout<<"Sensor "<<res.sensor<<" correction = "<<Tcorrection<<std::endl;
			// Transform3D<> Tbase2sensor= Kinematics::frameTframe(baseFrame, sensorFrame, _workcell->getDefaultState()); // Not used
			primaryDevice2Sensor[res.sensor] = Tcorrection;
		} else {
			if (secondaryDeviceTransforms.find(std::make_pair(res.device, res.sensor)) != secondaryDeviceTransforms.end()) {
				RW_THROW("For some reason the device sensor pair "<<res.device<<" and "<<res.sensor<<" is represented several times in the result!");
			}
			secondaryDeviceTransforms[std::make_pair(res.device, res.sensor)] = res.sensor2base;
		}
	}

	//Runs through the device, sensor pairs and compute the corresponding corrections.
	std::map<std::string, std::vector<Transform3D<> > > deviceBaseCorrections;
	for (std::map<std::pair<std::string, std::string>, Transform3D<> >::iterator it = secondaryDeviceTransforms.begin(); it != secondaryDeviceTransforms.end(); ++it) {
		const std::string& deviceName = (*it).first.first;
		const std::string& sensorName = (*it).first.second;
		const Transform3D<>& sensor2base = (*it).second;
		
		Frame* baseFrame = _workcell->findDevice(deviceName)->getBase();
		Frame* sensorFrame = _workcell->findFrame(sensorName);
		Transform3D<> Tsensor2baseModel = Kinematics::frameTframe(sensorFrame, baseFrame, _workcell->getDefaultState());
		Transform3D<> Tcorrection = inverse(Tsensor2baseModel)*primaryDevice2Sensor[sensorName] * sensor2base;
		//std::cout<<"Sensor Correction for Secondary Device = "<<Tcorrection<<std::endl;		
		deviceBaseCorrections[deviceName].push_back(Tcorrection);
	}

	//Average the corrections
	for (std::map<std::string, std::vector<Transform3D<> > >::iterator it = deviceBaseCorrections.begin(); it != deviceBaseCorrections.end(); ++it) {
		const std::vector<Transform3D<> >& corrections = (*it).second;
		Vector3D<> eaas(0,0,0);
		Vector3D<> pos(0,0,0);
		BOOST_FOREACH(const Transform3D<>& correction, corrections) {
			EAA<> eaa(correction.R());
			eaas += eaa.axis() * eaa.angle();
			pos += correction.P();
		}
		eaas /= (*it).second.size();
		pos /= (*it).second.size();
		Transform3D<> Tcorrection(pos, EAA<>(eaas(0), eaas(1), eaas(2)));
	//	std::cout<<"Final Correction for Secondary Device = "<<Tcorrection<<std::endl;		
		workcellCalibration->getFixedFrameCalibrationForDevice((*it).first)->setCorrectionTransform(Tcorrection);
	}

}

void WorkCellExtrinsicCalibrator::calibrateForSingleDevice(const std::string& deviceName, const std::vector<CalibrationMeasurement::Ptr>& measurements, std::vector<Device2SensorResult>& results) {
	typedef std::map<std::string, std::vector<CalibrationMeasurement::Ptr> > StringMeasurementMap;
	StringMeasurementMap sortedMeasurements;

	BOOST_FOREACH(CalibrationMeasurement::Ptr measurement, measurements) {
		//std::cout<<"\tMeasurement = "<<measurement->getDeviceName()<<" "<<measurement->getMarkerFrameName()<<" "<<measurement->getSensorFrameName()<<" "<<measurement->getQ()(0)<<std::endl;
		sortedMeasurements[measurement->getSensorFrameName()].push_back(measurement);
	}
//	std::cout<<"Measurement Count = "<<measurements.size()<<std::endl;
//	int cnt = 0; // Not used
	for (StringMeasurementMap::iterator it = sortedMeasurements.begin(); it != sortedMeasurements.end(); ++it) {
		Device2SensorResult result;
		result.device = deviceName;
		result.sensor = (*it).first; //The sensor name
		//std::pair<Transform3D<>, Transform3D<> > res = 
//		std::cout<<"Calibrate "<<deviceName<<" and "<<(*it).first<<" with "<<(*it).second.size()<<" measurements "<<std::endl;
		calibrateSingleDeviceAndSensor((*it).second, result);
		results.push_back(result);
		//std::cout<<"Result = "<<result.device<<std::endl;
		//std::cout<<"\tsensor = "<<result.sensor<<std::endl;
		//std::cout<<"\tsensor2base = "<<result.sensor2base<<std::endl;
		//std::cout<<"\ttool2marker = "<<result.tool2marker<<std::endl;
	}
}

Transform3D<> WorkCellExtrinsicCalibrator::getFK(const std::string& device, const std::string& markerFrame, const Q& q) {
	SerialDevice::Ptr dev = _workcell->findDevice<SerialDevice>(device);
	Frame* marker = _workcell->findFrame(markerFrame);
	State state = _workcell->getDefaultState();
	dev->setQ(q, state);
	//return dev->baseTframe(dev->getJoints().back(), state);
	return dev->baseTframe(marker, state);
	//return dev->baseTend(state);	
}

Transform3D<> WorkCellExtrinsicCalibrator::getFK(CalibrationMeasurement::Ptr measurement) {
	return getFK(measurement->getDeviceName(), measurement->getMarkerFrameName(), measurement->getQ());
}

void WorkCellExtrinsicCalibrator::calibrateSingleDeviceAndSensor(const std::vector<CalibrationMeasurement::Ptr>& measurements, Device2SensorResult& result) 
{
	//Perform the HGP pre calibration 

	//Compute the initial estimate of the base transformation
	//1: Compute p_avg = 1/K Sum(p_k)
	const unsigned int K = measurements.size();
	Eigen::VectorXd p_avg = Eigen::VectorXd::Zero(3);
	BOOST_FOREACH(CalibrationMeasurement::Ptr measurement, measurements) {
		//std::cout<<"p org="<<measurement->getTransform().P()<<std::endl;
		p_avg += measurement->getTransform().P().e();
		//std::cout<<"Measurement: "<<measurement->getTransform().P()<<std::endl;
	}
	p_avg /= K;
	//std::cout<<"p_avg = "<<p_avg<<std::endl;
	//char ch[4]; std::cin.getline(ch, 1);
	//2: Compute p_k
	std::vector<Eigen::MatrixXd> p_k;
	BOOST_FOREACH(CalibrationMeasurement::Ptr measurement, measurements) {		
		p_k.push_back( (measurement->getTransform().P().e() - p_avg) );
	//	std::cout<<"p k "<<p_k.back()<<std::endl;
	}
	//Ok hertil
	//3, 4: Compute Ai,j, Bi,j, alphai,j, betai,j
	Transform3D<>** Aij = new Transform3D<>*[K];
	Transform3D<>** Bij = new Transform3D<>*[K];
	EAA<>** alphaij = new EAA<>*[K];
	EAA<>** betaij = new EAA<>*[K];
//	std::cout<<"=================="<<std::endl;
	for (size_t i = 0; i<K; i++) {
		//if (i < 2) {
		//	std::cout<<i<<": Q = "<<measurements[i]->getQ()<<std::endl<<std::endl;
		//	std::cout<<i<<": FK = "<<getFK(measurements[i])<<std::endl<<std::endl;
		//	std::cout<<i<<": Measurement = "<<measurements[i]->getTransform()<<std::endl<<std::endl;
		//}
		Aij[i] = new Transform3D<>[K];
		Bij[i] = new Transform3D<>[K];
		alphaij[i] = new EAA<>[K];
		betaij[i] = new EAA<>[K];
		for (size_t j = 0; j<K; j++) {
			//if (j < 2) {
			//	std::cout<<j<<": Q = "<<measurements[j]->getQ()<<std::endl<<std::endl;
			//	std::cout<<j<<": FK = "<<getFK(measurements[j])<<std::endl<<std::endl;
			//	std::cout<<j<<": Measurement = "<<measurements[j]->getTransform()<<std::endl<<std::endl;
			//}

			Aij[i][j] = measurements[i]->getTransform() * inverse(measurements[j]->getTransform());
			//std::cout<<"Measumement["<<i<<"] = "<<measurements[i]->getTransform()<<std::endl;
			//std::cout<<"Measumement["<<j<<"] = "<<measurements[j]->getTransform()<<std::endl;			
			Bij[i][j] = getFK(measurements[i]) * inverse(getFK(measurements[j]));
			alphaij[i][j] = EAA<>(Aij[i][j].R());
			betaij[i][j] = EAA<>(Bij[i][j].R());
			//if (i == 0 && j == 1) 
			//{
			//	std::cout<<"Aij["<<i<<","<<j<<"] = "<<Aij[i][j]<<std::endl;
			//	std::cout<<"Bij["<<i<<","<<j<<"] = "<<Bij[i][j].P()<<std::endl;
			//	std::cout<<"alphaij["<<i<<","<<j<<"] = "<<alphaij[i][j]<<std::endl;
			//	std::cout<<"betaij["<<i<<","<<j<<"] = "<<betaij[i][j]<<std::endl;
			//	char ch[4]; std::cin.getline(ch, 1);
			//}
			
		}
	}


	//7,8,9,10: Compute alpha_k, beta_k, Gamma_k and pi_k
	std::vector<Eigen::VectorXd> alpha_k; //Average of the EAA
	std::vector<Eigen::VectorXd> beta_k; //Average of the EAA angles
	std::vector<Eigen::MatrixXd> Gamma_k; //Average of the rotation matrices
	std::vector<Eigen::VectorXd> pi_k; //Average of the translation matrices
	for (size_t k = 0; k<K; k++) {
		Eigen::VectorXd alpha = Eigen::VectorXd::Zero(3);
		Eigen::VectorXd beta = Eigen::VectorXd::Zero(3);
		Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(3,3);
		Eigen::VectorXd p = Eigen::VectorXd::Zero(3);
		for (size_t i = 0; i<K; i++) {
			alpha(0) += alphaij[i][k](0);
			alpha(1) += alphaij[i][k](1);
			alpha(2) += alphaij[i][k](2);

			beta(0) += betaij[i][k](0);
			beta(1) += betaij[i][k](1);
			beta(2) += betaij[i][k](2);

			Gamma += Bij[i][k].R().e();
			p += Bij[i][k].P().e();
		}
		alpha /= (double)K;
		alpha_k.push_back(alpha);
		beta /= (double)K;
		beta_k.push_back(beta);
//		std::cout<<"Gamma Org = "<<Gamma<<std::endl;
		Gamma /= (double)K;
		Gamma_k.push_back(Gamma);	
		p /= (double)K;
		pi_k.push_back(p);
		//if (k == 1) {
		//	std::cout<<"alpha["<<k<<"] = "<<alpha.transpose()<<std::endl;
		//	std::cout<<"beta["<<k<<"] = "<<beta.transpose()<<std::endl;
		//	std::cout<<"Gamma["<<k<<"] = "<<Gamma<<std::endl;
		//	std::cout<<"p["<<k<<"] = "<<p.transpose()<<std::endl;
		//	char ch[2]; std::cin.getline(ch, 1);
		//}
		
	}
	//BOOST_FOREACH(const Eigen::MatrixXd& ra, RA_k) {		
	//	Rotation3D<> R(ra);
	//	EAA<> eaa(R);
	//	Eigen::MatrixXd m(3,1);
	//	m(0,0) = eaa(0);
	//	m(1,0) = eaa(1);
	//	m(2,0) = eaa(2);

	//	alpha_k.push_back(m);
	//}

	////6: Compute beta_k
	//std::vector<Eigen::MatrixXd> beta_k;
	//BOOST_FOREACH(const Eigen::MatrixXd& rb, RB_k) {
	//	Rotation3D<> R(rb);
	//	EAA<> eaa(R);
	//	Eigen::MatrixXd m(3,1);
	//	m(0,0) = eaa(0);
	//	m(1,0) = eaa(1);
	//	m(2,0) = eaa(2);
	//	beta_k.push_back(m);
	//}

	//11: Define the matrix P
	Eigen::MatrixXd P(3, K);
	for (size_t i = 0; i<K; i++) {
		const Eigen::MatrixXd& p = p_k[i];
		P(0, i) = p(0,0);
		P(1, i) = p(1,0);
		P(2, i) = p(2,0);
	}
	//std::cout<<"P="<<P<<std::endl;
	
	//12: Compute U = P^T (P P^T)^-1 P
	Eigen::MatrixXd U = P.transpose() * (P * P.transpose()).inverse() * P;
	//std::cout<<"U = "<<U<<std::endl;
	//13, 14: Compute Z(k) and z(k)
	std::vector<Eigen::MatrixXd> Zk;
	std::vector<Eigen::MatrixXd> zk;
	for (size_t k = 0; k<K; k++) {
		Eigen::MatrixXd Z(Eigen::MatrixXd::Zero(3,3));
		Eigen::MatrixXd z(Eigen::MatrixXd::Zero(3,1));
		for (size_t i = 0; i<K; i++) {
			const Eigen::MatrixXd I_Gammai_inv = (Eigen::MatrixXd::Identity(3,3) - Gamma_k[i].inverse());
			const Eigen::MatrixXd I_Gammak_inv = (Eigen::MatrixXd::Identity(3,3) - Gamma_k[k]).inverse();
			//Z += U(k, i)*I_Gammai_inv * I_Gammak_inv * Gamma_k[k];
			Z += U(k, i)*I_Gammai_inv * (Eigen::MatrixXd::Identity(3,3) - Gamma_k[k].inverse()).inverse() * Gamma_k[k];
			z += U(k, i)*( (-1*(Gamma_k[i].inverse()*pi_k[i]))-I_Gammai_inv*(Eigen::MatrixXd::Identity(3,3) - Gamma_k[k]).inverse()*pi_k[k]);
		}

		Zk.push_back(Z);
		zk.push_back(z);
		
	}

	//15: Compute v_k
	std::vector<Eigen::MatrixXd> v_k;
	for (size_t k = 0; k<K; k++) {
		Eigen::MatrixXd v = (Eigen::MatrixXd::Identity(3,3) - Zk[k]).inverse() * zk[k];
		v_k.push_back(v);
	//	std::cout<<"vk = "<<v_k.back()<<std::endl;
	}
	

	//std::ofstream outfile("d:\\temp\\MComponents.txt");

	//Step 16: Compute M
	Eigen::MatrixXd M(Eigen::MatrixXd::Zero(3,3));
	for (size_t k = 0; k<K; k++) {
		//outfile<<"beta["<<k<<"] = "<<beta_k[k]<<std::endl;
		//outfile<<"alpha["<<k<<"] = "<<alpha_k[k]<<std::endl;
		//outfile<<"v["<<k<<"] = "<<v_k[k]<<std::endl;
		//outfile<<"p["<<k<<"] = "<<p_k[k]<<std::endl;
		//outfile<<"z["<<k<<"] = "<<zk[k]<<std::endl;
		//outfile<<"Z["<<k<<"] = "<<Zk[k]<<std::endl;
		//outfile<<"Gamma["<<k<<" = "<<Gamma_k[k]<<std::endl<<std::endl;
		if (_useRotation)
			M += beta_k[k]*alpha_k[k].transpose();
		if (_usePosition)
			M += v_k[k]*p_k[k].transpose();
		//M += (beta_k[k]*alpha_k[k].transpose() + p_k[k]*v_k[k].transpose());
		//M += (beta_k[k]*alpha_k[k].transpose() + v_k[k]*p_k[k].transpose());
	}
	//std::cout<<"UseRotation: "<<_useRotation<<std::endl;
	//std::cout<<"UsePosition: "<<_usePosition<<std::endl;
	//std::cout<<"M = "<<std::setprecision(16)<<M<<std::endl;
	//
	//Step 17: Compute RX=(M^T M)^(-1/2) M^T
	Eigen::MatrixXd MTM = M.transpose()*M;
	//std::cout<<"MTM = "<<MTM<<std::endl;
	const Eigen::JacobiSVD<Eigen::MatrixXd> svd = MTM.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::MatrixXd Usvd = svd.matrixU();
	Eigen::VectorXd sigma = svd.singularValues();
	Eigen::MatrixXd Vsvd = svd.matrixV();
	//std::cout<<"SVD U = "<<Usvd<<std::endl;
	//std::cout<<"SVD sigma = "<<sigma<<std::endl;
	//std::cout<<"SVD V = "<<Vsvd<<std::endl;
	for (int i = 0; i<sigma.size(); i++) {
		sigma(i) = sqrt(sigma(i));
	}

	//std::cout<<"SVD Sqrt = "<<Usvd*Eigen::DiagonalMatrix<double, Eigen::Dynamic>(sigma)*Vsvd<<std::endl;
	//std::cout<<"SVD Sqrt = "<<Usvd*Eigen::DiagonalMatrix<double, Eigen::Dynamic>(sigma)*Vsvd.transpose()<<std::endl;
	Eigen::MatrixXd sqrtM = Usvd*Eigen::DiagonalMatrix<double, Eigen::Dynamic>(sigma)*Vsvd.transpose();
	//Eigen::LDLT<Eigen::MatrixXd> ldl = MTM.ldlt();
	//Eigen::Diagonal<const Eigen::MatrixXd> diagonal = ldl.vectorD();
	//Eigen::MatrixXd sqrtDiagonal = Eigen::MatrixXd::Zero(diagonal.size(), diagonal.size());
	//for (int i = 0; i<diagonal.size(); i++) {
	//	sqrtDiagonal(i, i) = sqrt(diagonal(i));
	//}
	//Eigen::Transpositions<Eigen::Dynamic> transposition = ldl.transpositionsP();

	/*std::cout<<"TransitionP = "<<transposition*Eigen::MatrixXd(ldl.matrixL())<<std::endl;
	std::cout<<"TransitionP = "<<Eigen::MatrixXd(ldl.matrixU())*transposition<<std::endl;
	std::cout<<"Inverse ? = "<<transposition.transpose()*Eigen::MatrixXd(ldl.matrixL())*sqrtDiagonal*Eigen::MatrixXd(ldl.matrixU())*transposition<<std::endl;
	std::cout<<"ldl.matrixLDLT"<<ldl.matrixLDLT()<<std::endl;	
	std::cout<<"Lower = "<<Eigen::MatrixXd(ldl.matrixL())<<std::endl;
	std::cout<<"Diagonal = "<<diagonal<<std::endl;
	std::cout<<"Upper = "<<Eigen::MatrixXd(ldl.matrixU())<<std::endl;*/
//	Eigen::MatrixXd tl = ldl.transpositionsP().transpose() * ldl.matrixL();
//	Eigen::MatrixXd sqrtM =  ldl.matrixL() * sqrtDiagonal * ldl.matrixU();
	//std::cout<<"sqrtM = "<<sqrtM<<std::endl;
	Eigen::MatrixXd RX = (sqrtM).inverse()*M.transpose();
	//std::cout<<"RX = "<<std::setprecision(16)<<RX<<std::endl;
	//std::cout<<"Det(RX) = "<<RX.determinant()<<std::endl;

	//Step 18: Compute C as 3K x 3 stacked with (I-RBk) RX^T
	Eigen::MatrixXd C(3*K, 3);
	for (size_t k = 0; k<K; k++) {
		Eigen::MatrixXd I_Gamma_RX = (Eigen::MatrixXd::Identity(3,3) - Gamma_k[k]) * RX.transpose();
		for (size_t i = 0; i<3; i++) {
			for (size_t j = 0; j<3; j++) {
				C(3*k+i, j) = I_Gamma_RX(i,j);
			}
		}		
	}

	//Step 19: Compute c as the 3K vector based on -(pbk+RBk RX^T pk)
	Eigen::MatrixXd c(3*K,1);
	for (size_t k = 0; k<K; k++) {
		Eigen::MatrixXd a = -(pi_k[k]+ Gamma_k[k]*RX.transpose()*p_k[k]);
		for (size_t i = 0; i<3; i++) {
			c(3*k + i, 0) = a(i);
		}
	}

	//Step 20: Compute px=(C^T C)^-1 C^T c+ p_avg
	Eigen::MatrixXd px = (C.transpose()*C).inverse()*C.transpose() * c + p_avg;
	//std::cout<<"px = "<<px<<std::endl;
	//Find the Tbase2cam
        /* mband: commented the following to get rid of a warning regarding unused variable, and this functionality doesn't seem to do anything, unless it relies on non-intuitive side-effects... */
	// Transform3D<> TbaseCorrection = Transform3D<>(Vector3D<>(px), Rotation3D<>(RX));
	// for (size_t k = 0; k<K; k++) {
	// 	CalibrationMeasurement::Ptr measurement = measurements[k];
	// 	Transform3D<> fk = getFK(measurement);
	// 	Transform3D<> Ttcp = inverse(TbaseCorrection*fk) * measurement->getTransform();
	// }

	Transform3D<> Tbase = Transform3D<>(Vector3D<>(px), Rotation3D<>(RX));
	//std::cout<<"Tbase = "<<Tbase<<std::endl;
	//Find the average TnTCP
	std::vector<Transform3D<> > tntcps;
	Vector3D<> posTool(0,0,0);
	Vector3D<> eaaAxis(0,0,0);
	double eaaAngle = 0;
	for (size_t k = 0; k<K; k++) {
		Transform3D<> fk = Tbase;
		Transform3D<> fk2 = getFK(measurements[k]); 
		//std::cout<<"fk = "<<fk<<std::endl;
		//std::cout<<"fk2 = "<<fk2<<std::endl;
		//std::cout<<"Measurement = "<<(measurements[k]->getTransform())<<std::endl;
		//std::cout<<"q = "<< measurements[k]->getQ()<<std::endl;
		//std::cout<<"fk total = "<<fk*fk2<<std::endl;
		Transform3D<> tntcp = inverse(fk*fk2) * (measurements[k]->getTransform());
		//std::cout<<"TNTCP = "<<tntcp<<std::endl;
		posTool += tntcp.P();
		EAA<> eaa(tntcp.R());
		//std::cout<<"eaa["<<k<<"] = "<<eaa<<std::endl;
		eaaAxis += eaa.axis();
		eaaAngle += eaa.angle();
		//eaaTool(0) += eaa(0);
		//eaaTool(1) += eaa(1);
		//eaaTool(2) += eaa(2);
		//char ch[4]; std::cin.getline(ch, 1);
	}
	posTool /= (double)K;
	eaaAxis /= (double)K;
	eaaAxis = normalize(eaaAxis);
	eaaAngle /= (double)K;
	//std::cout<<"posTool = "<<posTool<<std::endl;
	//std::cout<<"eaaTool = "<<eaaAxis<<"  "<<eaaAngle<<std::endl;
	//std::cout<<"Ttool = "<<Transform3D<>(posTool, EAA<>(eaaAxis, eaaAngle))<<std::endl;
	result.sensor2base = Tbase;
	result.tool2marker = Transform3D<>(posTool, EAA<>(eaaAxis, eaaAngle));
	result.cnt = measurements.size();
	//return std::make_pair(Tbase, Transform3D<>(posTool, EAA<>(eaaTool(0), eaaTool(1), eaaTool(2))));


	for (size_t i = 0; i<K; ++i) {
		delete[] Aij[i];
		delete[] Bij[i];
		delete[] alphaij[i];
		delete[] betaij[i];
	}

	delete[] Aij;
	delete[] Bij;
	delete[] alphaij;
	delete[] betaij;



}
