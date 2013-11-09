/*
 * ModelPropertyTest.cpp
 *
 *  Created on: 14-05-2009
 *      Author: jimali
 */
#include <fstream>

#include <rw/graspplanning/GraspTable.hpp>

using namespace rw::graspplanning;
using namespace rw::math;
using namespace rw::sensor;

int main(int argc, char** argv)
{


	if( argc < 3 ){
		std::cout << "------ Usage: " << std::endl;
        std::cout << "- Arg 1 name of danni table" << std::endl;
		std::cout << "- Arg 2 name of grasp table\n" << std::endl;

	    return 0;
	}

	std::string danniFile(argv[1]);
	std::string filename(argv[2]);

	//
    std::ifstream in;
    in.open(danniFile.c_str());
    const int LINE_SIZE=10000;
    char line[LINE_SIZE];
    in.getline(line, LINE_SIZE);
/*
    1: timestamp,
    2: millisekunder,
    3-5: Force x, y, z,
    6-8: torque x, y, z,
    9-14: Robot State (q0-q5)
    15: Grip closed (bool)
    16-22: SDH State (q0-q6)
    23-508: Tactile data.
    509-515: Trakstar Position (x, y, z) + rotation (qw, qx, qy, qz) (hånden)
    516-522: Trakstar Position (x, y, z) + rotation (qw, qx, qy, qz) ( base-plate)
    523-529: Trakstar Position (x, y, z) + rotation (qw, qx, qy, qz) ( Peg)
*/
    GraspTable gtable("SchunkHand", "object");
    GraspTable::GraspData gdata;


    int count = 0;
    char comma;
    long timestamp, timestamp_ms, gripClosed;
    double force[3], torque[3], robot_q[6], sdh_q[7], tactile[486], track[21];
    while(!in.eof()){
        in >> timestamp >> comma >> timestamp_ms >> comma;
        for(int i=0;i<3;i++) in >> force[i] >> comma;
        for(int i=0;i<3;i++) in >> torque[i] >> comma;
        for(int i=0;i<6;i++) in >> robot_q[i] >> comma;
        in >> gripClosed >> comma;
        for(int i=0;i<7;i++) in >> sdh_q[i] >> comma;

        for(int i=0;i<486;i++) in >> tactile[i] >> comma;
        for(int i=0;i<21;i++) in >> track[i] >> comma;
        // perhaps we need to skip newline, but i don't think so
        //std::cout << "\r" << count << "    ";
        count++;
        std::cout << count << ": " << timestamp<< ":" << timestamp_ms << ":" << tactile[485]<< ":" << track[20] << "\n";

        gdata.cq = Q(7, sdh_q);
        // copy tactile data 13x6, 13x6, 13x6, 14x6, 14x6 14x6
        int prox1 =          0, dist1 = 14*6;
        int prox2 = dist1+13*6, dist2 = prox2+14*6;
        int prox3 = dist2+13*6, dist3 = prox3+14*6;


        gdata._tactiledata.resize(6);
        TactileArray::ValueMatrix distal1(13,6),distal2(13,6),distal3(13,6);
        for(int x=0;x<13;x++)
            for(int y=0;y<6;y++){
                distal1(x,y) = tactile[x*6+y + dist1];
                distal2(x,y) = tactile[x*6+y + dist2];
                distal3(x,y) = tactile[x*6+y + dist3];
            }

        TactileArray::ValueMatrix proximal1(14,6),proximal2(14,6),proximal3(14,6);
        for(int x=0;x<14;x++)
            for(int y=0;y<6;y++){
                proximal1(x,y) = tactile[x*6+y + prox1];
                proximal2(x,y) = tactile[x*6+y + prox2];
                proximal3(x,y) = tactile[x*6+y + prox3];
            }
        gdata._tactiledata[0] = distal1;
        gdata._tactiledata[1] = distal2;
        gdata._tactiledata[2] = distal3;
        gdata._tactiledata[3] = proximal1;
        gdata._tactiledata[4] = proximal2;
        gdata._tactiledata[5] = proximal3;

        gtable.addGrasp( gdata );

    }
    std::cout << "\n The count: " << count << std::endl;


	gtable.save(filename);
	return 0;
}
