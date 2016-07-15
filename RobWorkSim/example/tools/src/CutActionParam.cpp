#include "CutActionParam.hpp"

#include <fstream>

using rw::math::Q;

void CutActionParam::save(const std::vector<CutActionParam>& params, const std::string& file){
    std::ofstream ostr;
    ostr.open(file.c_str());

    ostr<< "# each line define a parameterization\n"
        << "# pose is defined by (position in meter, orientation in ZYX euler)\n"
        << "# Object pose data in meter and radians: pos_x pos_y pos_z rpy_r rpy_p rpy_y\n"
        << "# Knife pose data in meter and radians: pos_x pos_y pos_z rpy_r rpy_p rpy_y \n"
        << "# direction in world: dir_x dir_y dir_z\n"
        << "# gripper pos and rpy\n"
        << "# gripper configuration: length q1 q2 q3 .... q{length}\n\n";

    for(std::size_t i=0;i<params.size();i++){
        const CutActionParam &param = params[i];

        ostr << param.posObj[0] << " " << param.posObj[1]<< " " << param.posObj[2]<< " ";
        ostr << param.rpyObj[0] << " "<< param.rpyObj[1] << " "<< param.rpyObj[2]<< "   ";
        ostr << param.pos[0] << " "<< param.pos[1] << " "<< param.pos[2]<< " ";
        ostr << param.rpy[0]<< " " << param.rpy[1] << " "<< param.rpy[2]<< "   ";

        ostr << param.dir[0] << " "<< param.dir[1] << " "<< param.dir[2]<< " ";
        ostr << param.len<< "    ";

        ostr << param.posGripper[0] << " "<< param.posGripper[1] << " "<< param.posGripper[2]<< " ";
        ostr << param.rpyGripper[0] << " "<< param.rpyGripper[1] << " "<< param.rpyGripper[2]<< "   ";
        int qlen = param.gripperQ.size();
        ostr << qlen << " ";
        for (int i = 0; i < qlen; i++)
            ostr << param.gripperQ(i) << " ";
        ostr << "\n";
    }
}

std::vector<CutActionParam> CutActionParam::load(const std::string& file)
{
    std::vector<CutActionParam> params;
    std::fstream fstr;
    fstr.open(file.c_str());
    std::string line;
    while (!fstr.eof()) {
        CutActionParam param;
        getline(fstr, line);
        if (line[0] == '#') continue;
        if (line.size() < 10) continue;

        std::stringstream sstr(line);

        sstr >> param.posObj[0] >> param.posObj[1] >> param.posObj[2];
        sstr >> param.rpyObj[0] >> param.rpyObj[1] >> param.rpyObj[2];
        sstr >> param.pos[0] >> param.pos[1] >> param.pos[2];
        sstr >> param.rpy[0] >> param.rpy[1] >> param.rpy[2];

        sstr >> param.dir[0] >> param.dir[1] >> param.dir[2];
        sstr >> param.len;

        sstr >> param.posGripper[0] >> param.posGripper[1] >> param.posGripper[2];
        sstr >> param.rpyGripper[0] >> param.rpyGripper[1] >> param.rpyGripper[2];
        int qlen;
        sstr >> qlen;
        param.gripperQ = Q(qlen);
        for (int i = 0; i < qlen; i++)
            sstr >> param.gripperQ(i);

        params.push_back(param);
    }
    return params;
}
